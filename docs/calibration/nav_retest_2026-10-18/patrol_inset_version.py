#!/usr/bin/env python
"""Supervised fixed-scene AMCL patrol; one resident TCP owner, no Serial access."""
from __future__ import division
import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0,os.path.join(ROOT,'integration'))
from g2_patrol_policy import Policy,wrap,finite,guided_heading,stop_reserve
from g2_geometry import LIDAR_FORWARD_M


class Transient(RuntimeError):
    pass


def rpc(command,path='/tmp/grasp_health.sock'):
    s=socket.socket(socket.AF_UNIX);s.settimeout(2)
    try:
        s.connect(path);s.sendall((command+'\n').encode('ascii'))
        return json.loads(s.makefile('r').readline())
    finally:
        s.close()


def available_mb():
    with open('/proc/meminfo') as f:
        return next(float(x.split()[1])/1024 for x in f if x.startswith('MemAvailable:'))


def checkpoint(report):
    path=report.get('output_path')
    if path:
        with open(path+'.tmp','w') as f:f.write(json.dumps(report))
        os.rename(path+'.tmp',path)


def front_distance(points):
    hits=[]
    for angle,d in points:
        a=angle+math.pi;x=LIDAR_FORWARD_M+d*math.cos(a);y=d*math.sin(a)
        if x>.20 and abs(y)<=.18:
            if x<.32:
                return x
            hits.append((math.atan2(y,x),x,y))
    hits.sort();best=float('inf');cluster=[]
    for p in hits:
        if cluster and (p[0]-cluster[-1][0]>math.radians(1) or
                        math.hypot(p[1]-cluster[-1][1],p[2]-cluster[-1][2])>.04):
            if len(cluster)>=3:best=min(best,min(q[1] for q in cluster))
            cluster=[]
        cluster.append(p)
    if len(cluster)>=3:best=min(best,min(q[1] for q in cluster))
    return best


class Sensors:
    def __init__(self):
        import rospy,tf
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_srvs.srv import Empty
        self.rospy=rospy;self.tf=tf;self.listener=tf.TransformListener()
        self.lock=threading.Lock();self.scan=None;self.odom=None;self.amcl=None
        def scan(m):
            points=[(m.angle_min+i*m.angle_increment,float(v)) for i,v in enumerate(m.ranges)
                    if not math.isnan(v) and not math.isinf(v) and m.range_min<=v<=m.range_max]
            with self.lock:self.scan=(time.time(),m.header.stamp.to_sec(),m.header.frame_id,points)
        def odom(m):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            with self.lock:self.odom=(time.time(),m.header.stamp.to_sec(),m.header.frame_id,m.child_frame_id,
                [p.x,p.y,tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]],m.twist.twist.linear.x,m.twist.twist.angular.z)
        def amcl(m):
            with self.lock:self.amcl=(time.time(),m.header.stamp.to_sec(),m.header.frame_id,list(m.pose.covariance))
        self.subs=[rospy.Subscriber('/scan',LaserScan,scan,queue_size=1,tcp_nodelay=True),
                   rospy.Subscriber('/odom_setmotor',Odometry,odom,queue_size=1,tcp_nodelay=True),
                   rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,amcl,queue_size=1,tcp_nodelay=True)]
        self.nomotion=rospy.ServiceProxy('/request_nomotion_update',Empty)
        def update(_):
            try:self.nomotion()
            except rospy.ServiceException:pass
        self.timer=rospy.Timer(rospy.Duration(.5),update)

    def state(self):
        with self.lock:a,o,p=self.scan,self.odom,self.amcl
        now=time.time()
        if a is None or o is None or p is None:raise Transient('missing ROS data')
        if a[2]!='laser' or o[2:4]!=('odom','base_footprint') or p[2]!='map':raise RuntimeError('wrong frames')
        if now-a[0]>.35 or now-o[0]>.20 or not 0<=now-a[1]<=.4 or not 0<=now-o[1]<=.4:raise Transient('stale scan/odom')
        if now-p[0]>2 or not 0<=now-p[1]<=2:raise Transient('stale AMCL')
        if not a[3]:raise Transient('empty scan')
        cov=p[3]
        if not all(finite(v) for v in (cov[0],cov[7],cov[35])):
            raise RuntimeError('invalid covariance')
        if min(cov[0],cov[7],cov[35])<0:raise RuntimeError('negative covariance')
        if max(cov[0],cov[7])>.15**2 or cov[35]>math.radians(10)**2:raise Transient('AMCL uncertainty')
        try:
            stamp=self.listener.getLatestCommonTime('map','odom').to_sec()
            t,q=self.listener.lookupTransform('map','odom',self.rospy.Time(0))
        except self.tf.Exception as exc:raise Transient('map TF: '+str(exc))
        if not -.6<=now-stamp<=1:raise Transient('stale map TF')
        yaw=self.tf.transformations.euler_from_quaternion(q)[2];c,s=math.cos(yaw),math.sin(yaw)
        pose=[t[0]+c*o[4][0]-s*o[4][1],t[1]+s*o[4][0]+c*o[4][1],wrap(yaw+o[4][2])]
        if not all(finite(v) for v in pose+o[4]+[o[5],o[6]]):raise RuntimeError('nonfinite pose/twist')
        return {'pose':pose,'odom':o[4],'odom_stamp':o[1],'vx':o[5],'wz':o[6],
                'front':front_distance(a[3]),'scan_age':now-a[1],
                'cov':[cov[0],cov[7],cov[35]],'tf_age':now-stamp}


class Runner:
    def __init__(self,sensors,policy,report,seconds):
        self.sensors=sensors;self.policy=policy;self.report=report
        self.end=time.time()+seconds;self.sock=None;self.last_health=0;self.pid=rpc('health')['service_pid']

    def stop(self):
        if self.sock is not None:
            for _ in range(3):
                try:self.sock.sendall(b'{"action":"stop"}\n')
                except socket.error:break

    def rotation_allowed(self,pose):
        return all(self.policy.allowed(pose[0]+.06*math.cos(i*math.pi/8),
                                       pose[1]+.06*math.sin(i*math.pi/8)) for i in range(16))

    def state(self):
        if time.time()>=self.end:raise RuntimeError('patrol duration limit')
        memory=available_mb();self.report['min_available_mb']=min(memory,self.report.get('min_available_mb',memory))
        if memory<768:raise RuntimeError('RAM pressure')
        if time.time()-self.last_health>1:
            h=rpc('health');ch=h['chassis'];self.last_health=time.time()
            if h['service_pid']!=self.pid or ch['arm_pose']!='travel' or ch['arm_busy'] or ch['board_rx_age_s']>.2:
                raise RuntimeError('resident feedback/arm/owner fault')
        s=self.sensors.state()
        if not self.policy.allowed(*s['pose'][:2]):raise RuntimeError('current centre outside navigation mask')
        return s

    def ready(self,seconds=3):
        end=time.time()+seconds
        while True:
            try:return self.state()
            except Transient:
                self.stop()
                if time.time()>=end:raise
                time.sleep(.04)

    def settle(self):
        self.stop();start=time.time();last=None;stable=0
        while time.time()-start<3:
            s=self.ready()
            if s['odom_stamp']!=last:
                last=s['odom_stamp'];stable=stable+1 if abs(s['vx'])<.01 and abs(s['wz'])<.02 else 0
            if stable>=3 and time.time()-start>=.45:return s
            time.sleep(.03)
        raise Transient('stop feedback timeout')

    def pulse(self,vx,wz,duration):
        initial=self.ready();start=time.time();self.report['pulses']+=1
        try:
            if vx and duration>getattr(self,'max_forward_duration',.6):raise RuntimeError('forward pulse duration limit')
            while time.time()-start<duration:
                s=self.state();pose=s['pose']
                if math.hypot(s['odom'][0]-initial['odom'][0],s['odom'][1]-initial['odom'][1])>.35*duration+.12:
                    raise RuntimeError('pulse translation limit')
                if vx:
                    if s['front']<getattr(self,'front_brake_m',.5):raise Transient('front brake')
                    if not self.policy.stopping_path_allowed(pose,.35*max(0,duration-(time.time()-start))+stop_reserve(duration)):raise Transient('predicted stop crosses mask')
                else:
                    if not self.rotation_allowed(pose):
                        raise RuntimeError('turn drift reserve outside mask')
                    if abs(wrap(s['odom'][2]-initial['odom'][2]))>math.radians(20):raise RuntimeError('pulse angular limit')
                self.sock.sendall((json.dumps({'action':'velocity','vx':vx,'wz':wz})+'\n').encode('ascii'))
                time.sleep(.04)
        finally:self.stop()
        final=self.settle();self.report['trace'].append(final);checkpoint(self.report)

    def navigate(self,goal,label):
        self.report['active_goal']={'label':label,'point':goal};checkpoint(self.report)
        current=self.ready()['pose'][:2]
        origin=self.report.get('last_planned_goal',current)
        if math.hypot(origin[0]-current[0],origin[1]-current[1])>.13:
            origin=current
        begun=time.time();last_distance=None;stalled=0
        for _ in range(160):
            s=self.ready();p=s['pose'];distance=math.hypot(goal[0]-p[0],goal[1]-p[1])
            if distance<=.045:
                s=self.settle();error=math.hypot(goal[0]-s['pose'][0],goal[1]-s['pose'][1])
                if error<=.065:
                    self.report['last_planned_goal']=goal
                    self.report['arrivals'].append({'label':label,'goal':goal,'pose':s['pose'],'error_m':error,'cov':s['cov']});checkpoint(self.report);return
            if time.time()-begun>150:raise RuntimeError('waypoint timeout')
            if not self.policy.segment_allowed(p[:2],goal):raise RuntimeError('waypoint segment crosses mask')
            err=wrap(guided_heading(p,origin,goal)-p[2])
            try:
                if abs(err)>math.radians(3):
                    self.pulse(0,1 if err>0 else -1,min(.12,max(.04,(abs(err)-.02)/1.5)))
                else:
                    choices=[d for d in (.6,.35,.15,.08,.04) if d<=getattr(self,'max_forward_duration',.6) and .35*d+.025<distance and self.policy.stopping_path_allowed(p,.35*d+stop_reserve(d))]
                    if not choices:
                        if distance<.10:choices=[.04] if self.policy.stopping_path_allowed(p,.074) else []
                        if not choices:raise RuntimeError('cannot approach goal within stop reserve')
                    self.pulse(.15,0,choices[0])
                stalled=stalled+1 if last_distance is not None and abs(distance-last_distance)<.002 and abs(err)<math.radians(3) else 0
                if stalled>=5:raise RuntimeError('no translational progress')
                last_distance=distance
            except Transient as exc:
                self.stop();self.report['recoveries'].append(str(exc))
                if len(self.report['recoveries'])>12:raise RuntimeError('recovery budget exhausted')
                self.settle();self.ready()
        raise RuntimeError('waypoint iteration limit')

    def goto(self,goal,label):
        current=self.ready()['pose'][:2];path=self.policy.plan(current,goal)
        self.report.setdefault('planned_paths',[]).append({'label':label,'points':path})
        for i,point in enumerate(path[1:]):self.navigate(point,label if i==len(path)-2 else label+'_via%d'%i)

    def face_interior(self):
        centre=[sum(p[i] for p in self.policy.layout['room_polygon'])/4 for i in (0,1)]
        for _ in range(50):
            s=self.ready();pose=s['pose'];error=wrap(math.atan2(centre[1]-pose[1],centre[0]-pose[0])-pose[2])
            if abs(error)<=math.radians(6):
                self.report['park_facing']='room_centre';self.report['park_yaw_error_deg']=math.degrees(error);return
            try:self.pulse(0,1 if error>0 else -1,min(.15,max(.05,(abs(error)-.03)/1.5)))
            except Transient as exc:
                self.report['recoveries'].append(str(exc));self.settle()
                if len(self.report['recoveries'])>12:raise RuntimeError('recovery budget exhausted')
        raise RuntimeError('final orientation limit')


def resolve_goals(requested,policy,points):
    if not isinstance(requested,list) or not 1<=len(requested)<=12:
        raise ValueError('goals must be a list of 1..12 entries')
    result=[]
    for i,item in enumerate(requested):
        if item in ('home','start'):goal=points[0];name='home'
        elif item=='dropoff':
            raise ValueError('dropoff approach/exit not yet validated')
        elif isinstance(item,dict):goal=item['point'];name=str(item.get('name','goal%d'%i))
        else:raise ValueError('unknown goal')
        if not isinstance(goal,(list,tuple)) or len(goal)!=2:raise ValueError('illegal goal')
        goal=[float(v) for v in goal]
        if not all(finite(v) for v in goal) or not policy.allowed(*goal):raise ValueError('illegal goal')
        result.append((name,goal))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',required=True);p.add_argument('--output',required=True)
    p.add_argument('--loops',type=int,default=1);p.add_argument('--seconds',type=int,default=600);p.add_argument('--real',action='store_true');p.add_argument('--goals',help='JSON list of home, dropoff or {name,point:[x,y]}')
    p.add_argument('--inset',type=float,default=.60,help='Patrol centre distance inside fitted walls, 0.60..0.90 m')
    args=p.parse_args()
    if not 1<=args.loops<=2 or not 30<=args.seconds<=600:p.error('bounded loops 1..2, seconds 30..600')
    if not finite(args.inset) or not .60<=args.inset<=.90:p.error('patrol inset must be 0.60..0.90 m')
    policy=Policy(args.scene);points=policy.patrol_points(args.inset);report={'ok':False,'real':args.real,'output_path':args.output,'waypoints':points,'patrol_inset_m':args.inset,'primary_patrol_point':'opposite paper-box corner','arrivals':[],'recoveries':[],'trace':[],'pulses':0}
    runner=None
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    try:
        if args.real:policy.require_motion_ready(LIDAR_FORWARD_M)
        goals=None
        if args.goals:
            with open(args.goals) as f:goals=resolve_goals(json.load(f),policy,points)
        if available_mb()<1024:raise RuntimeError('RAM below 1GB at start')
        h=rpc('health');ch=h['chassis'];owner=str(h['service_pid'])
        held=subprocess.check_output(['fuser','/dev/myserial']).decode().split()
        if held!=[owner] or ch['motors']!=[0,0,0,0] or ch['moving'] or ch['client'] is not None:raise RuntimeError('sole idle owner preflight failed')
        import rospy
        rospy.init_node('g2_map_patrol',disable_signals=True);sensors=Sensors();runner=Runner(sensors,policy,report,args.seconds)
        report['initial']=runner.ready(8)
        if args.real:
            result=rpc('maintenance-unlock','/tmp/grasp_service.sock')
            if not result.get('ok'):raise RuntimeError('unlock refused')
            runner.sock=socket.create_connection(('127.0.0.1',7000),timeout=1);runner.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            # Enter the interior before rotating when parked near the centre-mask
            # edge. No reverse/blind turn; each forward pulse retains all gates.
            for _ in range(3):
                current=runner.ready()['pose']
                if runner.rotation_allowed(current):break
                toward=(points[0][0]-current[0])*math.cos(current[2])+(points[0][1]-current[1])*math.sin(current[2])
                if toward<=0 or not policy.stopping_path_allowed(current,.2725):raise RuntimeError('cannot enter turn reserve')
                runner.pulse(.15,0,.35)
            if not runner.rotation_allowed(runner.ready()['pose']):raise RuntimeError('entry turn reserve not reached')
            if goals:
                for name,goal in goals:runner.goto(goal,name)
            else:
                runner.goto(points[0],'diagonal_start')
                for loop in range(args.loops):
                    for i in (1,2,3,0):runner.goto(points[i],'loop%d_point%d'%(loop+1,i))
            runner.face_interior()
        report['ok']=True
    except (Exception,KeyboardInterrupt) as exc:report['error']=str(exc) or 'interrupted'
    finally:
        if runner is not None:
            runner.stop()
            if runner.sock is not None:runner.sock.close()
        if args.real:
            try:
                for _ in range(30):
                    h=rpc('health')
                    if h['chassis']['motors']==[0,0,0,0] and not h['chassis']['moving']:
                        result=rpc('maintenance-lock','/tmp/grasp_service.sock')
                        if result.get('ok'):break
                    time.sleep(.1)
                else:raise RuntimeError('maintenance lock timeout')
                report['final_health']=rpc('health')
            except Exception as exc:report['ok']=False;report['cleanup_error']=str(exc)
        with open(args.output,'w') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:report.get(k) for k in ('ok','error','pulses','recoveries','arrivals')}));return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
