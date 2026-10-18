#!/usr/bin/env python
"""Supervised measured-gap approach, not autonomous map/vision docking.

Only an unchanged current-boot operator measurement authorizes this straight
corridor. No turning, arm commands, persistent mask changes or Serial access.
"""
from __future__ import division
import argparse
import json
import math
import os
import signal
import socket
import subprocess
import time

from g2_map_patrol import Sensors,Policy,Transient,rpc,available_mb,wrap,checkpoint
from g2_patrol_policy import read_pgm


def components(origin,current):
    dx,dy=current[0]-origin[0],current[1]-origin[1]
    c,s=math.cos(origin[2]),math.sin(origin[2])
    return c*dx+s*dy,-s*dx+c*dy,wrap(current[2]-origin[2])


def approach_target(reference,current,gap_min,requested):
    if not all(not math.isnan(v) and not math.isinf(v) for v in reference+current+[gap_min,requested]):
        raise ValueError('nonfinite measured reference')
    if not .05<=requested<=.30 or gap_min<requested+.04:
        raise ValueError('requested approach exceeds measured-gap allowance')
    along,side,yaw=components(reference,current)
    if math.hypot(along,side)>.08 or abs(side)>.06 or abs(yaw)>math.radians(6):
        raise ValueError('not aligned with the current measured reference')
    target=min(.30,requested-along,gap_min-along-.04)
    if target<.02:raise ValueError('no remaining measured approach')
    return target,along


def wall_clearance(polygon,point):
    return min(((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]))/
               math.hypot(b[0]-a[0],b[1]-a[1])
               for a,b in zip(polygon,polygon[1:]+polygon[:1]))


def resume_parameters(previous,reference,current_odom):
    if previous.get('ok') or not previous.get('real'):raise ValueError('resume needs an interrupted real approach')
    old=previous['final_health']
    if old['service_pid']!=reference['service_pid']:raise ValueError('resume owner changed')
    expected=old['odom']['pose']
    if math.hypot(current_odom[0]-expected[0],current_odom[1]-expected[1])>.02 or abs(wrap(current_odom[2]-expected[2]))>math.radians(2):
        raise ValueError('robot moved since interrupted approach')
    target,offset=approach_target(reference['pose_map'],previous['initial']['pose'],
        reference['operator_front_gap_range_m'][0],reference['requested_advance_m'])
    if abs(target-previous['target_from_start_m'])>1e-6:raise ValueError('resume measurement changed')
    return list(previous['initial']['odom']),target,offset


class FreeDisk:
    def __init__(self,folder,policy):
        import yaml
        with open(os.path.join(folder,policy.layout['map'])) as f:meta=yaml.safe_load(f)
        if tuple(meta['origin'])!=policy.origin or meta['resolution']!=policy.resolution:
            raise ValueError('raw map geometry mismatch')
        self.w,self.h,self.cells=read_pgm(os.path.join(folder,meta['image']))
        self.origin=policy.origin;self.resolution=policy.resolution;self.cache={}

    def allowed(self,x,y):
        r=self.resolution;col=int(math.floor((x-self.origin[0])/r));row=self.h-1-int(math.floor((y-self.origin[1])/r))
        key=(col,row)
        if key not in self.cache:
            # Cover 24cm body radius plus grid-centre quantization error.
            radius=.24+r/math.sqrt(2);n=int(math.ceil(radius/r));valid=True
            for dy in range(-n,n+1):
                for dx in range(-n,n+1):
                    if math.hypot(dx*r,dy*r)>radius:continue
                    c,v=col+dx,row+dy
                    if not(0<=c<self.w and 0<=v<self.h) or self.cells[v*self.w+c]<=250:valid=False
            self.cache[key]=valid
        return self.cache[key]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',required=True);p.add_argument('--reference',required=True)
    p.add_argument('--output',required=True);p.add_argument('--real',action='store_true')
    p.add_argument('--resume-report',help='Continue the original target after a verified stopped interruption')
    args=p.parse_args()
    report={'ok':False,'real':args.real,'output_path':args.output,'trace':[],'pulses':0,
            'scope':'operator-measured straight approach only; no permanent box registration',
            'front_brake_m':.35,'max_pulse_s':.08,'arm_or_release_executed':False,'recoveries':[]}
    sock=None;origin=None;target=None;sensors=None;begun=time.time()
    def stop():
        if sock is not None:
            for _ in range(3):
                try:sock.sendall(b'{"action":"stop"}\n')
                except socket.error:break
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:
        with open(args.reference) as f:ref=json.load(f)
        with open('/proc/sys/kernel/random/boot_id') as f:boot=f.read().strip()
        health=rpc('health');owner=health['service_pid'];ch=health['chassis']
        if ref['boot_id']!=boot or ref['service_pid']!=owner:raise ValueError('measurement owner/boot expired')
        if subprocess.check_output(['fuser','/dev/myserial']).decode().split()!=[str(owner)]:raise ValueError('sole owner check failed')
        if ch['motors']!=[0,0,0,0] or ch['moving'] or ch['client'] is not None:raise ValueError('owner not idle')
        if available_mb()<1024:raise ValueError('RAM below 1GB before approach')
        policy=Policy(args.scene);policy.require_motion_ready(.10);free=FreeDisk(args.scene,policy)
        import rospy
        rospy.init_node('g2_relative_dock_probe',disable_signals=True);sensors=Sensors()
        previous=None
        if args.resume_report:
            with open(args.resume_report) as f:previous=json.load(f)
            origin,target,offset=resume_parameters(previous,ref,health['odom']['pose'])
            report['resume_source']=args.resume_report
        def checked():
            if time.time()-begun>90:raise RuntimeError('approach time limit')
            ram=available_mb();report['min_available_mb']=min(ram,report.get('min_available_mb',ram))
            if ram<768:raise RuntimeError('RAM pressure')
            h=rpc('health');c=h['chassis']
            if h['service_pid']!=owner or c['arm_pose']!='travel' or c['arm_busy'] or c['board_rx_age_s']>.2:
                raise RuntimeError('resident owner/arm/feedback changed')
            state=sensors.state()
            if wall_clearance(policy.layout['room_polygon'],state['pose'])<.40 or not free.allowed(*state['pose'][:2]):
                raise RuntimeError('wall or raw-map body clearance')
            along,side,yaw=components(ref['pose_map'],state['pose'])
            if not -.10<=along<=ref['requested_advance_m']+.02 or abs(side)>.06 or abs(yaw)>math.radians(6):
                raise RuntimeError('outside measured straight corridor')
            if origin is not None:
                distance,lateral,angle=components(origin,state['odom'])
                if distance<-.01 or distance>target+.01 or abs(lateral)>.025 or abs(angle)>math.radians(3):
                    raise RuntimeError('odom approach travel/heading bound')
                state['progress_m']=distance
            return state
        def ready():
            end=time.time()+3
            while True:
                try:return checked()
                except Transient:
                    stop()
                    if time.time()>end:raise
                    time.sleep(.04)
        def settle():
            stop();start=time.time();last=None;stable=0
            while time.time()-start<3:
                state=ready()
                if state['odom_stamp']!=last:
                    last=state['odom_stamp'];stable=stable+1 if abs(state['vx'])<.01 and abs(state['wz'])<.02 else 0
                if stable>=3 and time.time()-start>=.45:return state
                time.sleep(.03)
            raise RuntimeError('stop feedback timeout')
        initial=ready()
        if previous is None:
            report['initial']=initial
            target,offset=approach_target(ref['pose_map'],initial['pose'],ref['operator_front_gap_range_m'][0],ref['requested_advance_m'])
            origin=list(initial['odom'])
        else:
            report['initial']=previous['initial'];report['resume_initial']=initial
        report.update(target_from_start_m=target,reference_along_offset_m=offset)
        if args.real:
            if not rpc('maintenance-unlock','/tmp/grasp_service.sock').get('ok'):raise RuntimeError('unlock refused')
            sock=socket.create_connection(('127.0.0.1',7000),timeout=1)
            sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            for _ in range(60):
                state=settle();report['trace'].append(state);checkpoint(report)
                remaining=target-state['progress_m']
                if remaining<=.008:break
                duration=.08 if remaining>.02 else .04
                start=time.time();report['pulses']+=1
                try:
                    while time.time()-start<duration:
                        state=checked()
                        if state['front']<.35:raise RuntimeError('front brake')
                        sock.sendall(b'{"action":"velocity","vx":0.15,"wz":0}\n');time.sleep(.02)
                except Transient as exc:
                    report['recoveries'].append(str(exc))
                    if len(report['recoveries'])>6:raise RuntimeError('transient recovery budget exhausted')
                finally:stop()
            else:raise RuntimeError('approach pulse budget exhausted')
            final=settle();report['final']=final;report['travel_m']=final['progress_m']
            if not target-.008<=report['travel_m']<=target+.01:raise RuntimeError('final approach distance outside bound')
        report['ok']=True
    except (Exception,KeyboardInterrupt) as exc:report['error']=str(exc) or 'interrupted'
    finally:
        stop()
        if sock is not None:sock.close()
        if args.real:
            try:
                for _ in range(40):
                    h=rpc('health');c=h['chassis']
                    if c['motors']==[0,0,0,0] and not c['moving']:
                        locked=rpc('maintenance-lock','/tmp/grasp_service.sock')
                        if locked.get('ok'):break
                    time.sleep(.1)
                else:raise RuntimeError('maintenance lock timeout')
                report['final_health']=rpc('health')
            except Exception as exc:report['ok']=False;report['cleanup_error']=str(exc)
        checkpoint(report)
    print(json.dumps({k:report.get(k) for k in ('ok','error','travel_m','target_from_start_m','pulses','min_available_mb')}))
    return 0 if report['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
