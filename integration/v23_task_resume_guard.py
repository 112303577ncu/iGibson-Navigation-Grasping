#!/usr/bin/env python3
"""Read-only task wait/resume checks; --real may submit a validated AMCL seed.

Never opens Serial, sends chassis commands, or repeats arm operations. Heavy
vision and ArUco metric estimation belong to the calibrated offboard producer.
"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import time
import uuid

PHASES=('idle','grasp','release','visual-approach')


def read_json(path):
    if os.path.getsize(path)>65536:raise ValueError('oversize state file')
    with open(path) as f:value=json.load(f)
    if not isinstance(value,dict):raise ValueError('state must be an object')
    return value


def atomic(path,value):
    path=Path(path);temp=path.with_name(path.name+'.tmp')
    with temp.open('w') as f:json.dump(value,f,allow_nan=False,indent=2)
    os.replace(str(temp),str(path))


def resident_health(path='/tmp/grasp_health.sock'):
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(1.5);s.connect(path);s.sendall(b'health\n')
        with s.makefile('rb') as stream:line=stream.readline(65537)
    if len(line)>65536:raise ValueError('oversize resident health')
    result=json.loads(line)
    if not isinstance(result,dict):raise ValueError('invalid resident health')
    return result


def available_mb():
    with open('/proc/meminfo') as f:
        return next(float(line.split()[1])/1024 for line in f if line.startswith('MemAvailable:'))


def vector(value,n=3):
    if (not isinstance(value,(tuple,list)) or len(value)!=n
            or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value)):
        raise ValueError('invalid coordinate vector')
    return list(value)


def wrap(angle):return (angle+math.pi)%(2*math.pi)-math.pi


def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])


def stopped(health):
    ch=health['chassis'];duration=ch.get('stopped_for_s')
    return (ch.get('motors')==[0,0,0,0] and ch.get('moving') is False
            and isinstance(duration,(int,float)) and duration>=.5)


def task_ready(health):
    ch=health['chassis']
    return stopped(health) and ch.get('arm_busy') is False and ch.get('arm_pose')=='travel' and 'client' in ch and ch['client'] is None


def localization_problem(health,loc,boot,now):
    if (not isinstance(loc.get('stamp'),(int,float)) or not 0<=now-loc['stamp']<=.4):return 'STALE_LOCALIZATION_HEALTH'
    if loc.get('boot_id')!=boot or loc.get('owner_pid')!=health['service_pid']:return 'LOCALIZATION_BINDING_CHANGED'
    if not loc.get('map_hash'):return 'LOCALIZATION_MAP_UNKNOWN'
    if loc.get('ready') is not True:return loc.get('reason_code','LOCALIZATION_NOT_READY')
    state=loc['state'];vector(state['pose']);vector(state['odom']);cov=vector(state['cov'])
    if not 0<=now-state['odom_stamp']<=.4:return 'STALE_ODOM'
    if not 0<=state['scan_age']<=.4 or not -.6<=state['tf_age']<=1:return 'STALE_SCAN_OR_TF'
    if min(cov)<0 or max(cov[:2])>.15**2 or cov[2]>math.radians(10)**2:return 'AMCL_UNCERTAIN'
    if (health.get('ok') is not True or health['odom'].get('valid') is not True
            or not 0<=health['chassis']['board_rx_age_s']<=.2):return 'RESIDENT_FEEDBACK_UNAVAILABLE'
    measured=vector(health['odom']['pose'])
    if distance(measured,state['odom'])>.02 or abs(wrap(measured[2]-state['odom'][2]))>math.radians(2):return 'ODOM_SNAPSHOT_MISMATCH'
    return None


def begin_checkpoint(phase,health,loc,boot,now,operation_id=None,next_task='return'):
    if phase not in PHASES:raise ValueError('invalid task phase')
    if not task_ready(health):raise ValueError('begin before task: resident must be stopped, travel, and unclaimed')
    if phase in ('grasp','release') and not operation_id:raise ValueError('arm task requires its unique operation ID')
    problem=localization_problem(health,loc,boot,now)
    if problem:raise ValueError(problem)
    if not loc.get('map_hash'):raise ValueError('raw map hash required')
    return dict(schema=1,task_id=uuid.uuid4().hex,phase=phase,operation_id=operation_id,
                next_task=next_task,stamp=now,boot_id=boot,owner_pid=health['service_pid'],
                map_hash=loc['map_hash'],start_pose=vector(loc['state']['pose']),
                start_odom=vector(loc['state']['odom']))


class ResumeGate:
    def __init__(self,checkpoint):
        if checkpoint.get('schema')!=1 or checkpoint.get('phase') not in PHASES:raise ValueError('invalid checkpoint')
        for key in ('task_id','boot_id','owner_pid','map_hash','stamp'):checkpoint[key]
        vector(checkpoint['start_pose']);vector(checkpoint['start_odom'])
        if checkpoint['phase'] in ('grasp','release') and not isinstance(checkpoint.get('operation_id'),str):
            raise ValueError('arm checkpoint needs operation ID')
        if checkpoint['phase'] in ('grasp','release') and not checkpoint['operation_id']:raise ValueError('empty operation ID')
        if checkpoint.get('next_task') not in ('return','patrol','dropoff'):raise ValueError('invalid next task')
        self.checkpoint=checkpoint;self.last_stamp=None;self.last_pose=None;self.count=0;self.reanchored=False

    def reset(self):self.count=0;self.last_stamp=None;self.last_pose=None

    def poll(self,health,loc,boot,now,action_result='unknown',operation_id=None,relocated=False):
        cp=self.checkpoint
        result=dict(ok=False,status='WAIT_LOCALIZATION',task_id=cp['task_id'],next_task=cp['next_task'],
                    stable_frames=self.count,arm_or_motion_executed=False)
        def fail(status,reason):
            self.reset();result.update(status=status,reason=reason,stable_frames=0);return result
        if (boot!=cp['boot_id'] or health['service_pid']!=cp['owner_pid']
                or (loc.get('map_hash') and loc['map_hash']!=cp['map_hash'])):
            return fail('NEW_TASK_CONTEXT_REQUIRED','boot/owner/map changed; create a new checkpoint after localization')
        if cp['phase'] in ('grasp','release'):
            if action_result!='success' or operation_id!=cp['operation_id']:
                return fail('WAIT_ACTION_RESULT','confirm this operation ID/result; never repeat an unknown arm action')
            expected=cp['phase']=='grasp'
            if health.get('holding') is not expected:return fail('WAIT_ACTION_RESULT','resident holding state disagrees with confirmed arm result')
        if not task_ready(health):return fail('WAIT_TASK_READY','wait for stopped base, stowed arm and released controller; task time is not localization timeout')
        if relocated and not self.reanchored:return fail('NEED_REFERENCE','relocation reported; require a new registered absolute reference')
        problem=localization_problem(health,loc,boot,now)
        if problem:
            return fail('NEED_REFERENCE' if problem in ('REFERENCE_REQUIRED','LOCALIZATION_BINDING_CHANGED') else 'WAIT_LOCALIZATION',problem)
        state=loc['state'];odom=state['odom'];pose=state['pose'];old=cp['start_odom'];initial=cp['start_pose']
        angle=initial[2]-old[2];dx,dy=odom[0]-old[0],odom[1]-old[1]
        predicted=[initial[0]+math.cos(angle)*dx-math.sin(angle)*dy,
                   initial[1]+math.sin(angle)*dx+math.cos(angle)*dy,wrap(initial[2]+odom[2]-old[2])]
        if (cp['phase']!='visual-approach' and (math.hypot(dx,dy)>.03 or abs(wrap(odom[2]-old[2]))>math.radians(3))):
            return fail('NEED_REFERENCE','base moved during a stationary task; re-anchor before planning')
        if distance(predicted,pose)>.15 or abs(wrap(predicted[2]-pose[2]))>math.radians(10):
            return fail('NEED_REFERENCE','AMCL/odom continuity mismatch')
        result.update(status='WAIT_STABLE_POSE',reason='collect five fresh stable odom/pose samples')
        if (self.last_pose is not None and (distance(self.last_pose,pose)>.03
                or abs(wrap(self.last_pose[2]-pose[2]))>math.radians(3))):self.reset()
        if state['odom_stamp']!=self.last_stamp:
            if self.last_stamp is not None and state['odom_stamp']<self.last_stamp:return fail('WAIT_LOCALIZATION','odom timestamp moved backwards')
            self.count+=1;self.last_stamp=state['odom_stamp']
            if self.last_pose is None:self.last_pose=list(pose)
        result['stable_frames']=self.count
        if self.count>=5:
            result.update(ok=True,status='READY_TO_REPLAN',pose=list(pose),stamp=now,valid_until=now+.4,
                          boot_id=boot,owner_pid=cp['owner_pid'],map_hash=cp['map_hash'],
                          reason='plan from current position; do not replay old relative motion or arm actions')
        return result

    def reanchor(self,pose,odom):
        self.checkpoint=dict(self.checkpoint,start_pose=vector(pose),start_odom=vector(odom))
        self.reanchored=True;self.reset()


class ReferenceCollector:
    """Accept three new stable metric observations from a registered producer."""
    def __init__(self,config,checkpoint):
        if (config.get('calibration_verified') is not True or config.get('source')!='aruco'
                or config.get('map_hash')!=checkpoint['map_hash']
                or not config.get('calibration_id') or not config.get('registration_id')
                or not config.get('marker_ids')):raise ValueError('verified ArUco calibration and raw-map registration required')
        if (not isinstance(config['marker_ids'],list) or any(type(i) is not int or not 0<=i<50 for i in config['marker_ids'])):
            raise ValueError('registered marker IDs must belong to DICT_4X4_50')
        self.config=config;self.cp=checkpoint;self.last=None;self.pose=None;self.count=0

    def feed(self,packet,health,boot,now):
        try:
            cfg=self.config;pose=vector(packet['pose_map']);cov=vector(packet['covariance_diag'])
            seq=packet['seq'];stamp=packet['stamp'];ids=packet['marker_ids']
            if (packet.get('valid') is not True or packet.get('source')!='aruco' or packet.get('frame')!='map'
                    or packet.get('boot_id')!=boot or packet.get('owner_pid')!=health['service_pid']
                    or boot!=self.cp['boot_id'] or health['service_pid']!=self.cp['owner_pid']
                    or packet.get('map_hash')!=self.cp['map_hash']
                    or any(packet.get(k)!=cfg[k] for k in ('calibration_id','registration_id'))
                    or not isinstance(seq,int) or isinstance(seq,bool) or seq<0
                    or not isinstance(stamp,(int,float)) or not 0<=now-stamp<=1
                    or not isinstance(ids,list) or not ids or any(type(i) is not int or i not in cfg['marker_ids'] for i in ids)
                    or min(cov)<0 or max(cov[:2])>.05**2 or cov[2]>math.radians(5)**2
                    or not task_ready(health) or health['odom'].get('valid') is not True
                    or not 0<=health['chassis']['board_rx_age_s']<=.2):raise ValueError('invalid absolute reference')
            if self.last is not None and (seq<=self.last[0] or stamp<=self.last[1]):return None
            if self.last is not None and stamp-self.last[1]>1:self.count=0;self.pose=None
            self.last=(seq,stamp)
            if self.pose is not None and (distance(pose,self.pose)>.03 or abs(wrap(pose[2]-self.pose[2]))>math.radians(3)):self.count=0
            self.pose=pose;self.count+=1
            if self.count<3:return None
            return dict(stamp=stamp,boot_id=boot,owner_pid=health['service_pid'],map_hash=self.cp['map_hash'],
                        pose=pose,odom=vector(health['odom']['pose']),source='registered_aruco_task_resume',
                        task_id=self.cp['task_id'],calibration_id=cfg['calibration_id'],registration_id=cfg['registration_id'])
        except (KeyError,TypeError,ValueError):self.count=0;self.pose=None;return None


def run_resume(checkpoint,snapshot,wait_seconds=20,action_result='unknown',operation_id=None,
               relocated=False,collector=None,reference_read=None,seed_write=None,real=False,
               wall_clock=time.time,clock=time.monotonic,sleep=time.sleep):
    if not 0<wait_seconds<=60:raise ValueError('resume wait must be within 60 seconds')
    gate=ResumeGate(checkpoint);end=clock()+wait_seconds;pending=None;result={};submitted=False
    while clock()<end:
        try:
            health,loc,boot,ram=snapshot();now=wall_clock()
            result=gate.poll(health,loc,boot,now,action_result,operation_id,relocated)
            if result['status'] in ('WAIT_ACTION_RESULT','WAIT_TASK_READY','NEW_TASK_CONTEXT_REQUIRED'):return result
            if ram<1024:
                gate.reset();result=dict(ok=False,status='WAIT_MEMORY',reason='need 1024 MB before next phase')
            else:
                if pending is not None:
                    if (loc.get('ready') is True and loc.get('accepted_reference_stamp')==pending['stamp']
                            and not localization_problem(health,loc,boot,now)):
                        expected=pending['pose'];actual=loc['state']['pose']
                        if distance(expected,actual)<=.08 and abs(wrap(expected[2]-actual[2]))<=math.radians(8):
                            gate.reanchor(actual,loc['state']['odom']);pending=None
                            result=gate.poll(health,loc,boot,now,action_result,operation_id,relocated)
                    if pending is not None:
                        result=dict(ok=False,status='WAIT_REFERENCE_APPLIED',reason='wait for supervisor acknowledgement and fresh consistent AMCL')
                        sleep(.05);continue
                if result['ok']:return result
                if result['status']=='NEED_REFERENCE' and submitted:return result
                if result['status'] in ('NEED_REFERENCE','WAIT_LOCALIZATION') and collector is not None and not submitted:
                    seed=collector.feed(reference_read(),health,boot,now)
                    if seed is not None:
                        if not real:return dict(ok=False,status='REFERENCE_READY_DRY_RUN',proposed_seed=seed,arm_or_motion_executed=False)
                        seed_write(seed);pending=seed;submitted=True;gate.reset()
                elif result['status']=='NEED_REFERENCE':return result
        except (OSError,ValueError,KeyError,TypeError,StopIteration) as exc:
            gate.reset();result=dict(ok=False,status='WAIT_INPUT',reason=str(exc))
        sleep(.05)
    return dict(result,ok=False,status='RESUME_TIMEOUT',last_status=result.get('status'),
                next_action='keep base stopped; obtain fresh input/reference; do not replay task',arm_or_motion_executed=False)


def observe_wait(checkpoint,health,loc,boot,now,ram):
    problem=localization_problem(health,loc,boot,now)
    if ram<768:problem='RAM_PRESSURE'
    context=(boot==checkpoint['boot_id'] and health['service_pid']==checkpoint['owner_pid']
             and loc.get('map_hash')==checkpoint['map_hash'])
    return dict(ok=True,status='TASK_WAIT_OBSERVATION',task_id=checkpoint['task_id'],
                task_wait_elapsed_s=max(0,now-checkpoint['stamp']),localization_ready=problem is None and context,
                localization_reason=problem if context else 'TASK_CONTEXT_CHANGED',task_ready=task_ready(health),
                available_mb=ram,task_completed=False,resume_check_required=True,arm_or_motion_executed=False)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=('begin','wait','resume'))
    p.add_argument('--checkpoint',required=True)
    p.add_argument('--phase',choices=PHASES,default='idle')
    p.add_argument('--next-task',choices=('return','patrol','dropoff'),default='return')
    p.add_argument('--operation-id');p.add_argument('--completed-operation-id')
    p.add_argument('--action-result',choices=('success','failure','unknown'),default='unknown')
    p.add_argument('--localization',default='/dev/shm/g2_localization_health.json')
    p.add_argument('--resident-socket',default='/tmp/grasp_health.sock')
    p.add_argument('--reference-config');p.add_argument('--reference',default='/dev/shm/g2_aruco_absolute_reference.json')
    p.add_argument('--seed',default='/dev/shm/g2_localization_seed.json')
    p.add_argument('--relocated',action='store_true')
    p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--real',action='store_true',help='allow only validated AMCL seed submission; never drive or actuate arm')
    p.add_argument('--output',required=True)
    args=p.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0<args.seconds<=(3600 if args.mode=='wait' else 60):p.error('seconds out of range')
    if Path(args.output).resolve() in {Path(v).resolve() for v in (args.checkpoint,args.localization,args.seed,args.reference)}:
        p.error('output must not overwrite checkpoint or input/seed files')
    def snapshot():
        with open('/proc/sys/kernel/random/boot_id') as f:boot=f.read().strip()
        return resident_health(args.resident_socket),read_json(args.localization),boot,available_mb()
    result=dict(ok=False,arm_or_motion_executed=False)
    try:
        if args.mode=='begin':
            if Path(args.checkpoint).exists():raise ValueError('checkpoint exists; use a new task path')
            health,loc,boot,ram=snapshot()
            if ram<1024:raise ValueError('RAM below new-phase reserve')
            cp=begin_checkpoint(args.phase,health,loc,boot,time.time(),args.operation_id,args.next_task)
            atomic(args.checkpoint,cp);result.update(ok=True,status='WAIT_CHECKPOINT_RECORDED',task_id=cp['task_id'])
        elif args.mode=='wait':
            cp=read_json(args.checkpoint);ResumeGate(cp);end=time.monotonic()+args.seconds
            while time.monotonic()<end:
                try:
                    health,loc,boot,ram=snapshot();result=observe_wait(cp,health,loc,boot,time.time(),ram)
                except (OSError,ValueError,KeyError,TypeError,StopIteration) as exc:
                    result=dict(ok=True,status='TASK_WAIT_OBSERVATION',localization_ready=False,
                                localization_reason=str(exc),task_completed=False,resume_check_required=True,arm_or_motion_executed=False)
                atomic(args.output,result);time.sleep(min(.5,max(0,end-time.monotonic())))
            result['status']='WAIT_OBSERVATION_FINISHED'
        else:
            cp=read_json(args.checkpoint)
            collector=ReferenceCollector(read_json(args.reference_config),cp) if args.reference_config else None
            result=run_resume(cp,snapshot,args.seconds,args.action_result,args.completed_operation_id,args.relocated,
                              collector,lambda:read_json(args.reference),lambda seed:atomic(args.seed,seed),args.real)
    except (OSError,ValueError,KeyError,TypeError,StopIteration,KeyboardInterrupt) as exc:result.update(ok=False,status='CHECK_FAILED',reason=str(exc))
    result['arm_or_motion_executed']=False;atomic(args.output,result);print(json.dumps(result))
    return 0 if result['ok'] else (2 if result.get('status')!='CHECK_FAILED' else 1)


if __name__=='__main__':raise SystemExit(main())
