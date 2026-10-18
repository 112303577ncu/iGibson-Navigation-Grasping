#!/usr/bin/env python3
"""Resident-only dropoff approach using koala's bounded alignment controller.

Vision runs offboard. This process consumes calibrated, target-specific metric
observations; it never loads YOLO/SAM or opens a serial port, and never releases.
Default: read-only preflight. No blind fixed-distance finishing move.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
from target_approach import ApproachConfig,TargetApproachController
from resident_motion import ResidentMotion
from g2_patrol_policy import Policy


class DockController:
    def __init__(self, config):
        if (config.get('role')!='dropoff' or config.get('calibration_verified') is not True
                or not config.get('target_id') or not config.get('calibration_id')):
            raise ValueError('verified dropoff target/calibration required; sugarbox is not the dropoff box')
        self.cfg=config
        self.standoff=float(config['standoff_m'])
        if not .20<=self.standoff<=.60:raise ValueError('standoff must be measured, 0.20..0.60 m from base centre')
        self.aligner=TargetApproachController(ApproachConfig(
            enabled=True,center_min_wz=1.,center_max_wz=1.,
            center_tolerance_px=math.radians(3)/.0045))
        self.seq=-1;self.stamp=0.;self.confirmed=0

    def step(self, observation, now):
        stop={'vx':0.,'wz':0.,'done':False,'reason':'WAIT_TARGET'}
        o=observation or {}
        try:
            stamp=float(o['stamp']);seq=o['seq'];x=float(o['x_m']);y=float(o['y_m'])
            if (o.get('valid') is not True or o.get('role')!='dropoff'
                    or o.get('target_id')!=self.cfg['target_id']
                    or o.get('calibration_id')!=self.cfg['calibration_id']
                    or o.get('frame')!='base_footprint'
                    or o.get('coordinate_convention')!='x_forward_y_left'
                    or not isinstance(seq,int) or isinstance(seq,bool)
                    or not all(math.isfinite(v) for v in (stamp,x,y))
                    or not 0<=now-stamp<=.5
                    or not .10<x<1.2 or abs(y)>.6):
                raise ValueError('invalid/stale/wrong target')
        except (KeyError,ValueError,TypeError):
            self.confirmed=0
            return stop
        if seq<=self.seq or stamp<=self.stamp:
            stop['reason']='WAIT_NEW_FRAME'
            return stop
        self.seq,self.stamp=seq,stamp
        error=x-self.standoff;bearing=math.atan2(y,x)
        if error<-.02:
            raise ValueError('target too close; reverse requires a separate validated exit')
        if abs(error)<=.015 and abs(y)<=.02 and abs(bearing)<=math.radians(3):
            self.confirmed+=1
            return dict(stop,done=self.confirmed>=3,reason='DOCK_CONFIRM')
        self.confirmed=0
        # Reuse koala's bounded alignment law with calibrated base-relative
        # bearing. +Y is left; legacy right-positive offsets are not accepted.
        wz=self.aligner._turn(-bearing/self.aligner.cfg.center_kp)
        if wz:return dict(stop,wz=wz,reason='ALIGN')
        if error<=.015:return dict(stop,reason='WAIT_LATERAL_ALIGNMENT')
        return dict(stop,vx=.08,reason='APPROACH')


def read(path):
    # Fixed-size latest-only files. Reject unexpectedly large payloads.
    if os.path.getsize(path)>65536:raise ValueError('oversize observation/health')
    with open(path) as f:return json.load(f)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True)
    p.add_argument('--observation',default='/dev/shm/v23_dropoff_target.json')
    p.add_argument('--localization',default='/dev/shm/g2_localization_health.json')
    p.add_argument('--scene',required=True)
    p.add_argument('--real',action='store_true')
    p.add_argument('--seconds',type=int,default=60)
    p.add_argument('--output',default='/tmp/v23_dock_approach.json')
    args=p.parse_args(argv)
    if not 5<=args.seconds<=90:p.error('seconds must be 5..90')
    report={'ok':False,'real':args.real,'arm_or_release_executed':False,'pulses':0}
    motor=None;lease=None;unlocked=False
    # Native ROS stays in its own Python 2 service; no ROS/model imports here.
    sys.path.insert(0,str(ROOT/'deploy/ros'))
    from g2_map_patrol import rpc,available_mb
    try:
        controller=DockController(read(args.config));policy=Policy(args.scene)
        policy.require_motion_ready(.10)
        config=controller.cfg
        if config.get('use_registered_dock_zone'):
            policy.enable_dock_zone(.10)
        if available_mb()<1024:raise RuntimeError('RAM below startup reserve')
        initial=rpc('health');owner=initial['service_pid']
        if (initial['chassis']['client'] is not None or initial['chassis']['motors']!=[0,0,0,0]
                or initial['chassis']['moving']):raise RuntimeError('resident controller is busy')
        def guard():
            health=rpc('health');ch=health['chassis'];loc=read(args.localization);now=time.time()
            if (health['service_pid']!=owner or ch['arm_pose']!='travel' or ch['arm_busy']
                    or ch['board_rx_age_s']>.2 or not health['odom']['valid']):
                raise RuntimeError('resident owner/arm/feedback fault')
            if available_mb()<768:raise RuntimeError('RAM pressure')
            if (loc.get('ready') is not True or not 0<=now-loc['stamp']<=.4
                    or loc.get('owner_pid')!=owner):raise RuntimeError('localization unavailable')
            state=loc['state']
            if not policy.allowed(*state['pose'][:2]):raise RuntimeError('centre outside navigation mask')
            return state
        state=guard();observation=read(args.observation)
        command=controller.step(observation,time.time())
        if command['reason'] in ('WAIT_TARGET','WAIT_NEW_FRAME'):raise RuntimeError('fresh dropoff observation required')
        report.update(preflight=True,initial=state)
        if not args.real:report['ok']=True
        else:
            import fcntl
            lease=open('/tmp/g2_motion_client.lock','a')
            fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if subprocess_owner()!=[str(owner)]:raise RuntimeError('resident is not sole serial owner')
            if not rpc('maintenance-unlock','/tmp/grasp_service.sock').get('ok'):raise RuntimeError('unlock refused')
            unlocked=True;motor=ResidentMotion(guard=guard);motor.connect()
            end=time.time()+args.seconds;lost=None;start_odom=state['odom'];total=0.
            while time.time()<end:
                state=guard()
                total=math.hypot(state['odom'][0]-start_odom[0],state['odom'][1]-start_odom[1])
                if total>.40:raise RuntimeError('visual approach travel limit')
                try:observation=read(args.observation)
                except (OSError,ValueError):observation=None
                command=controller.step(observation,time.time())
                if command['done']:
                    motor.stop();report.update(ok=True,final=state);break
                if not command['vx'] and not command['wz']:
                    motor.stop();lost=lost or time.time()
                    if time.time()-lost>3:raise RuntimeError('target unavailable/alignment timeout')
                    time.sleep(.05);continue
                lost=None;pose=state['pose']
                if command['vx']:
                    if state['front'] is not None and state['front']<.35:raise RuntimeError('front brake')
                    if not policy.stopping_path_allowed(pose,.074):raise RuntimeError('visual stopping reserve crosses mask')
                elif not state.get('turn_clear',False) or not all(policy.allowed(pose[0]+.06*math.cos(i*math.pi/8),pose[1]+.06*math.sin(i*math.pi/8)) for i in range(16)):
                    raise RuntimeError('visual turn reserve crosses mask')
                pulse=time.time()+.08
                try:
                    while time.time()<pulse:
                        current=guard()
                        if command['wz'] and not current.get('turn_clear',False):raise RuntimeError('live visual turn clearance')
                        # Require the same target to stay fresh during the pulse.
                        if not 0<=time.time()-observation['stamp']<=.5:raise RuntimeError('target expired during pulse')
                        if command['vx'] and ((current['front'] is not None and current['front']<.35)
                                or not policy.stopping_path_allowed(current['pose'],.074)):
                            raise RuntimeError('live visual stopping gate')
                        motor.set_car_motion(command['vx'],0,command['wz']);time.sleep(.02)
                finally:motor.stop()
                report['pulses']+=1
                settle=time.time()+2;stopped=False
                while time.time()<settle:
                    guard();ch=rpc('health')['chassis']
                    if ch['motors']==[0,0,0,0] and not ch['moving'] and ch['stopped_for_s']>=.5:
                        stopped=True;break
                    time.sleep(.05)
                if not stopped:raise RuntimeError('stop feedback timeout')
            if not report['ok']:raise RuntimeError('visual approach duration limit')
    except (Exception,KeyboardInterrupt) as exc:report['error']=str(exc) or 'interrupted'
    finally:
        if motor:motor.close()
        if unlocked:
            try:
                until=time.time()+3
                while time.time()<until:
                    ch=rpc('health')['chassis']
                    if ch['motors']==[0,0,0,0] and not ch['moving']:
                        if not rpc('maintenance-lock','/tmp/grasp_service.sock').get('ok'):raise RuntimeError('lock refused')
                        break
                    time.sleep(.1)
                else:raise RuntimeError('stop/lock timeout')
            except Exception as exc:report.update(ok=False,cleanup_error=str(exc))
        if lease:lease.close()
        with open(args.output,'w') as f:json.dump(report,f,indent=2)
    print(json.dumps(report));return 0 if report['ok'] else 1


def subprocess_owner():
    import subprocess
    return subprocess.check_output(['fuser','/dev/myserial']).decode().split()


if __name__=='__main__':sys.exit(main())
