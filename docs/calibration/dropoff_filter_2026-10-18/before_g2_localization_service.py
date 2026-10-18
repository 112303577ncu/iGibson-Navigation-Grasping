#!/usr/bin/env python
"""Persistent AMCL supervision; no Serial or velocity commands.

No automatic corner guessing. A fresh seed is bound to boot, owner and raw map.
Old/ambiguous references wait for a new seed instead of restarting from home.
"""
from __future__ import division
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from g2_map_patrol import Sensors, rpc, available_mb
from g2_localization_policy import RestartBudget, propagated_seed, turn_clear, FaultDelay


def atomic(path, value):
    with open(path+'.tmp','w') as f:json.dump(value,f,allow_nan=False)
    os.rename(path+'.tmp',path)


def map_hash(path):
    import yaml
    with open(path) as f:meta=yaml.safe_load(f)
    image=os.path.join(os.path.dirname(path),meta['image'])
    h=hashlib.sha256()
    for name in (path,image):
        with open(name,'rb') as f:h.update(f.read())
    h.update(b'lidar_forward_m=0.10')
    return h.hexdigest()


def main():
    import rospy
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',required=True)
    p.add_argument('--seed',default='/dev/shm/g2_localization_seed.json')
    p.add_argument('--health',default='/dev/shm/g2_localization_health.json')
    args=p.parse_args()
    # roslaunch starts map_server from its own directory, not this unit's cwd.
    args.map=os.path.abspath(args.map)
    rospy.init_node('g2_localization_supervisor',disable_signals=True)
    sensors=Sensors();child=None;seed=None;consumed_stamp=None
    budget=RestartBudget();bad_since=None;launched=0;running=[True];input_fault=FaultDelay(8)
    def stop_signal(*_):running[0]=False
    signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
    with open('/proc/sys/kernel/random/boot_id') as f:boot=f.read().strip()
    digest=map_hash(args.map)
    launch=os.path.join(os.path.dirname(__file__),'g2_scene_localization.launch')
    def stop_child():
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            until=time.time()+4
            while child.poll() is None and time.time()<until:time.sleep(.1)
            if child.poll() is None:os.killpg(child.pid,signal.SIGKILL)
            child.wait()
    try:
        while running[0] and not rospy.is_shutdown():
            now=time.time();out={'stamp':now,'ready':False,'status':'waiting_for_seed','map_hash':digest,'boot_id':boot}
            try:
                health=rpc('health');owner=health['service_pid'];ch=health['chassis']
                out['owner_pid']=owner;out['available_mb']=available_mb()
                with sensors.lock:od=sensors.odom;scan=sensors.scan
                # Read the clock after taking the callback snapshot. A callback
                # arriving during health RPC must not look like future data.
                now=time.time();out['stamp']=now
                if seed is not None and seed['owner_pid']!=owner:
                    seed=None;stop_child()
                    raise RuntimeError('owner changed; fresh pose reference required')
                if out['available_mb']<768:raise RuntimeError('RAM pressure')
                if (od is None or scan is None or now-od[0]>.3 or now-scan[0]>.4
                        or not 0<=now-od[1]<=.4 or not 0<=now-scan[1]<=.5
                        or ch['board_rx_age_s']>.5 or not health['odom']['valid']):
                    raise RuntimeError('waiting for fresh board/odom/scan')
                input_fault.update(False,now)
                # A supplied reference may be consumed only once, even if a child dies.
                try:
                    with open(args.seed) as f:supplied=json.load(f)
                    if supplied['stamp']!=consumed_stamp:
                        propagated_seed(supplied,od[4],now,boot,owner,digest,120)
                        seed=supplied;consumed_stamp=supplied['stamp']
                except (IOError,ValueError,KeyError,TypeError):pass
                if child is not None and child.poll() is None:
                    try:
                        state=sensors.state()
                        with sensors.lock:amcl_received=sensors.amcl[0] if sensors.amcl else 0
                        if amcl_received<launched:raise RuntimeError('waiting for new AMCL publication after restart')
                        if seed is None or seed['owner_pid']!=owner:raise RuntimeError('owner changed')
                        # A jump must not turn into an automatic trusted restart seed.
                        expected=propagated_seed(seed,state['odom'],now,boot,owner,digest,120)
                        delta=((state['pose'][2]-expected[2]+3.141592653589793)%6.283185307179586)-3.141592653589793
                        if ((state['pose'][0]-expected[0])**2+(state['pose'][1]-expected[1])**2)>.15**2 or abs(delta)>.20:
                            raise RuntimeError('localization jump; new reference required')
                        seed={'stamp':now,'boot_id':boot,'owner_pid':owner,'map_hash':digest,'pose':state['pose'],'odom':state['odom']}
                        # front may be infinity in an empty view; JSON must stay strict.
                        if state['front']==float('inf'):state['front']=None
                        state['turn_clear']=turn_clear(scan[3])
                        out.update(ready=True,status='localized',state=state,seed=seed)
                        bad_since=None
                    except Exception as exc:
                        out.update(status='waiting_for_localization',reason=str(exc))
                        bad_since=bad_since or now
                        if now-launched>12 and now-bad_since>8:stop_child()
                else:
                    if seed is None:raise RuntimeError('fresh pose seed required')
                    pose=propagated_seed(seed,od[4],now,boot,owner,digest)
                    if (ch['motors']!=[0,0,0,0] or ch['moving'] or ch['arm_busy']
                            or ch['stopped_for_s']<.5):raise RuntimeError('waiting for stopped resident')
                    if out['available_mb']<1024:raise RuntimeError('RAM below startup reserve')
                    if not budget.take(now):raise RuntimeError('localization restart budget/cooldown')
                    # Child receives no duration limit. Kill only this process group.
                    command=['roslaunch',launch,'map:='+args.map,'initial_x:='+str(pose[0]),'initial_y:='+str(pose[1]),'initial_yaw:='+str(pose[2])]
                    child=subprocess.Popen(command,preexec_fn=os.setsid)
                    launched=now;bad_since=now
                    out['status']='starting_localization'
                out['restart_attempts_10min']=len(budget.attempts)
            except Exception as exc:
                out['reason']=str(exc)
                out.update(ready=False,status='waiting_for_recovery')
                # Short sensor/RPC outages pause navigation without needlessly
                # killing healthy AMCL or consuming the restart budget.
                persistent=input_fault.update(True,time.time())
                if child is not None and child.poll() is None and (persistent or str(exc)=='RAM pressure'):
                    stop_child()
            atomic(args.health,out)
            time.sleep(.1)
    finally:
        stop_child()
        atomic(args.health,{'stamp':time.time(),'ready':False,'status':'stopped'})


if __name__=='__main__':main()
