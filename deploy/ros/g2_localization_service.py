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
import socket
import subprocess
import sys
import time
from g2_map_patrol import Sensors, rpc, available_mb
from g2_localization_policy import RestartBudget, propagated_seed, restart_pose, resident_stopped, turn_clear, FaultDelay, RecoveryWatchdog


def monotonic_seconds():
    if hasattr(time,'monotonic'):return time.monotonic()
    # ROS Melodic Python 2 lacks time.monotonic; uptime is independent of NTP.
    with open('/proc/uptime') as f:return float(f.read().split()[0])


def systemd_notify(message):
    address=os.environ.get('NOTIFY_SOCKET')
    if not address:return False
    if address.startswith('@'):address='\0'+address[1:]
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sock.settimeout(.2)
    try:
        sock.connect(address);sock.sendall(message.encode('utf-8'));return True
    except socket.error:return False
    finally:sock.close()


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
    p.add_argument('--last-good',default='/dev/shm/g2_localization_last_good.json')
    p.add_argument('--restart-budget',default='/dev/shm/g2_localization_restart_budget.json')
    p.add_argument('--wait-seconds',type=int,default=20,help='5..60s before reporting degraded; never drive with expired data')
    args=p.parse_args()
    if not 5<=args.wait_seconds<=60:p.error('--wait-seconds must be 5..60')
    # roslaunch starts map_server from its own directory, not this unit's cwd.
    args.map=os.path.abspath(args.map)
    rospy.init_node('g2_localization_supervisor',disable_signals=True)
    sensors=Sensors();child=None;seed=None;consumed_stamp=None;external_pending=False
    budget=RestartBudget();bad_since=None;launched=0;launched_clock=0;running=[True];input_fault=FaultDelay(8)
    watchdog=RecoveryWatchdog(args.wait_seconds);last_notice=None;last_notice_at=-1e9
    def stop_signal(*_):running[0]=False
    signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
    with open('/proc/sys/kernel/random/boot_id') as f:boot=f.read().strip()
    digest=map_hash(args.map)
    try:
        with open(args.restart_budget) as f:budget.restore(json.load(f),monotonic_seconds(),boot)
    except IOError:pass
    launch=os.path.join(os.path.dirname(__file__),'g2_scene_localization.launch')
    def stop_child():
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            until=monotonic_seconds()+4
            while child.poll() is None and monotonic_seconds()<until:time.sleep(.1)
            if child.poll() is None:os.killpg(child.pid,signal.SIGKILL)
            child.wait()
    systemd_notify('READY=1\nSTATUS=AMCL supervisor running; robot readiness is in health JSON')
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
                out['scan_age_s']=None if scan is None else round(now-scan[1],3)
                out['odom_age_s']=None if od is None else round(now-od[1],3)
                out['board_rx_age_s']=ch['board_rx_age_s']
                if seed is not None and seed['owner_pid']!=owner:
                    seed=None;external_pending=False;stop_child()
                    raise RuntimeError('owner changed; fresh pose reference required')
                if out['available_mb']<768:raise RuntimeError('RAM pressure')
                if ch['board_rx_age_s']>.5:raise RuntimeError('board feedback stale')
                if od is None or now-od[0]>.3 or not 0<=now-od[1]<=.4 or not health['odom']['valid']:
                    raise RuntimeError('waiting for fresh odom')
                if scan is None or now-scan[0]>.4 or not 0<=now-scan[1]<=.5:
                    raise RuntimeError('waiting for fresh scan')
                input_fault.update(False,monotonic_seconds())
                # A quick supervisor crash may reuse only an independently
                # validated last-good pose <=15s old, with unchanged bindings
                # and bounded odom motion. Never renew an expired cache.
                if seed is None:
                    try:
                        with open(args.last_good) as f:cached=json.load(f)
                        restart_pose(cached,od[4],now,boot,owner,digest)
                        seed=cached;external_pending=False
                        consumed_stamp=cached.get('consumed_external_stamp')
                        out['restored_last_good']=True
                    except (IOError,ValueError,KeyError,TypeError) as exc:out['last_good_warning']=str(exc)
                # A supplied reference may be consumed only once, even if a child dies.
                try:
                    with open(args.seed) as f:supplied=json.load(f)
                    if supplied['stamp']!=consumed_stamp:
                        propagated_seed(supplied,od[4],now,boot,owner,digest,120)
                        if resident_stopped(ch):
                            # Explicit reference submission is an actual reseed,
                            # not merely comparison against the old AMCL pose.
                            if child is not None and child.poll() is None:stop_child()
                            seed=supplied;consumed_stamp=supplied['stamp'];external_pending=True
                        else:out['supplied_seed_warning']='new reference deferred until resident stopped and arm idle'
                except (IOError,ValueError,KeyError,TypeError) as exc:out['supplied_seed_warning']=str(exc)
                if child is not None and child.poll() is None:
                    try:
                        state=sensors.state()
                        with sensors.lock:
                            amcl_received=sensors.amcl[0] if sensors.amcl else 0
                            amcl_stamp=sensors.amcl[1] if sensors.amcl else 0
                        if min(amcl_received,amcl_stamp)<launched:raise RuntimeError('waiting for new AMCL publication after restart')
                        if seed is None or seed['owner_pid']!=owner:raise RuntimeError('owner changed')
                        # A jump must not turn into an automatic trusted restart seed.
                        expected=propagated_seed(seed,state['odom'],now,boot,owner,digest,120)
                        delta=((state['pose'][2]-expected[2]+3.141592653589793)%6.283185307179586)-3.141592653589793
                        if ((state['pose'][0]-expected[0])**2+(state['pose'][1]-expected[1])**2)>.15**2 or abs(delta)>.20:
                            raise RuntimeError('localization jump; new reference required')
                        seed={'stamp':now,'boot_id':boot,'owner_pid':owner,'map_hash':digest,'pose':state['pose'],'odom':state['odom']}
                        external_pending=False
                        # front may be infinity in an empty view; JSON must stay strict.
                        if state['front']==float('inf'):state['front']=None
                        state['turn_clear']=turn_clear(scan[3])
                        atomic(args.last_good,dict(seed,consumed_external_stamp=consumed_stamp))
                        out.update(ready=True,status='localized',state=state,seed=seed)
                        bad_since=None
                    except Exception as exc:
                        out.update(status='waiting_for_localization',reason=str(exc))
                        if bad_since is None:bad_since=monotonic_seconds()
                        if monotonic_seconds()-launched_clock>=12 and monotonic_seconds()-bad_since>=8:stop_child()
                else:
                    if seed is None:raise RuntimeError('fresh pose seed required')
                    pose=restart_pose(seed,od[4],now,boot,owner,digest,external_pending)
                    if not resident_stopped(ch):raise RuntimeError('waiting for stopped resident')
                    if out['available_mb']<1024:raise RuntimeError('RAM below startup reserve')
                    if not budget.take(monotonic_seconds()):raise RuntimeError('localization restart budget/cooldown')
                    atomic(args.restart_budget,{'boot_id':boot,'attempts':budget.attempts})
                    # Child receives no duration limit. Kill only this process group.
                    command=['roslaunch',launch,'map:='+args.map,'initial_x:='+str(pose[0]),'initial_y:='+str(pose[1]),'initial_yaw:='+str(pose[2])]
                    child=subprocess.Popen(command,preexec_fn=os.setsid)
                    launched=time.time();launched_clock=monotonic_seconds();bad_since=launched_clock
                    out['status']='starting_localization'
            except Exception as exc:
                out['reason']=str(exc)
                out.update(ready=False,status='waiting_for_recovery')
                # Short sensor/RPC outages pause navigation without needlessly
                # killing healthy AMCL or consuming the restart budget.
                persistent=input_fault.update(True,monotonic_seconds())
                if child is not None and child.poll() is None and (persistent or str(exc)=='RAM pressure'):
                    stop_child()
            clock_now=monotonic_seconds()
            out['restart_attempts_10min']=sum(clock_now-t<600 for t in budget.attempts)
            out['retry_after_s']=round(budget.retry_after(clock_now),3)
            out['reference_kind']='none' if seed is None else ('external_pending' if external_pending else 'last_good_amcl')
            out['reference_age_s']=None if seed is None else round(time.time()-seed['stamp'],3)
            out['accepted_reference_stamp']=consumed_stamp
            out['amcl_pid']=child.pid if child is not None and child.poll() is None else None
            watchdog.annotate(out,clock_now)
            atomic(args.health,out)
            notice=(out['status'],out['reason_code'])
            if notice!=last_notice and clock_now-last_notice_at>=2:
                print(json.dumps({key:out.get(key) for key in ('status','reason_code','next_action','wait_elapsed_s','retry_after_s')}))
                sys.stdout.flush();last_notice=notice;last_notice_at=clock_now
            systemd_notify('WATCHDOG=1\nSTATUS='+out['status']+': '+out['reason_code'])
            time.sleep(.1)
    finally:
        stop_child()
        atomic(args.health,{'stamp':time.time(),'ready':False,'status':'stopped'})
        systemd_notify('STOPPING=1')


if __name__=='__main__':main()
