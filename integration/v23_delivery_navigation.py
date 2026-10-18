#!/usr/bin/env python3
"""Stepwise patrol/dropoff navigation. Arm actions remain teammate-owned.

No implicit release/retry and no replay of an old relative motion correction.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]


def atomic(path,value):
    with open(str(path)+'.tmp','w') as f:json.dump(value,f,indent=2)
    os.replace(str(path)+'.tmp',str(path))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=('return','patrol','dropoff'),required=True)
    p.add_argument('--scene',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--visual-config')
    p.add_argument('--observation',default='/dev/shm/v23_dropoff_target.json')
    p.add_argument('--real',action='store_true')
    args=p.parse_args(argv)
    path=Path(args.output);report={'ok':False,'real':args.real,'task':args.task,'stages':[],
        'arm_or_release_executed':False,'release_ready':False,'mission_complete':False,'started_at':time.time()}
    lease=None
    try:
        import fcntl
        lease=open('/tmp/g2_delivery_flow.lock','a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        destination={'return':'nearest-patrol','patrol':'patrol-cycle','dropoff':'dropoff-preparation'}[args.task]
        nav_path=str(path)+'.navigation.json'
        command=['/bin/bash','-c','source /opt/ros/melodic/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1; exec /usr/bin/python "$@"','g2-native',
                 str(ROOT/'deploy/ros/g2_map_patrol.py'),'--scene',str(Path(args.scene).resolve()),
                 '--destination',destination,'--inset','.70','--boundary-micro','--keep-heading','--seconds','600','--output',nav_path]
        if args.real:command.append('--real')
        report['stages'].append({'name':'MAP_NAVIGATION','state':'running','report':nav_path});atomic(path,report)
        process=subprocess.Popen(command)
        try:code=process.wait(timeout=620)
        except BaseException:
            process.terminate()
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            raise
        with open(nav_path) as f:nav=json.load(f)
        if code or not nav.get('ok'):raise RuntimeError('map navigation: '+str(nav.get('error',code)))
        report['stages'][-1]['state']='passed' if args.real else 'preflight_passed'
        report['navigation_ok']=True
        if args.task!='dropoff':
            report.update(ok=True,next_stage='PATROL_RECOGNITION' if args.real else 'READY_FOR_NAVIGATION')
        elif not args.visual_config:
            report.update(next_stage='WAIT_VISUAL_TARGET',reason='paper-box detector or ArUco and calibrated target config are required; preparation arrival is not release readiness')
            report['stages'].append({'name':'VISUAL_DOCK','state':'waiting_for_target_configuration'})
        else:
            dock_path=str(path)+'.dock.json'
            command=[sys.executable,str(ROOT/'integration/v23_dock_approach.py'),
                     '--scene',str(Path(args.scene).resolve()),'--config',args.visual_config,
                     '--observation',args.observation,'--output',dock_path]
            if args.real:command.append('--real')
            report['stages'].append({'name':'VISUAL_DOCK','state':'running','report':dock_path});atomic(path,report)
            code=subprocess.call(command)
            with open(dock_path) as f:dock=json.load(f)
            if code or not dock.get('ok'):raise RuntimeError('visual dock: '+str(dock.get('error',code)))
            report['stages'][-1]['state']='passed' if args.real else 'preflight_passed'
            # Teammate must consume this unique operation once and return an
            # explicit outcome. Never infer release success from a lost socket.
            report.update(ok=True,next_stage='AWAIT_TEAMMATE_RELEASE' if args.real else 'READY_FOR_VISUAL_DOCK',
                          release_ready=bool(args.real),release_token=uuid.uuid4().hex if args.real else None)
            report['stages'].extend([{'name':'RELEASE','state':'teammate_owned_not_executed'},
                                     {'name':'EXIT_AND_RETURN','state':'pending_release_result_and_exit_validation'}])
    except (Exception,KeyboardInterrupt) as exc:
        report.update(error=str(exc) or 'interrupted',next_stage='WAIT_REPAIR_OR_LOCALIZATION')
    finally:
        if lease:lease.close()
        report['finished_at']=time.time();atomic(path,report)
    print(json.dumps(report))
    if report['ok']:return 0
    # A deliberate handoff wait differs from a hardware/navigation failure.
    return 2 if report.get('navigation_ok') and report.get('next_stage')=='WAIT_VISUAL_TARGET' else 1


if __name__=='__main__':sys.exit(main())
