#!/usr/bin/env python
"""Staged, bounded paper-box approach through the existing patrol gates."""
from __future__ import division
import argparse
import json
import math
import signal
import socket
import subprocess
import sys
import time

from g2_map_patrol import Policy,Runner,Sensors,available_mb,rpc,wrap,checkpoint,LIDAR_FORWARD_M


def dock_staging(policy):
    d=policy.layout['bin']['dock_candidate'];yaw=float(d['yaw'])
    goal=[float(d['x']),float(d['y'])]
    stage=[goal[0]-.18*math.cos(yaw),goal[1]-.18*math.sin(yaw)]
    if not policy.segment_allowed(stage,goal):raise ValueError('dock approach crosses mask')
    return stage,goal,yaw


def final_approach_geometry(pose,goal):
    dx,dy=goal[0]-pose[0],goal[1]-pose[1]
    ahead=dx*math.cos(pose[2])+dy*math.sin(pose[2])
    lateral=-dx*math.sin(pose[2])+dy*math.cos(pose[2])
    return math.hypot(dx,dy),ahead,lateral


def require_box_registration(policy):
    if policy.layout['bin'].get('status')=='dock_candidate_failed_physical_alignment_recalibration_required':
        raise ValueError('physical box alignment failed; re-register dock before more motion')


def approach_without_turn(runner,goal):
    """Final straight micro-pulses; reject lateral/yaw error instead of rotating."""
    for _ in range(50):
        state=runner.ready();distance,ahead,lateral=final_approach_geometry(state['pose'],goal)
        if distance<=.045:
            final=runner.settle();error=math.hypot(goal[0]-final['pose'][0],goal[1]-final['pose'][1])
            if error<=.065:
                runner.report['arrivals'].append({'label':'box_dock_candidate','goal':goal,'pose':final['pose'],'error_m':error,'cov':final['cov']})
                checkpoint(runner.report);return
        if distance>.24 or ahead<=0 or abs(lateral)>.02 or abs(math.atan2(lateral,ahead))>math.radians(10):
            raise RuntimeError('dock straight approach alignment outside bounds')
        runner.pulse(.15,0,.08 if distance>.08 else .04)
    raise RuntimeError('dock straight approach iteration limit')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',required=True)
    p.add_argument('--output',required=True);p.add_argument('--real',action='store_true')
    p.add_argument('--short-final-approach',action='store_true');p.add_argument('--resume-final',action='store_true')
    p.add_argument('--local-dock-zone',action='store_true');args=p.parse_args()
    if args.resume_final and not args.short_final_approach:p.error('resume requires short-final-approach')
    if args.local_dock_zone and not args.short_final_approach:p.error('local dock zone requires short-final-approach')
    report={'ok':False,'real':args.real,'output_path':args.output,'trace':[],'arrivals':[],'recoveries':[],'pulses':0}
    runner=None
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    try:
        policy=Policy(args.scene);stage,goal,yaw=dock_staging(policy)
        if args.real:
            policy.require_motion_ready(LIDAR_FORWARD_M);require_box_registration(policy)
        if args.local_dock_zone:
            policy.enable_dock_zone(LIDAR_FORWARD_M);policy.dock_cells=None
        report['stage']=stage;report['goal']=goal;report['goal_yaw']=yaw
        if available_mb()<1024:raise RuntimeError('RAM below 1GB at start')
        h=rpc('health');ch=h['chassis']
        if subprocess.check_output(['fuser','/dev/myserial']).decode().split()!=[str(h['service_pid'])]:raise RuntimeError('sole owner check failed')
        if ch['motors']!=[0,0,0,0] or ch['moving'] or ch['client'] is not None:raise RuntimeError('owner not idle')
        import rospy
        rospy.init_node('g2_dock_probe',disable_signals=True)
        runner=Runner(Sensors(),policy,report,300);report['initial']=runner.ready(8)
        report['planned_stage_path']=policy.plan(report['initial']['pose'][:2],stage)
        if args.real:
            if not args.resume_final and not runner.rotation_allowed(report['initial']['pose']):raise RuntimeError('initial turn reserve not available')
            if not rpc('maintenance-unlock','/tmp/grasp_service.sock').get('ok'):raise RuntimeError('unlock refused')
            runner.sock=socket.create_connection(('127.0.0.1',7000),timeout=1)
            runner.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            if not args.resume_final:
                runner.goto(stage,'box_staging')
                for _ in range(50):
                    current=runner.ready()['pose'];error=wrap(math.atan2(goal[1]-current[1],goal[0]-current[0])-current[2])
                    if abs(error)<math.radians(3):break
                    runner.pulse(0,1 if error>0 else -1,min(.12,max(.04,(abs(error)-.02)/1.5)))
                else:raise RuntimeError('dock prealignment limit')
            # Keep all existing live scan/mask/stop checks. No turn after arrival,
            # no release or unvalidated reverse departure in this probe.
            runner.report.pop('last_planned_goal',None)
            if args.short_final_approach:
                if args.local_dock_zone:policy.enable_dock_zone(LIDAR_FORWARD_M)
                # Only this final known-mask segment: body front 20cm + 15cm
                # clearance at the brake, versus <=8.8cm projected pulse+coast.
                runner.front_brake_m=.35;runner.max_forward_duration=.08
                report['final_approach_policy']={'front_brake_m':.35,'max_pulse_s':.08,'max_projected_travel_m':.088,'body_front_m':.20}
                approach_without_turn(runner,goal)
            else:runner.goto(goal,'box_dock_candidate')
        report['final']=runner.settle();report['ok']=True
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc) or 'interrupted'
        if runner is not None:
            try:report['final']=runner.settle()
            except Exception as final_exc:report['final_error']=str(final_exc)
    finally:
        if runner is not None:
            runner.stop()
            if runner.sock is not None:runner.sock.close()
        if args.real:
            try:
                for _ in range(30):
                    h=rpc('health');ch=h['chassis']
                    if ch['motors']==[0,0,0,0] and not ch['moving']:
                        if rpc('maintenance-lock','/tmp/grasp_service.sock').get('ok'):break
                    time.sleep(.1)
                else:raise RuntimeError('maintenance lock timeout')
                report['final_health']=rpc('health')
            except Exception as exc:report['cleanup_error']=str(exc);report['ok']=False
        checkpoint(report)
    print(json.dumps({k:report.get(k) for k in ('ok','error','pulses','arrivals','final')}))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
