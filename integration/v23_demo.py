#!/usr/bin/env python3
"""Finalist v23 demo: bounded plans over the resident service, never serial.

Default is plan validation/preview. Execution never retries an arm command or
resumes a failed plan. This runner is not autonomous AMCL map patrol.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
VISION_HEALTH = '/tmp/x3plus_vision_health.json'
ARM_TIMEOUTS = {'home':60, 'grasp':180, 'stow':60, 'release':120}


def validate_plan(plan):
    if not isinstance(plan,dict) or set(plan)-{'version','steps','orientation_evidence'}:
        raise ValueError('unknown plan fields')
    if plan.get('version') != 'v23':
        raise ValueError('plan must use v23')
    steps = plan.get('steps')
    if not isinstance(steps,list) or not 1 <= len(steps) <= 20:
        raise ValueError('plan requires 1..20 steps')
    pose, holding = 'travel', False
    for step in steps:
        if not isinstance(step,dict):
            raise ValueError('step must be an object')
        command = step.get('action')
        allowed = {'action','distance_m'} if command == 'drive' else {'action'}
        if set(step)-allowed or command not in set(ARM_TIMEOUTS)|{'drive'}:
            raise ValueError('unknown action or fields')
        if command == 'drive':
            distance = step.get('distance_m')
            if type(distance) not in (int,float) or not math.isfinite(distance) or \
                    not 0.10 <= distance <= 0.25 or pose != 'travel':
                raise ValueError('drive requires travel pose and 0.10..0.25 m')
            if not isinstance(plan.get('orientation_evidence'),str) or not plan['orientation_evidence']:
                raise ValueError('drive requires measured orientation evidence')
        elif command == 'home':
            if holding:
                raise ValueError('home would open a loaded jaw; use release')
            pose = 'e1'
        elif command == 'grasp':
            if pose != 'e1' or holding:
                raise ValueError('grasp requires empty E1 pose')
            holding = True
        elif command == 'release':
            if not holding:
                raise ValueError('release requires a confirmed grasp')
            pose, holding = 'e1', False
        else:
            pose = 'travel'
    if pose != 'travel':
        raise ValueError('finish with stow to leave the chassis driveable')
    return steps


def service_request(command, timeout=2):
    # No retry: after a timeout the server may still be completing the motion.
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect('/tmp/grasp_service.sock')
        connection.sendall((command+'\n').encode())
        with connection.makefile('rb') as stream:
            return json.loads(stream.readline(65536))


class ResidentBackend:
    def __init__(self, orientation_evidence=''):
        self.orientation_evidence = orientation_evidence

    def health(self):
        return service_request('health')

    def guard(self, expected_pose, holding):
        import g2_safety_audit as audit
        status = self.health()
        check_health(status,expected_pose,holding)
        if not audit.owner_matches():
            raise RuntimeError('serial owner is not grasp-service')
        if audit.available_mb() < 1024:
            raise RuntimeError('RAM reserve below 1024 MB before demo step')
        pid = int(audit.shell(['systemctl','show','grasp-vision','-p','MainPID','--value']).stdout.strip())
        if audit.vision_faults(VISION_HEALTH,time.monotonic(),pid):
            raise RuntimeError('vision frames/inference missing or stale')
        return status

    def arm(self, command):
        return service_request(command,ARM_TIMEOUTS[command])

    def drive(self, distance):
        import g2_safety_audit as audit
        argv = [sys.executable,str(ROOT/'integration/g2_nav_client.py'),'--real',
                '--distance-m',str(distance),'--lidar-orientation-evidence',
                self.orientation_evidence,'--i-confirm-clear-path','--i-confirm-power-cut']
        child = subprocess.Popen(argv)
        try:
            deadline = time.monotonic()+20
            while child.poll() is None:
                if audit.available_mb() < 768 or time.monotonic() >= deadline:
                    raise RuntimeError('RAM_pressure_or_drive_timeout')
                time.sleep(0.1)
            if child.returncode:
                raise RuntimeError('bounded navigation failed; no next step')
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill();child.wait()


def check_health(status, pose, holding):
    ch, od = status.get('chassis') or {}, status.get('odom') or {}
    age = ch.get('board_rx_age_s')
    if status.get('stack_version') != 'v23' or not status.get('ok') or \
            not status.get('chassis_tcp_ready'):
        raise RuntimeError('v23 resident service not ready')
    if Path(status.get('model_path','')).name != 'candidate_v23_seed23401_ckpt250000.zip' or \
            Path(status.get('vecnorm_path','')).name != 'candidate_v23_seed23401_ckpt250000_vec.pkl':
        raise RuntimeError('resident model pair is not the v23 finalist pair')
    if ch.get('motors') != [0,0,0,0] or ch.get('moving') is not False or \
            ch.get('arm_busy') is not False or ch.get('client') is not None or \
            ch.get('maintenance') is not False or ch.get('arm_pose') != pose:
        raise RuntimeError('resident pose/idle/maintenance gate')
    if type(age) not in (int,float) or not math.isfinite(age) or not 0 <= age <= 0.2:
        raise RuntimeError('stale board feedback')
    if status.get('holding') is not holding:
        raise RuntimeError('held object state differs from plan; do not replay')
    twist = od.get('twist')
    if not od.get('valid') or not isinstance(twist,list) or len(twist) != 3 or \
            not all(type(v) in (int,float) and math.isfinite(v) and abs(v)<=0.01 for v in twist):
        raise RuntimeError('odom is not valid and stationary')


def emit_telemetry(telemetry,index,action,state):
    phase = {'home':'ALIGN','grasp':'GRASP','stow':'CARRY_HOME',
             'drive':'APPROACH','release':'PLACE','abort':'FAULT'}[action]
    return telemetry.say(phase,action.upper(),state,step=index,
                         chassis_allowed=action=='drive' and state=='running',
                         arm_allowed=action in ARM_TIMEOUTS and state=='running')


def run_plan(plan, backend, report=None):
    steps = validate_plan(plan)
    pose, holding = 'travel', False
    for index, step in enumerate(steps):
        backend.guard(pose,holding)
        action = step['action']
        if report:
            report(index,action,'running')
        if action == 'drive':
            backend.drive(step['distance_m'])
        else:
            result = backend.arm(action)
            if not result.get('ok') or (action=='grasp' and not result.get('confirmed')):
                raise RuntimeError('%s failed: %s' % (action,result))
            if action == 'home':
                pose = 'e1'
            elif action == 'grasp':
                holding = True
            elif action == 'release':
                pose, holding = 'e1', False
            else:
                pose = 'travel'
        backend.guard(pose,holding)
        if report:
            report(index,action,'complete')
    return {'ok':True,'version':'v23','steps_completed':len(steps),'holding':holding}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--dry-run',action='store_true')
    parser.add_argument('--output',default='/tmp/v23_demo_state.json')
    parser.add_argument('--status-udp',default=None)
    args = parser.parse_args(argv)
    plan_path = Path(args.plan).resolve()
    plan = json.loads(plan_path.read_text())
    validate_plan(plan)
    if not args.execute:
        print(json.dumps({'validated':True,'version':'v23','executes_hardware':False,
                          'plan':plan},indent=2))
        return 0
    # One demo controller at a time; this lock never opens the robot serial port.
    import fcntl
    with open('/tmp/v23_demo.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        evidence = plan.get('orientation_evidence','')
        if evidence:
            evidence = str((plan_path.parent/evidence).resolve())
        from mission_status import SimpleReporter
        telemetry = SimpleReporter(args.status_udp,mode='D')
        def report(index,action,state):
            sample = dict(version='v23',index=index,action=action,state=state,
                          pid=os.getpid(),monotonic=time.monotonic())
            target = Path(args.output)
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(sample)+'\n');temporary.replace(target)
            print(json.dumps(sample),flush=True)
            emit_telemetry(telemetry,index,action,state)
        try:
            result = run_plan(plan,ResidentBackend(evidence),report)
            telemetry.say('COMPLETE','STOP','v23 plan completed')
        except (Exception,KeyboardInterrupt) as exc:
            report(-1,'abort',str(exc) or 'interrupted')
            raise
        finally:
            telemetry.close()
        print(json.dumps(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
