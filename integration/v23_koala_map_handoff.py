#!/usr/bin/env python3
"""Jetson map transit to a surveyed camera handoff; never operates the arm.

The Windows mission stops vision control before invoking this process, and
unlocks the resident only after this process has exited successfully.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from integration import v23_task_resume_guard as guard
from integration.g2_patrol_policy import Policy

DEFAULT_CONFIG = ROOT / 'integration/placement_navigation_handoff.example.json'


def point(value):
    if (not isinstance(value, list) or len(value) != 2 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in value)):
        raise ValueError('handoff needs a finite map point')
    return value


def load_config(path, scene, expected_hash=None):
    path = Path(path)
    if expected_hash and hashlib.sha256(path.read_bytes()).hexdigest() != expected_hash:
        raise ValueError('PC/Jetson handoff configuration mismatch')
    config = json.loads(path.read_text(encoding='utf-8'))
    if config.get('schema') != 1 or config.get('scene') != scene:
        raise ValueError('handoff schema/scene mismatch')
    for flag in ('verified', 'box_registration_verified', 'camera_handoff_verified'):
        if config.get(flag) is not True:
            raise ValueError('WAIT_HANDOFF_CALIBRATION: ' + flag)
    if not config.get('calibration_id') or not config.get('map_hash'):
        raise ValueError('handoff calibration/map identity missing')
    goal, target = point(config.get('point_map')), point(config.get('box_center_map'))
    # Koala's current close-range visual acquisition cannot initialize <0.8m.
    if not .90 <= math.hypot(goal[0]-target[0], goal[1]-target[1]) <= 1.40:
        raise ValueError('visual handoff must start 0.90..1.40 m from box centre')
    policy = Policy(str(ROOT / scene))
    policy.require_motion_ready(.10)
    if not policy.allowed(*goal):
        raise ValueError('handoff point outside navigation centre mask')
    return config


def snapshot():
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    return (guard.resident_health(), guard.read_json('/dev/shm/g2_localization_health.json'),
            boot, guard.available_mb())


def ready(config, expected_holding=True):
    health, loc, boot, ram = snapshot()
    if health.get('holding') is not expected_holding:
        raise ValueError('holding state disagrees with delivery/reentry phase')
    if loc.get('map_hash') != config['map_hash']:
        raise ValueError('handoff raw-map identity mismatch')
    if ram < 1024:
        raise ValueError('RAM below 1024 MB before transit')
    checkpoint = guard.begin_checkpoint('idle', health, loc, boot, time.time(), next_task='dropoff')
    result = guard.run_resume(checkpoint, snapshot, wait_seconds=20)
    if not result.get('ok'):
        raise ValueError('resume check: ' + str(result))
    # Holding is independent from the localization checkpoint.
    current = guard.resident_health()
    if current.get('holding') is not expected_holding:
        raise ValueError('holding state changed during resume check')
    return result


def navigation_command(scene, config, output, real):
    command = ['/bin/bash', '-c',
               'source /opt/ros/melodic/setup.bash; '
               'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1; '
               'exec /usr/bin/python "$@"', 'g2-native',
               str(ROOT/'deploy/ros/g2_map_patrol.py'), '--scene', str(ROOT/scene),
               '--point'] + [str(v) for v in config['point_map']]
    command += ['--face-point'] + [str(v) for v in config['box_center_map']]
    command += ['--inset', '.70', '--boundary-micro', '--seconds', '600', '--output', str(output)]
    if real:
        command.append('--real')
    return command


def run_navigation(command):
    process = subprocess.Popen(command)
    try:
        return process.wait(timeout=620)
    except BaseException:
        process.terminate()
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--expected-config-sha256')
    parser.add_argument('--scene', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--reentry-only', action='store_true', help='read-only post-release localization/mask check')
    args = parser.parse_args(argv)
    output = Path(args.output)
    if output.resolve() == Path(args.config).resolve():
        parser.error('output must not overwrite configuration')
    report = dict(ok=False, real=args.real, arm_or_release_executed=False,
                  next_stage='WAIT_HANDOFF_CALIBRATION')
    try:
        config = load_config(args.config, args.scene, args.expected_config_sha256)
        report['initial_resume'] = ready(config, expected_holding=not args.reentry_only)
        if args.reentry_only:
            pose = report['initial_resume']['pose']
            if not Policy(str(ROOT/args.scene)).allowed(*pose[:2]):
                raise ValueError('WAIT_EXIT_CORRIDOR: current centre still outside patrol mask')
            report.update(ok=True, next_stage='PATROL_REENTRY_READY', release_ready=False)
            guard.atomic(output, report)
            print(json.dumps(report))
            return 0
        nav_path = str(output) + '.navigation.json'
        command = navigation_command(args.scene, config, nav_path, args.real)
        code = run_navigation(command)
        nav = guard.read_json(nav_path)
        if code or nav.get('ok') is not True or nav.get('real') is not args.real:
            raise ValueError('map transit failed: ' + str(nav.get('error', code)))
        report['navigation_report'] = nav_path
        report['final_resume'] = ready(config)
        for field in ('boot_id', 'owner_pid', 'map_hash'):
            before, after = report['initial_resume'], report['final_resume']
            if field not in before or before[field] != after.get(field):
                raise ValueError('delivery context changed during transit: ' + field)
        report.update(ok=True, next_stage='KOALA_VISUAL_APPROACH' if args.real else 'READY_FOR_MAP_HANDOFF',
                      release_ready=False)
    except (Exception, KeyboardInterrupt) as exc:
        report.update(error=str(exc) or 'interrupted')
    guard.atomic(output, report)
    print(json.dumps(report))
    return 0 if report['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
