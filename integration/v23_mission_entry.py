"""PC entry: registered ArUco/LiDAR initialization, live video gate, koala mission.

Use a separate current koala checkout; this entry never replaces its arm or
approach files. SSH authentication comes from the operator's SSH configuration.
Default is the upstream read-only preflight. Physical actions require --real.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
JETSON = '/home/jetson/Documents/deploy_jetson2'
SCENE = 'maps/demo_box_20261018_patrol'
REGISTRY = SCENE + '/aruco_corner_landmarks.json'


def prepared_runtime(root):
    path = ROOT / 'integration/g2_prepare_koala.py'
    spec = importlib.util.spec_from_file_location('g2_runtime_preparer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.verify_manifest(root)


def remote(host, command, timeout=35, data=None, check=True):
    result = subprocess.run(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', 'jetson@' + host, command],
        input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    if check and result.returncode:
        raise RuntimeError(result.stderr.decode('utf-8', 'replace')[-1800:] or
                           result.stdout.decode('utf-8', 'replace')[-1800:])
    return result


def rpc(host, command, timeout=30):
    program = """import socket,sys
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(15)
s.connect('/tmp/grasp_health.sock' if sys.argv[1]=='health' else '/tmp/grasp_service.sock')
s.sendall((sys.argv[1]+'\\n').encode());print(s.makefile().readline());s.close()
"""
    return json.loads(remote(host, 'python3 -c ' + shlex.quote(program) + ' ' +
                             shlex.quote(command), timeout=timeout).stdout)


def require_parked(health):
    chassis = health.get('chassis', {})
    if (health.get('ok') is not True or health.get('holding') is not False or
            chassis.get('motors') != [0, 0, 0, 0] or chassis.get('moving') is not False or
            chassis.get('arm_pose') != 'travel' or chassis.get('arm_busy') is not False or
            chassis.get('client') is not None):
        raise RuntimeError('initialization requires a known empty, parked travel state')


def registry_bytes():
    raw = (ROOT / REGISTRY).read_bytes()
    registry = json.loads(raw)
    if (set(registry['markers']) != set(map(str, range(8))) or
            registry.get('registration_verified') is not True):
        raise RuntimeError('formal registered IDs 0..7 required')
    return raw, registry


def require_localized(result):
    if not (result.get('ok') is True and result.get('localized') is True and
            result.get('status') == 'LOCALIZED'):
        raise RuntimeError('corner initialization refused: ' + json.dumps(result))


def existing_reference(records, registry):
    """Keep healthy AMCL running; a recorded ack is valid only in this context."""
    previous = -1.0
    first = None
    for record in records:
        health, loc, seed = record['health'], record['localization'], record['reference']
        require_parked(health)
        state = loc.get('state', {})
        pose, cov = state.get('pose', []), state.get('cov', [])
        if (len(pose) != 3 or len(cov) != 3 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in pose + cov) or
                min(cov) < 0 or max(cov[:2]) > .15**2 or cov[2] > math.radians(10)**2 or
                loc.get('ready') is not True or not 0 <= record['now'] - loc['stamp'] < 1 or
                loc.get('boot_id') != record['boot_id'] or
                loc.get('owner_pid') != health['service_pid'] or
                record['owners'] != [str(health['service_pid'])] or
                seed.get('boot_id') != loc.get('boot_id') or
                seed.get('owner_pid') != loc.get('owner_pid') or
                seed.get('map_hash') != loc.get('map_hash') or
                loc.get('map_hash') != registry['map_hash'] or
                seed.get('registration_id') != registry['registration_id'] or
                seed.get('stamp') is None or loc.get('accepted_reference_stamp') != seed['stamp'] or
                state.get('odom_stamp', -1) <= previous):
            raise RuntimeError('existing AMCL reference is not fresh and context-bound')
        if first is not None and (math.hypot(pose[0] - first[0], pose[1] - first[1]) > .03 or
                abs((pose[2] - first[2] + math.pi) % (2 * math.pi) - math.pi) > math.radians(3)):
            raise RuntimeError('existing AMCL reference is not stable')
        first = first or pose
        previous = state['odom_stamp']
    if len(records) < 5:
        raise RuntimeError('five distinct healthy samples required')
    return {'ok': True, 'localized': True, 'status': 'LOCALIZED',
            'mode': 'existing_reference_validated', 'seed_written': False,
            'accepted_reference_stamp': records[-1]['localization']['accepted_reference_stamp']}


def download(host, source, target):
    data = remote(host, 'cat ' + shlex.quote(source)).stdout
    expected = remote(host, 'sha256sum ' + shlex.quote(source)).stdout.decode('ascii').split()[0]
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError('SSH download truncated or changed: ' + source)
    target.write_bytes(data)


def upload(host, target, data):
    program = 'import pathlib,sys;pathlib.Path(sys.argv[1]).write_bytes(sys.stdin.buffer.read())'
    remote(host, 'python3 -c ' + shlex.quote(program) + ' ' + shlex.quote(target), data=data)
    expected = remote(host, 'sha256sum ' + shlex.quote(target)).stdout.decode('ascii').split()[0]
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError('SSH upload truncated or changed: ' + target)


def initialize(args, folder):
    require_parked(rpc(args.host, 'health'))
    if rpc(args.host, 'maintenance-lock').get('ok') is not True:
        raise RuntimeError('initialization maintenance lock refused')
    raw, registry = registry_bytes()
    remote_registry = JETSON + '/' + REGISTRY
    old = remote(args.host, 'cat ' + shlex.quote(remote_registry)).stdout
    (folder / 'registry_before.json').write_bytes(old)
    if hashlib.sha256(old).digest() != hashlib.sha256(raw).digest():
        stage = remote_registry + '.' + folder.name + '.upload'
        upload(args.host, stage, raw)
        program = """import pathlib,shutil,os,sys,json
p=pathlib.Path(sys.argv[1]);stage=pathlib.Path(sys.argv[2]);r=json.loads(stage.read_text())
assert set(r['markers'])==set(map(str,range(8))) and r['registration_verified'] is True
backup=pathlib.Path(str(p)+'.'+sys.argv[3]+'.before');assert not backup.exists()
shutil.copy2(str(p),str(backup));os.replace(str(stage),str(p))
"""
        remote(args.host, 'python3 -c ' + shlex.quote(program) + ' ' +
               ' '.join(shlex.quote(v) for v in (remote_registry, stage, folder.name)))
    downloaded = remote(args.host, 'cat ' + shlex.quote(remote_registry)).stdout
    if downloaded != raw:
        raise RuntimeError('PC/Jetson registry bytes differ')
    (folder / 'registry.json').write_bytes(raw)
    if not args.relocalize:
        program = """import json,socket,pathlib,subprocess,time
rows=[]
for i in range(5):
 s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(5);s.connect('/tmp/grasp_health.sock');s.sendall(b'health\\n');h=json.loads(s.makefile().readline());s.close()
 l=json.load(open('/dev/shm/g2_localization_health.json'));ref=json.load(open('/dev/shm/g2_localization_seed.json'))
 rows.append(dict(health=h,localization=l,reference=ref,now=time.time(),boot_id=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),owners=subprocess.check_output(['fuser','/dev/myserial'],stderr=subprocess.DEVNULL).decode().split()))
 time.sleep(.12)
print(json.dumps(rows))
"""
        try:
            records = json.loads(remote(args.host, 'python3 -c ' + shlex.quote(program)).stdout)
            (folder / 'existing_reference_samples.json').write_text(json.dumps(records, indent=2))
            result = existing_reference(records, registry)
            (folder / 'localization_result.json').write_text(json.dumps(result, indent=2))
            print('[ENTRY] five fresh samples validated the existing AMCL reference; no reseed', flush=True)
            return result
        except (ValueError, RuntimeError, KeyError) as exc:
            (folder / 'existing_reference_rejection.txt').write_text(str(exc), encoding='utf-8')
    run = '/tmp/v23_mission/' + folder.name
    capture_dir = run + '/corner'
    command = ('cd ' + shlex.quote(JETSON) + ' && timeout 28s /home/jetson/grasp_venv/bin/python3' +
        ' -u deploy/ros/g2_corner_camera_capture.py --output-dir ' +
        shlex.quote(capture_dir) + ' --registration-id ' + shlex.quote(registry['registration_id']))
    capture = remote(args.host, command, timeout=35)
    (folder / 'capture_command.txt').write_bytes(capture.stdout + capture.stderr)
    for name in ('capture.json', 'observations.json', 'frame_00.jpg', 'frame_01.jpg', 'frame_02.jpg'):
        download(args.host, capture_dir + '/' + name, folder / name)
    download(args.host, JETSON + '/maps/demo_box_20261017/demo_box.yaml', folder / 'active_map.yaml')
    match = subprocess.run(
        [sys.executable, str(ROOT / 'integration/g2_scan_match.py'), '--snapshot',
         str(folder / 'capture.json'), '--scene', str(ROOT / SCENE), '--reference-map-yaml',
         str(folder / 'active_map.yaml'), '--output', str(folder / 'hypotheses.json')],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    (folder / 'scan_match.txt').write_bytes(match.stdout + match.stderr)
    if match.returncode:
        raise RuntimeError('scan matcher failed; see scan_match.txt')
    upload(args.host, run + '/hypotheses.json', (folder / 'hypotheses.json').read_bytes())
    apply = remote(args.host, 'cd ' + shlex.quote(JETSON) +
                   ' && timeout 30s /home/jetson/grasp_venv/bin/python3 '
                   'integration/g2_aruco_corner_seed.py --snapshot /dev/shm/g2_corner_capture.json '
                   '--candidates ' + shlex.quote(run + '/hypotheses.json') +
                   ' --observations ' + shlex.quote(capture_dir + '/observations.json') +
                   ' --landmarks ' + shlex.quote(REGISTRY) +
                   ' --output ' + shlex.quote(run + '/result.json') + ' --real',
                   timeout=38, check=False)
    (folder / 'apply_command.txt').write_bytes(apply.stdout + apply.stderr)
    download(args.host, run + '/result.json', folder / 'localization_result.json')
    result = json.loads((folder / 'localization_result.json').read_bytes())
    require_localized(result)
    print('[ENTRY] new ArUco/LiDAR reference acknowledged by AMCL', flush=True)
    return result


def load_koala(root):
    root = root.resolve()
    # Relative imports must all come from the same coherent koala checkout.
    existing = sys.modules.get('integration')
    if existing is not None and not any(Path(p).resolve() == root / 'integration'
                                        for p in getattr(existing, '__path__', [])):
        raise RuntimeError('launch this entry as a script; another integration package is loaded')
    sys.path.insert(0, str(root))
    spec = importlib.util.spec_from_file_location('v23_upstream_mission', root / 'integration/mission_loop.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_recovery_deployment(host):
    names = ('deploy/ros/g2_map_patrol.py', 'integration/g2_task_recovery.py')
    expected = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names}
    program = ("import hashlib,json,os,sys\nroot=sys.argv[1]\n"
        "print(json.dumps({name:hashlib.sha256(open(os.path.join(root,name),'rb').read()).hexdigest() "
        "for name in json.loads(sys.argv[2])}))\n")
    result = remote(host, 'python3 -c '+shlex.quote(program)+' '+shlex.quote(JETSON)+' '+shlex.quote(json.dumps(names)))
    actual = json.loads(result.stdout)
    if actual != expected:
        raise RuntimeError('paired Jetson recovery code/hash mismatch; keep the robot parked')
    return expected


def check_stream(args, folder, mission):
    from integration.rear_stream import StreamReader
    reader = StreamReader(5600, args.gstreamer)
    owned_pid = None
    try:
        pc_ip = mission.pc_address(args.host)
        if not mission.stream_running(args.host, pc_ip):
            program = """import subprocess,sys
log=open('/tmp/v23_entry_rear_stream.log','ab')
p=subprocess.Popen(['bash',sys.argv[1],sys.argv[2],'5600'],stdin=subprocess.DEVNULL,
 stdout=log,stderr=log,start_new_session=True)
print(p.pid)
"""
            command = ('python3 -c ' + shlex.quote(program) + ' ' +
                       shlex.quote(JETSON + '/deploy/ros/start_rear_approach_stream.sh') +
                       ' ' + shlex.quote(pc_ip))
            owned_pid = int(remote(args.host, command).stdout.strip())
            (folder / 'owned_sender_pid.json').write_text(json.dumps(owned_pid))
        start = time.monotonic()
        previous, samples = 0.0, []
        while time.monotonic() - start < 12:
            frame, stamp = reader.latest()
            if frame is not None and stamp > previous and time.monotonic() - stamp < .5:
                if frame.shape != (480, 640, 3):
                    raise RuntimeError('unexpected rear camera frame shape')
                samples.append(stamp)
                previous = stamp
                if len(samples) >= 8:
                    (folder / 'stream_gate.json').write_text(json.dumps(
                        {'new_frames': len(samples), 'span_s': samples[-1] - samples[0]}))
                    print('[ENTRY] eight fresh rear frames received; video gate passed', flush=True)
                    return owned_pid
            if reader.proc.poll() is not None:
                raise RuntimeError('rear decoder exited before video gate passed')
            time.sleep(.04)
        raise RuntimeError('no continuous fresh rear frames; mission remains parked')
    finally:
        reader.close()


def run_stages(initialize_stage, stream_stage, mission_stage):
    require_localized(initialize_stage())
    stream_stage()
    return mission_stage()


def install_patrol_reentry(mission, folder, with_placement=True):
    """Load our adapter by path so the upstream integration package stays coherent."""
    path = ROOT / 'integration/g2_patrol_reentry.py'
    spec = importlib.util.spec_from_file_location('v23_post_place_reentry', path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    recovery_path = ROOT / 'integration/g2_mission_recovery.py'
    recovery_spec = importlib.util.spec_from_file_location('v23_task_recovery', recovery_path)
    recovery = importlib.util.module_from_spec(recovery_spec)
    recovery_spec.loader.exec_module(recovery)
    placement = None
    placement_path = ROOT / 'integration/g2_placement_supervisor.py'
    if with_placement:
        placement_spec = importlib.util.spec_from_file_location('v23_placement_supervisor', placement_path)
        placement = importlib.util.module_from_spec(placement_spec)
        placement_spec.loader.exec_module(placement)
    def health(host, timeout=None):
        return rpc(host, 'health') if timeout is None else rpc(host, 'health', timeout=timeout)
    result = adapter.bind_mission(mission, SCENE, folder / 'mission',
                                  health, recovery=recovery, placement=placement)
    result['source_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    result['recovery_source_sha256'] = hashlib.sha256(recovery_path.read_bytes()).hexdigest()
    if with_placement:
        result['placement_source_sha256'] = hashlib.sha256(placement_path.read_bytes()).hexdigest()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--koala-root', type=Path, default=ROOT / '.runtime/koala-e8766f0-recovery',
                        help='runtime prepared by integration/g2_prepare_koala.py')
    parser.add_argument('--gstreamer', type=Path, default=os.environ.get('GSTREAMER_LAUNCH_EXE'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--max-deliveries', type=int)
    parser.add_argument('--relocalize', action='store_true',
                        help='require a new stopped camera/LiDAR reference even if AMCL is healthy')
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--i-am-beside-the-robot', action='store_true')
    args = parser.parse_args(argv)
    ipaddress.IPv4Address(args.host)
    if args.real and not args.i_am_beside_the_robot:
        parser.error('--real requires --i-am-beside-the-robot')
    if args.gstreamer is None or not args.gstreamer.is_file():
        parser.error('a working GStreamer executable is required')
    if args.max_deliveries is not None and args.max_deliveries < 1:
        parser.error('--max-deliveries must be positive')
    manifest = prepared_runtime(args.koala_root)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    folder = args.output or ROOT / 'docs/calibration' / ('mission_' + stamp)
    folder.mkdir(parents=True, exist_ok=False)
    report = {'ok': False, 'physical_mission_started': False,
              'koala_root': str(args.koala_root.resolve()), 'stage': 'PREFLIGHT'}
    report['runtime_manifest'] = manifest
    report['koala_source_sha256'] = {
        name: hashlib.sha256((args.koala_root / 'integration' / name).read_bytes()).hexdigest()
        for name in ('mission_loop.py', 'mission_runtime.py', 'robot_rpc.py',
                     'sugarbox_approach_grasp.py', 'sugarbox_rl_approach_final2.py')}
    mission = load_koala(args.koala_root)
    upstream = ['--host', args.host, '--gstreamer', str(args.gstreamer),
                '--log-dir', str(folder / 'mission')]
    if args.max_deliveries is not None:
        upstream += ['--max-deliveries', str(args.max_deliveries)]
    try:
        report['post_place_exit'] = install_patrol_reentry(mission, folder)
        if not args.real:
            code = mission.main(upstream)
        else:
            report['stage'] = 'RECOVERY_CONTRACT'
            report['jetson_recovery_sha256'] = verify_recovery_deployment(args.host)
            def localization():
                report['stage'] = 'LOCALIZATION'
                return initialize(args, folder)
            def video():
                report['stage'] = 'VIDEO_GATE'
                check_stream(args, folder, mission)
            def execute():
                report.update(stage='MISSION', physical_mission_started=True)
                return mission.main(upstream + ['--real', '--i-am-beside-the-robot'])
            code = run_stages(localization, video, execute)
        report.update(ok=code == 0, exit_code=code)
        return code
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        print('[ENTRY] stopped in {}: {}'.format(report['stage'], exc), flush=True)
        return 2
    finally:
        if args.real:
            try:
                mission.runner.stop(args.host)
                report['final_health'] = rpc(args.host, 'health')
                if report['final_health'].get('chassis', {}).get('arm_pose') == 'travel':
                    report['maintenance_lock'] = rpc(args.host, 'maintenance-lock')
            except Exception as exc:
                report['cleanup_error'] = str(exc)
            pid_file = folder / 'owned_sender_pid.json'
            if pid_file.exists():
                pid = int(json.loads(pid_file.read_text()))
                cleanup = """import pathlib,sys,os,signal
pid=int(sys.argv[1]);p=pathlib.Path('/proc/%d/cmdline'%pid)
if p.exists():
 command=p.read_bytes()
 if b'start_rear_approach_stream.sh' in command or b'gst-launch-1.0' in command:os.kill(pid,signal.SIGINT)
"""
                try:
                    remote(args.host, 'python3 -c ' + shlex.quote(cleanup) + ' ' + str(pid))
                except Exception as exc:
                    report['sender_cleanup_error'] = str(exc)
        (folder / 'entry_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    raise SystemExit(main())
