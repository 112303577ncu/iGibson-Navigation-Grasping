"""PC-side placement reservation; the existing native/vision clients own motion."""
import importlib.util
import inspect
import json
import math
from pathlib import Path
import shlex
import subprocess
import tempfile
import threading
import time
import uuid
from integration import g2_task_recovery as task

spec = importlib.util.spec_from_file_location('g2_placement_geometry', Path(__file__).with_name('g2_placement_corridor.py'))
corridor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(corridor)


def atomic(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')
    for attempt in range(5):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 4:
                raise
            # Windows readers can briefly hold a file without delete sharing.
            # Bound retries to 8ms; never keep refreshing an old sample's time.
            time.sleep(.002)


class LocalizationFeed:
    """One read-only SSH stream, not a new connection for every control sample."""
    def __init__(self, host, path, context):
        self.path, self.context = Path(path), list(context)
        self.closed = False
        atomic(self.path, dict(ready=False))
        program = """import json,time,sys
print('{"hello":true}',flush=True)
if not sys.stdin.readline():sys.exit(0)
while True:
 try:
  with open('/dev/shm/g2_localization_health.json') as f:r=json.load(f)
  print(json.dumps({'record':r,'now':time.time()}),flush=True)
 except Exception:print('{}',flush=True)
 time.sleep(.05)
"""
        self.proc = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            'jetson@' + host, 'python3 -u -c ' + shlex.quote(program)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, encoding='utf-8')
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        sent, anchor = None, None
        history = []
        try:
            for line in self.proc.stdout:
                if getattr(self, 'closed', False):
                    break
                try:
                    message = json.loads(line)
                    if message.get('hello') is True and sent is None:
                        sent = time.monotonic()
                        self.proc.stdin.write('capture\n')
                        self.proc.stdin.flush()
                        continue
                    loc, now = message['record'], message['now']
                    if sent is None:
                        raise ValueError('localization stream did not acknowledge capture')
                    received = time.monotonic()
                    anchor = now if anchor is None else anchor
                    state = loc.get('state') or {}
                    # First sample was captured AFTER our request. Keep this
                    # conservative clock binding for the entire stage, so queued
                    # old SSH data cannot become fresh merely by arriving late.
                    age = received - sent - (loc['stamp'] - anchor)
                    cov, pose = state['cov'], state['pose']
                    context = [loc['boot_id'], loc['owner_pid'], loc['map_hash']]
                    ready = (loc.get('ready') is True and context == self.context and 0 <= age <= .4
                             and 0 <= now - loc['stamp'] <= .4
                             and len(pose) == len(cov) == 3
                             and all(type(v) in (float, int) and math.isfinite(v) for v in pose + cov)
                             and min(cov) >= 0 and max(cov[:2]) <= (.04 / 3) ** 2
                             and all(type(state.get(key)) in (int, float) and math.isfinite(state[key])
                                     for key in ('vx', 'wz')))
                    if (ready and isinstance(state.get('odom'), list) and len(state['odom']) == 3
                            and task.finite(state['odom']+[state.get('odom_stamp')])):
                        if not history or history[-1]['stamp'] != state['odom_stamp']:
                            history.append(dict(at=received-age, pose=pose, odom=state['odom'],
                                                stamp=state['odom_stamp']))
                            history = history[-40:]
                    reason = ('READY' if ready else 'AMCL_UNCERTAIN' if loc.get('ready') is True
                              else 'REFERENCE_REQUIRED' if loc.get('status') == 'needs_reference'
                              else loc.get('reason_code') or 'LOCALIZATION_NOT_READY')
                    atomic(self.path, dict(ready=ready, received_at=received,
                        source_age_s=age, pose=pose, context=context,
                        measured_vx=state.get('vx'), measured_wz=state.get('wz'),
                        odom=state.get('odom'), stamp=state.get('odom_stamp'),
                        reason_code=reason,
                        wait_elapsed_s=loc.get('wait_elapsed_s', 0.),
                        pose_history=history,
                        fatal=not ready and reason not in task.TRANSIENT))
                except (OSError, ValueError, KeyError, TypeError, OverflowError):
                    atomic(self.path, dict(ready=False, reason_code='SUPERVISOR_STALE'))
        finally:
            atomic(self.path, dict(ready=False))

    def close(self):
        self.closed = True
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=2)
        self.thread.join(timeout=2)
        atomic(self.path, dict(ready=False))


class Supervisor:
    def __init__(self, mission, rooms, scene, folder):
        self.mission, self.rooms, self.scene, self.folder = mission, rooms, scene, Path(folder)
        self.active = None
        self.events = []
        runner = mission.runner
        for name in ('start_engine', 'place', 'drive', 'ctl'):
            if not hasattr(runner, name):
                raise RuntimeError('placement reservation requires coherent resident runner: ' + name)
        self.original_place, self.original_start = runner.place, runner.start_engine
        runner.start_engine, runner.place = self.start_engine, self.place

    def save(self, status, **fields):
        self.events.append(dict(status=status, **fields))
        self.folder.mkdir(parents=True, exist_ok=True)
        atomic(self.folder / 'placement_corridor.json', self.events)

    def start_engine(self, env):
        if env.get('SUGARBOX_TARGET') == 'placement':
            if self.active is None:
                raise RuntimeError('placement engine has no reserved return corridor')
            env = dict(env, G2_PLACEMENT_GUARD_FILE=str(self.active['file']))
        return self.original_start(env)

    def place(self, *args, **kwargs):
        """Intercept the actual release RPC, after any taught-wall correction."""
        if self.active is None:
            raise RuntimeError('release has no placement reservation')
        active = self.active
        if active.get('box', {}).get('verified') is False:
            self.observe_release(args, kwargs)
        read = kwargs.pop('read', self.mission.runner.ctl)
        drive = kwargs.pop('drive', self.mission.runner.drive)
        def checked_read(host, command):
            if command in ('release', 'release-preview'):
                actual = active['room']._snapshot(holding=True)
                expected = active.get('observed_release', active['plan']['release'])
                if math.hypot(actual['pose'][0] - expected[0], actual['pose'][1] - expected[1]) > .10:
                    raise RuntimeError('RELEASE_POSE_REFUSED: actual stop differs from reserved release point')
                problem = active['geometry'].command_problem(active['plan'], actual['pose'], 0., 0.)
                if problem:
                    raise RuntimeError('RELEASE_EXIT_REFUSED: ' + problem)
                self.save('RELEASE_EXIT_CONFIRMED', pose=actual['pose'],
                          exit=active['geometry'].exit(actual['pose']))
            return read(host, command)
        def checked_drive(host, metres, **options):
            actual = active['room']._snapshot(holding=True)
            for k in range(11):
                distance = metres * k / 10
                pose = actual['pose']
                point = [pose[0] + distance * math.cos(pose[2]),
                         pose[1] + distance * math.sin(pose[2]), pose[2]]
                if active['geometry'].command_problem(active['plan'], point,
                        math.copysign(.08, metres), 0.):
                    raise RuntimeError('wall correction would lose placement exit')
            return drive(host, metres, **options)
        return self.original_place(*args, **dict(kwargs, read=checked_read, drive=checked_drive))

    def observe_release(self, args, kwargs):
        """Use this run's vision/odometry arrival, never promote a map candidate.

        The native report measures the box EDGE in base coordinates. Capture
        its map position before the native taught-wall correction moves again.
        Keep the old conservative obstacle too; an observation cannot erase it.
        """
        values = inspect.signature(self.original_place).bind(*args, **kwargs).arguments
        report = values.get('report')
        final_x, aim_y = values.get('final_x'), values.get('aim_y')
        problem = self.mission.dock.arrival_problem(report, final_x, aim_y)
        if problem:
            raise RuntimeError('RELEASE_OBSERVATION_REFUSED: ' + problem)
        numbers = [report.get(k) for k in ('t', 'visual_age_s')]
        if (any(type(v) not in (int, float) or not math.isfinite(v) for v in numbers)
                or not 0 <= time.time() - numbers[0] <= 5.
                or not 0 <= numbers[1] <= 12.
                or report.get('target_kind') != 'placement' or report.get('blocked') is not False):
            raise RuntimeError('RELEASE_OBSERVATION_REFUSED: stale or invalid placement arrival')
        active = self.active
        snapshot = active['room']._snapshot(holding=True)
        if snapshot['context'] != active['context']:
            raise RuntimeError('RELEASE_OBSERVATION_REFUSED: localization context changed')
        pose = snapshot['pose']
        depth = corridor.half_depth(active['box'], pose[2], self.mission.dock.BOX_DEPTH_M / 2)
        point = self.mission.dock.to_map(pose, (report['x'] + depth, report['y']))
        if math.hypot(point[0] - active['box']['x'], point[1] - active['box']['y']) > .25:
            raise RuntimeError('RELEASE_OBSERVATION_REFUSED: observed bin exceeds candidate uncertainty')
        observed = dict(active['box'], x=point[0], y=point[1], verified=False)
        obstacle = (point[0], point[1], self.mission.dock.BIN_HALF_DIAGONAL_M + .04)
        room = active['room']
        geometry = corridor.Geometry(room.mask, room.walls, list(room.obstacles) + [obstacle], self.mission.dock)
        if geometry.exit(pose) is None:
            raise RuntimeError('RELEASE_EXIT_REFUSED: observed bin has no straight return')
        expected = self.mission.dock.to_map(pose, (report['x'] - final_x, report['y'] - aim_y))
        active['observed_release'] = [expected[0], expected[1], pose[2]]
        active['geometry'] = geometry
        room.obstacles = geometry.obstacles
        self.save('BIN_OBSERVATION_CHECKED', bin=observed, pose=pose,
                  registration_verified=False, exit=geometry.exit(pose))

    def transit(self, host, plan, real):
        coordinator = getattr(self.mission, '_g2_task_coordinator', None)
        if real and coordinator is not None:
            output = '/dev/shm/g2_placement_transit_' + uuid.uuid4().hex + '.json'
            return coordinator.transport(host, plan,
                lambda: self._transit(host, plan, real, self.mission.PERMIT_PATH, output), output)
        return self._transit(host, plan, real)

    def _transit(self, host, plan, real, vision_path=None, output=None):
        output = output or '/dev/shm/g2_placement_transit_' + uuid.uuid4().hex + '.json'
        argv = ['/usr/bin/python', 'deploy/ros/g2_map_patrol.py', '--scene', self.scene,
                '--point'] + [str(v) for v in plan['staging'][:2]]
        argv += ['--face-yaw', str(plan['yaw_rad'])]
        argv += ['--inset', '.70', '--boundary-micro', '--seconds', '180', '--output', output]
        if vision_path is not None:
            argv += ['--vision-permit', str(vision_path)]
        if real:
            argv.append('--real')
        command = ('cd /home/jetson/Documents/deploy_jetson2 && '
                   'source /opt/ros/melodic/setup.bash && '
                   'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 && '
                   'timeout -s TERM {}s '.format(195 if real else 20) + shlex.join(argv))
        result = self.mission.ssh(host, '/bin/bash -c ' + shlex.quote(command), timeout=210 if real else 35)
        report = json.loads(self.mission.ssh(host, 'cat ' + shlex.quote(output)).stdout)
        if result.returncode or report.get('ok') is not True or report.get('real') is not real:
            raise RuntimeError('placement staging transit refused: ' + str(report.get('error')))
        return report

    def execute(self, args, layout_copy, log_dir, n, new_reader, new_watcher, came_from=None):
        # A subprocess runner would bypass these in-process release guards.
        if getattr(self.mission, '_runtime', None) is None:
            raise RuntimeError('reserved placement requires the resident mission runtime')
        room = self.rooms.get(args.host)
        if room is None:
            raise RuntimeError('placement room checkpoint missing')
        layout = json.loads(Path(layout_copy).read_text(encoding='utf-8'))
        box = self.mission.dock.box_from_layout(layout, str(layout_copy))
        record = layout.get('bin') or {}
        pose = (record.get('pose_map') if record.get('pose_map_valid_for_current_placement') is True
                else (record.get('placement_update') or {}).get('midpoint_pose_map_candidate')) or {}
        dimensions = record.get('dimensions_m') or {}
        if all(v is not None for v in (pose.get('yaw'), dimensions.get('length'), dimensions.get('width'))):
            box.update(orientation_rad=pose['yaw'], length_m=dimensions['length'], width_m=dimensions['width'])
        self.box_point = [box['x'], box['y']]
        bad_headings = []
        try:
            for attempt in range(2):
                # A previous visual arrival may have added a conservative bin
                # envelope. Carry it into planning as well as the worker gate.
                geometry = corridor.Geometry(room.mask, room.walls, room.obstacles, self.mission.dock)
                before = room._snapshot(holding=True)
                plans = geometry.choose(box, before['pose'], self.mission.dock.PLACEMENT_FINAL_X_M,
                                        self.mission.dock.PLACEMENT_AIM_Y_M, bad_headings)
                selected = None
                planning_deadline = time.monotonic() + 30.
                for plan in plans:
                    if time.monotonic() > planning_deadline:
                        break
                    room._snapshot(holding=True)
                    try:
                        self.transit(args.host, plan, False)
                    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
                        self.save('STAGING_UNREACHABLE', error=str(exc), plan=plan)
                        continue
                    selected = plan
                    break
                if selected is None:
                    raise RuntimeError('NO_REACHABLE_DOCK: no reserved staging route')
                self.save('CORRIDOR_RESERVED', attempt=attempt + 1, plan=selected,
                          box_registration_verified=box.get('verified') is True)
                self.transit(args.host, selected, True)
                arrived = room._snapshot(holding=True)
                if (arrived['context'] != before['context'] or not corridor.turn_ready(room.mask, arrived['pose'])
                        or abs(corridor.wrap(arrived['pose'][2] - selected['yaw_rad'])) > math.radians(8)
                        or math.hypot(arrived['pose'][0] - selected['staging'][0],
                                      arrived['pose'][1] - selected['staging'][1]) > .10):
                    raise RuntimeError('placement staging acknowledgement failed')
                room.reentry_used = 0.
                self.mission.unlock(args.host)
                token = uuid.uuid4().hex
                temporary = tempfile.TemporaryDirectory(prefix='g2-placement-')
                contract_path = Path(temporary.name) / 'contract.json'
                state_path = Path(temporary.name) / 'state.json'
                contract = dict(schema=1, plan=selected, context=list(arrived['context']),
                    mask=corridor.encode_mask(room.mask), walls=room.walls, obstacles=room.obstacles,
                    state_file=str(state_path.resolve()))
                atomic(contract_path, contract)
                atomic(self.folder / ('placement_contract_' + token + '.json'), contract)
                feed = LocalizationFeed(args.host, state_path, arrived['context'])
                self.active = dict(file=contract_path.resolve(), room=room, geometry=geometry,
                                   plan=selected, box=box, context=arrived['context'])
                failure = None
                try:
                    argv = ['--target', 'placement', '--real', '--i-am-beside-the-robot']
                    argv += self.mission.stage_options(args)
                    if args.wall_m is not None:
                        argv += ['--wall-m', str(args.wall_m)]
                    argv += ['--obs-log', str(Path(log_dir) / ('place_{}_{}_obs.jsonl'.format(n, attempt)))]
                    code = self.mission.run_stage(args.host, argv,
                        Path(log_dir) / 'place_{}_{}.log'.format(n, attempt))
                    state = self.mission.arm_state(args.host)
                    if code == 0 and state == 'empty':
                        self.save('PLACED', attempt=attempt + 1)
                        return
                    failure = 'placement exit {}; arm {}'.format(code, state)
                except (Exception, KeyboardInterrupt) as exc:
                    failure = str(exc) or 'interrupted'
                    if isinstance(exc, KeyboardInterrupt):
                        raise
                    if isinstance(exc, task.Fault) and not exc.retryable:
                        raise
                finally:
                    try:
                        feed.close()
                    finally:
                        self.active = None
                        self.mission.runner.stop(args.host)
                        temporary.cleanup()
                # This is an approach retry only. Once release is possible or
                # ambiguous, never redispatch an arm action.
                if self.mission.arm_state(args.host) != 'holding':
                    raise RuntimeError('ACTION_OUTCOME_UNKNOWN: ' + failure)
                self.mission.to_travel(args.host)
                room._leave_ready(0., holding=True)
                self.save('HELD_OBJECT_RECOVERED_TO_PATROL', error=failure)
                bad_headings.append(selected['yaw_rad'])
                runtime = self.mission._runtime
                if attempt or (hasattr(runtime, 'usable') and not runtime.usable()):
                    raise RuntimeError('placement stopped at a turn-clear checkpoint: ' + failure)
        finally:
            self.mission.runner.stop(args.host)
