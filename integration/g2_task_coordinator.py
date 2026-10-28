"""Bind recovery to the actual resident mission, without another motion owner."""
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import os
import hashlib
import shlex
import uuid

from integration import g2_task_recovery as task


def footprint(mission, watcher):
    pixels = []
    for value in watcher.hull:
        while len(value) == 1:
            value = value[0]
        pixels.append((float(value[0]), float(value[1])))
    pixels = task.hull(pixels)
    x1, y1, x2, y2 = mission.ARM_CAMERA_REGION
    pieces = task.subtract(pixels, [(x1,y1),(x2,y1),(x2,y2),(x1,y2)])
    views = []
    for piece in pieces:
        points = []
        for u, v in piece:
            xy = mission.pixel_to_ground(u, v, watcher.H)
            if xy is None:
                raise task.Fault('OBSERVATION_GEOMETRY_MISSING', 'calibrated view cannot be projected')
            points.append((xy[0]-watcher.rear_cam_to_front+mission.GOAL_ORIGIN_OFFSET_X_M, xy[1]))
        if task.area(task.hull(points)) > 1e-6:
            views.append(task.hull(points))
    if not views:
        raise task.Fault('OBSERVATION_GEOMETRY_MISSING', 'arm-camera mask consumes calibrated view')
    return views


def verified_frame_bound(record, runtime_root):
    """External measurement evidence only; never edits camera calibration."""
    if record is None:
        return False
    record = Path(record)
    data = json.loads(record.read_text(encoding='utf-8'))
    if (data.get('schema') != 1 or data.get('verified') is not True
            or type(data.get('upper_bound_s')) not in (int, float)
            or not 0 < data['upper_bound_s'] <= .30):
        raise task.Fault('OBSERVATION_PROVENANCE_REQUIRED', 'unapproved source-to-decoder latency evidence')
    for field, relative in [('sender_sha256','deploy/ros/start_rear_approach_stream.sh'),
                            ('receiver_sha256','integration/rear_stream.py')]:
        actual = hashlib.sha256((Path(runtime_root)/relative).read_bytes()).hexdigest()
        if data.get(field) != actual:
            raise task.Fault('OBSERVATION_PROVENANCE_REQUIRED', 'latency evidence describes a different pipeline')
    proof = (record.parent/data['proof_file']).resolve()
    if data.get('proof_sha256') != hashlib.sha256(proof.read_bytes()).hexdigest():
        raise task.Fault('OBSERVATION_PROVENANCE_REQUIRED', 'measurement evidence missing or changed')
    return True


def suspended_goals(report):
    checkpoint = report.get('task_checkpoint') or {}
    remaining = report.get('remaining_route') or []
    path = [] if checkpoint.get('navigation_complete') else checkpoint.get('remaining_path') or []
    points = [list(p) for p in path]
    for goal in remaining:
        point = list(goal['point'])
        if not points or points[-1] != point:
            points.append(point)
    if len(points) > 12 or any(len(p) != 2 or not task.finite(p) for p in points):
        raise task.Fault('PATROL_RESUME_REFUSED', 'saved route is outside the existing bounded goals contract')
    return [dict(name='resume_'+str(i), point=p) for i,p in enumerate(points)]


def wait_feed_initialization(path, allowance, clock=time.monotonic, sleep=time.sleep):
    """Do not mistake a new stream's empty bootstrap file for a motion outage.

    This precedes worker/motor creation. A structured NOT_READY record is passed
    through to the normal recovery gate; it is never promoted to ready here.
    """
    end = clock()+max(0., allowance)
    while clock() < end:
        try:
            state = json.loads(Path(path).read_text(encoding='utf-8'))
            if state.get('context') is not None:
                return
        except (OSError, ValueError):
            pass
        sleep(min(.04, max(0., end-clock())))
    raise task.Fault('LOCALIZATION_FEED_START_TIMEOUT', 'no structured state before worker start')


class Coordinator:
    def __init__(self, mission, rooms, folder, feed_class):
        self.mission, self.rooms, self.folder, self.feed_class = mission, rooms, Path(folder), feed_class
        self.patrols, self.stage = {}, None
        self.frame_bound_verified = verified_frame_bound(os.environ.get('G2_RECOVERY_FRAME_BOUND_RECORD'),
                                                        getattr(mission, 'ROOT', '.'))
        self.original_stage, self.original_start = mission.run_stage, mission.runner.start_engine
        mission.run_stage, mission.runner.start_engine = self.run_stage, self.start_engine
        self.original_patrol = getattr(mission, 'start_patrol', None)
        if self.original_patrol is not None:
            mission.start_patrol = self.start_patrol
        mission._g2_task_coordinator = self

    def emit(self, value):
        print('[RECOVERY] ' + json.dumps(value), flush=True)

    def prime_motion(self, room):
        if getattr(room, '_observation', None) is not None:
            return
        runtime = getattr(self.mission, '_runtime', None)
        if runtime is None:
            raise task.Fault('OBSERVATION_SOURCE_MISSING', 'resident camera is required')
        watcher = self.mission.SugarboxWatcher(self.mission._g2_homography)
        room._observation = task.ObservationLedger(footprint(self.mission, watcher))
        room._observation_watcher = watcher
        self.observe_motion(room, room._last_snapshot, interrupted=False)

    def observe_motion(self, room, stopped, interrupted=True):
        """A stopped, newly captured view, with no invented repositioning."""
        runtime = self.mission._runtime
        watcher = room._observation_watcher
        ledger = room._observation
        if interrupted and not self.frame_bound_verified:
            room.checkpoint.save(status='OBSERVATION_PROVENANCE_REQUIRED', observation_complete=False)
            raise task.Fault('OBSERVATION_PROVENANCE_REQUIRED', 'source capture freshness is not validated')
        requested = time.monotonic()
        if interrupted and ledger.last is not None:
            ledger.interrupt(stopped['pose'], requested)
        sequence = 0 if ledger.last is None else ledger.last['sequence']
        lease = runtime.reader()
        try:
            stable, last = 0, None
            while True:
                if not interrupted and time.monotonic()-requested > 8.:
                    raise task.Fault('OBSERVATION_SOURCE_MISSING', 'no fresh stopped camera confirmation')
                room.episode.check()
                if interrupted and room.episode.started is None:
                    room.episode.pause('LOCALIZATION_NOT_READY')
                frame, stamp = lease.latest()
                if (frame is None or stamp-.30 <= requested or stamp == last
                        or not 0 <= time.monotonic()-stamp <= 1.5):
                    time.sleep(.04)
                    continue
                value = self.mission._g2_recovery.wait_snapshot(
                    lambda: room._snapshot_once(holding=room._holding),
                    lambda: self.mission.runner.stop(room.host), previous=stopped,
                    anchor=stopped, episode=room.episode)
                task.continuity(stopped, value)
                sighting = watcher.look(frame)
                sequence += 1
                ledger.observe(sequence, stamp-.30, value['pose'], .04)
                room.checkpoint.save(last_frame=ledger.last, stopped=value,
                    observed_target=sighting, status='WAIT_FRESH_OBSERVATION')
                last, stable = stamp, stable+1
                if stable >= 3:
                    ledger.require_complete()
                    if interrupted:
                        room.episode.resume()
                    room.checkpoint.save(observation_complete=True, status='REVALIDATE_PATH')
                    return
        finally:
            lease.close()

    def run_stage(self, host, argv, log):
        kind = argv[argv.index('--target')+1]
        room = self.rooms[host]
        before = room._snapshot(holding=kind == 'placement')
        if room.episode.pending and '--seed' in argv:
            argv = list(argv)
            at = argv.index('--seed')
            del argv[at:at+3]  # reacquire visually, do not reuse pre-outage coordinates
        room.checkpoint.save(stage='APPROACH_'+kind.upper(), target=kind,
            holding=kind == 'placement', arm_outcome='not_started', start_odom=before['odom'])
        room._anchor = None
        with tempfile.TemporaryDirectory(prefix='g2-stage-recovery-') as folder:
            self.stage = dict(room=room, folder=Path(folder), before=before, feed=None, kind=kind)
            try:
                code = self.original_stage(host, argv, log)
                state = self.mission.arm_state(host)
                expected = 'empty' if kind == 'placement' else 'holding'
                outcome = ('released' if kind == 'placement' else 'grasped') if code == 0 and state == expected else 'not_confirmed'
                if state == 'unknown':
                    outcome = 'unknown'
                room.checkpoint.save(arm_outcome=outcome, holding={'holding': True, 'empty': False}.get(state))
                if outcome in ('released', 'grasped'):
                    room.checkpoint.save(last_confirmed_arm_outcome=outcome)
                if state == 'unknown':
                    raise task.Fault('ACTION_OUTCOME_UNKNOWN', 'arm result requires operator confirmation; never replay')
                if code == 0 and state != expected:
                    raise task.Fault('ACTION_STATE_MISMATCH', 'success exit disagrees with confirmed holding state; never replay')
                return code
            except task.Fault:
                raise
            except (Exception, KeyboardInterrupt) as exc:
                room.checkpoint.save(arm_outcome='unknown', status='ACTION_OUTCOME_UNKNOWN')
                if isinstance(exc, KeyboardInterrupt):
                    raise
                raise task.Fault('ACTION_OUTCOME_UNKNOWN', 'stage ended without a confirmed result; never replay: '+str(exc)) from exc
            finally:
                anchor = None
                if self.stage['feed'] is not None:
                    self.stage['feed'].close()
                path = self.stage['folder'] / 'checkpoint.json'
                if path.exists():
                    data = json.loads(path.read_text())
                    anchor = data.get('last_valid_state') or data.get('stopped')
                    room.episode.used = max(room.episode.used, data.get('wait_used_s', 0.))
                    room.checkpoint.save(worker_recovery=data, last_frame=data.get('last_frame'))
                room._anchor = anchor  # compare an outage with the worker's latest measured motion
                room._observation = None
                self.stage = None

    def motion_anchor(self, room, fallback):
        if self.stage is None or self.stage['room'] is not room:
            return fallback
        path = self.stage['folder']/'checkpoint.json'
        if not path.exists():
            return fallback
        record = json.loads(path.read_text(encoding='utf-8'))
        value = record.get('last_valid_state') or record.get('stopped')
        if value is not None and (fallback is None or value['stamp'] > fallback['stamp']):
            return value
        return fallback

    def start_engine(self, env):
        if self.stage is None:
            return self.original_start(env)
        stage = self.stage
        context = stage['before']['context']
        # Reuse placement's existing read-only stream rather than adding a
        # second SSH connection. Sugarbox approach gets one stream per stage.
        active = getattr(getattr(self.mission, '_g2_placement_supervisor', None), 'active', None)
        if active is not None:
            contract = json.loads(Path(active['file']).read_text())
            state_path = contract['state_file']
        else:
            state_path = str(stage['folder'] / 'state.json')
            stage['feed'] = self.feed_class(self.mission._g2_host, state_path, context)
        wait_feed_initialization(state_path, stage['room'].episode.limit-stage['room'].episode.elapsed())
        watcher = self.mission.SugarboxWatcher(Path(env['SUGARBOX_HOMOGRAPHY']))
        env = dict(env, G2_RECOVERY_STATE_FILE=state_path,
            G2_RECOVERY_CONTEXT=json.dumps(list(context)),
            G2_RECOVERY_CHECKPOINT_FILE=str(stage['folder'] / 'checkpoint.json'),
            G2_RECOVERY_FOOTPRINT=json.dumps(footprint(self.mission, watcher)),
            G2_RECOVERY_HOLDING=json.dumps(stage['kind'] == 'placement'),
            G2_RECOVERY_FRAME_BOUND_VERIFIED=json.dumps(self.frame_bound_verified),
            G2_RECOVERY_WAIT_USED=str(stage['room'].episode.elapsed()))
        return self.original_start(env)

    def start_patrol(self, host, scene):
        room = self.rooms[host]
        goals = room.checkpoint.data.get('remaining_patrol_goals')
        if not goals:
            return self.original_patrol(host, scene)
        path = '/dev/shm/g2_resume_goals_'+uuid.uuid4().hex+'.json'
        program = 'import sys;open(sys.argv[1],"w").write(sys.argv[2])'
        write = self.mission.ssh(host, 'python3 -c '+shlex.quote(program)+' '+shlex.quote(path)+' '+
                                 shlex.quote(json.dumps(goals, allow_nan=False)))
        if write.returncode:
            raise task.Fault('PATROL_RESUME_REFUSED', 'could not write the saved route')
        command = self.mission.patrol_command(scene).replace('--destination patrol-cycle', '--goals '+shlex.quote(path))
        if '--destination patrol-cycle' in command or '--goals ' not in command:
            raise task.Fault('PATROL_RESUME_REFUSED', 'native route command is incompatible')
        result = self.mission.ssh(host, command)
        if result.returncode:
            raise task.Fault('PATROL_RESUME_REFUSED', 'native route process did not start')
        try:
            pid = int(result.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise task.Fault('PATROL_RESUME_REFUSED', 'native process identifier missing')
        room.checkpoint.save(status='RESUMING_SAVED_ROUTE', resume_goals=goals)
        return pid

    def begin_patrol(self, host, scene, reader, watcher, link, holding=False):
        room = self.rooms[host]
        initial = room._snapshot(holding=holding)
        views = footprint(self.mission, watcher)
        temporary = tempfile.TemporaryDirectory(prefix='g2-patrol-recovery-')
        folder = Path(temporary.name)
        feed = self.feed_class(host, folder/'state.json', initial['context'])
        gate = task.EngineGate(str(folder/'state.json'), initial['context'],
            str(folder/'checkpoint.json'), views, holding, emit=self.emit,
            frame_bound_verified=self.frame_bound_verified)
        gate.episode.used = room.episode.elapsed()
        self.patrols[host] = dict(gate=gate, feed=feed, temporary=temporary, sequence=0, room=room,
                                 last_stamp=None, paused=False, reset=False, native_generation=0)
        room.checkpoint.save(stage='PATROL', arm_outcome='not_started')
        link._g2_host = host

    def patrol_frame(self, host, frame, stamp, sighting, link):
        value = self.patrols[host]
        if stamp != value['last_stamp']:
            value['sequence'] += 1
            value['last_stamp'] = stamp
        native = link.recovery_now()
        problem = None
        if native and native.get('requires_observation'):
            generation = native.get('recovery_generation', 0)
            if generation > value['native_generation']:
                value['native_generation'] = generation
                problem = 'TF_STALE'  # native scan/odom/TF interruption, even before AMCL reports it
                value['room'].checkpoint.save(episode_completion_goal=native.get('target'))
        decision = value['gate'].poll(value['sequence'], time.time()-(time.monotonic()-stamp), sighting, problem)
        if not decision['allow'] and not decision.get('target_lost'):
            link.pause_recovery()
            if decision['reset'] and value['room'].checkpoint.data['stage'] == 'PATROL':
                report = self.mission.patrol_report(host)  # one read per outage, not per frame
                value['room'].checkpoint.save(native_route=report.get('remaining_route'),
                    native_checkpoint=report.get('task_checkpoint'),
                    episode_completion_goal=(report.get('task_checkpoint') or {}).get('target'))
            value['paused'] = True
            return False
        if decision.get('resumed'):
            value['reset'] = True
            value['paused'] = False
            value['room'].checkpoint.save(observation_complete=True, status='REVALIDATE_PATH')
            link._g2_recovery_ack = value['native_generation']
        value['room'].checkpoint.data['last_frame'] = value['gate'].ledger.last
        link._g2_last_frame = value['gate'].ledger.last
        completion = value['room'].checkpoint.data.get('episode_completion_goal')
        if (native and native.get('navigation_complete') is True and native.get('observation_complete') is True
                and native.get('target') == completion and value['gate'].episode.started is None
                and not value['gate'].ledger.pending):
            value['gate'].episode.complete(True, True)
            value['room'].episode.complete(True, True)
            value['room'].checkpoint.save(episode_completion_goal=None)
        return True

    def patrol_reset(self, host):
        value = self.patrols[host]
        reset, value['reset'] = value['reset'], False
        return reset

    def paused_duration(self, host):
        value = self.patrols.get(host)
        if value is None:
            return 0.
        gate = value['gate']
        return gate.paused_total + (time.monotonic()-gate.pause_origin if gate.pause_origin is not None else 0.)

    def transport(self, host, plan, navigate, report_path):
        """Observe during the existing sole-owner map transit, including waits."""
        room = self.rooms[host]
        reader = self.mission._runtime.reader()
        watcher = self.mission.SugarboxWatcher(self.mission._g2_homography)
        link = self.mission.VisionLink(host, report_path=report_path)
        stop, ready, errors = threading.Event(), threading.Event(), []
        self.begin_patrol(host, None, reader, watcher, link, holding=True)
        room.checkpoint.save(stage='HELD_TRANSIT', target=plan['staging'], holding=True,
                             arm_outcome='grasped')
        requested = time.monotonic()
        def observe():
            last = None
            try:
                while not stop.is_set():
                    frame, stamp = reader.latest()
                    if frame is None or stamp == last or stamp-.30 <= requested:
                        stop.wait(.04)
                        continue
                    if time.monotonic()-stamp > 1.5:
                        raise task.Fault('OBSERVATION_SOURCE_MISSING', 'transit image source stale')
                    sighting = watcher.look(frame)
                    if stop.is_set():
                        break
                    allowed = self.patrol_frame(host, frame, stamp, sighting, link)
                    if allowed:
                        link.send(True, time.monotonic()-stamp+.30)
                        ready.set()
                    last = stamp
            except BaseException as exc:
                errors.append(exc)
                link.pause_recovery()
                ready.set()
        thread = threading.Thread(target=observe, daemon=True)
        thread.start()
        try:
            if not ready.wait(30.) or errors:
                raise task.Fault('TRANSIT_OBSERVATION_REFUSED', str(errors[0]) if errors else 'no fresh view')
            result = navigate()
            if errors:
                raise errors[0]
            return result
        finally:
            stop.set()
            thread.join(timeout=3.)
            link.close()
            try:
                self.end_patrol(host)
            finally:
                reader.close()
            if thread.is_alive():
                raise task.Fault('OBSERVER_STUCK', 'model observer did not exit; do not reuse its worker')

    def end_patrol(self, host):
        value = self.patrols.pop(host, None)
        if value is None:
            return
        try:
            value['feed'].close()
            if value['gate'].last_good is not None:
                value['room']._anchor = value['gate'].last_good
            value['room'].episode.used = max(value['room'].episode.used, value['gate'].episode.elapsed())
            value['room'].checkpoint.save(last_frame=value['gate'].ledger.last,
                observation_complete=not value['gate'].ledger.pending)
            if value['room'].checkpoint.data['stage'] == 'PATROL':
                report = self.mission.patrol_report(host)
                native = report.get('task_checkpoint') or {}
                if not report.get('ok') and (not native or 'remaining_route' not in report):
                    raise task.Fault('PATROL_CHECKPOINT_MISSING', 'unfinished patrol has no trustworthy saved route')
                expected = value['room'].checkpoint.data.get('context')
                if native.get('context') is not None and expected is not None and tuple(native['context']) != tuple(expected):
                    raise task.Fault('CONTEXT_CHANGED', 'saved patrol route belongs to another boot/owner/map')
                goals = suspended_goals(report) if not report.get('ok') else []
                value['room'].checkpoint.save(remaining_patrol_goals=goals,
                    native_checkpoint=report.get('task_checkpoint'), native_route=report.get('remaining_route'))
        finally:
            value['temporary'].cleanup()
