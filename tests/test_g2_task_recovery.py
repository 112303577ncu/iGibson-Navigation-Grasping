"""Fault injection at motion/observation boundaries, without ROS or actuators."""
import importlib.util
import ast
import io
import json
import math
import textwrap
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from integration import g2_task_recovery as task
from integration import g2_mission_recovery as recovery
from integration import g2_patrol_reentry as reentry
from integration import g2_recovery_patches as patches

spec = importlib.util.spec_from_file_location('task_recovery_fakes', Path(__file__).with_name('test_g2_patrol_reentry.py'))
fakes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fakes)


class Clock:
    def __init__(self): self.t = 10.
    def now(self): return self.t
    def sleep(self, seconds): self.t += seconds


def sample(stamp, x=1., context=('boot', 7, 'map')):
    return dict(pose=[x, 1., 0.], odom=[x, 1., 0.], stamp=stamp, context=context)


def test_repeated_reentry_and_flapping_spend_one_allowance():
    clock = Clock()
    episode = task.Episode(clock=clock.now)
    for _ in range(3):
        episode.pause('SCAN_STALE')
        clock.sleep(8.)
        episode.resume()
        clock.sleep(100.)  # healthy execution is not localization waiting
    episode.pause('TF_STALE')
    clock.sleep(6.)
    with pytest.raises(task.Fault, match='RECOVERY_TIMEOUT'):
        episode.check()
    with pytest.raises(task.Fault, match='CHECKPOINT_INCOMPLETE'):
        episode.complete(True, False)


def test_wait_snapshot_reentry_does_not_reset_supervisor_wait():
    clock, stop = Clock(), Mock()
    episode = task.Episode(clock=clock.now)
    episode.pause('AMCL_STALE', 20.)
    clock.sleep(2.)
    episode.resume()
    def read(): raise recovery.RecoveryFault('TF_STALE', 'flapping', True)
    with pytest.raises(task.Fault, match='RECOVERY_TIMEOUT'):
        recovery.wait_snapshot(read, stop, episode=episode, clock=clock.now, sleep=clock.sleep)
    assert 21.9 <= clock.t <= 22.2  # two local seconds overlapped the inherited twenty


def test_an_observed_completed_leg_is_the_only_budget_reset():
    clock = Clock();episode = task.Episode(clock=clock.now)
    episode.pause('ODOM_STALE');clock.sleep(4.);episode.resume()
    assert episode.used == 4.
    episode.complete(True, True)
    assert episode.used == 0. and not episode.pending


@pytest.mark.parametrize('context', [('new', 7, 'map'), ('boot', 8, 'map'), ('boot', 7, 'new')])
def test_changed_context_is_not_recoverable(context):
    with pytest.raises(task.Fault, match='CONTEXT_CHANGED'):
        task.continuity(sample(1), sample(2, context=context))


def test_absolute_pose_jump_is_not_a_continuous_odom_motion():
    value = sample(2)
    value['pose'][0] += .3
    with pytest.raises(task.Fault, match='REFERENCE_REQUIRED'):
        task.continuity(sample(1), value)


def test_navigation_and_observation_have_separate_completion(tmp_path):
    cp = task.Checkpoint('PATROL', target='B', path=str(tmp_path/'checkpoint.json'))
    cp.save(route=['A', 'B', 'C'], leg_index=0, navigation_complete=True)
    value = json.loads((tmp_path/'checkpoint.json').read_text())
    assert value['navigation_complete'] and not value['observation_complete']
    assert value['route'][value['leg_index']+1] == 'B'
    for name in ('start_odom', 'progress_m', 'holding', 'arm_outcome', 'last_frame', 'context'):
        assert name in value


FOOTPRINT = [[.5, -.5], [1.5, -.5], [1.5, .5], [.5, .5]]


def test_stopped_new_view_covers_only_the_previously_unobserved_sweep():
    ledger = task.ObservationLedger(FOOTPRINT)
    ledger.observe(1, 10., [1., 1., 0.], .04)
    ledger.interrupt([1.1, 1., 0.], after=11.)
    assert ledger.pending
    assert not ledger.observe(1, 11.1, [1.1, 1., 0.], .04)
    assert not ledger.observe(2, 10., [1.1, 1., 0.], .04)
    assert ledger.pending
    assert ledger.observe(2, 11.2, [1.1, 1., 0.], .04)
    ledger.require_complete()


def test_a_new_frame_looking_the_wrong_way_does_not_clear_the_gap():
    ledger = task.ObservationLedger(FOOTPRINT)
    ledger.observe(1, 10., [1., 1., 0.])
    ledger.interrupt([1.2, 1., 0.], 11.)
    ledger.observe(2, 11.2, [1.2, 1., math.pi])
    with pytest.raises(task.Fault, match='OBSERVATION_GAP_UNRESOLVED'):
        ledger.require_complete()


class EnginePlant:
    def __init__(self, folder):
        self.clock, self.seq = Clock(), 0
        self.state = folder/'state.json'
        self.file = folder/'checkpoint.json'
        self.context = ['boot', 7, 'map']
        self.gate = task.EngineGate(str(self.state), self.context, str(self.file), FOOTPRINT,
                                    False, clock=self.clock.now, wall=self.clock.now,
                                    frame_bound_verified=True)  # simulated, known source timing
        self.stamp, self.x = 0., 1.

    def tick(self, ready=True, target=True, sequence=None, frame=None, stamp=None, code='AMCL_STALE'):
        self.clock.sleep(.1);self.seq += 1;self.stamp += 1
        task.atomic(self.state, dict(ready=ready, context=self.context, received_at=self.clock.t,
            source_age_s=0., pose=[self.x, 1., 0.], odom=[self.x, 1., 0.],
            stamp=self.stamp if stamp is None else stamp, measured_vx=0., measured_wz=0.,
            reason_code=code, wait_elapsed_s=0.))
        return self.gate.poll(self.seq if sequence is None else sequence,
            self.clock.t if frame is None else frame, {'x': .8, 'y': 0.} if target else None)

    def warm(self):
        for _ in range(8): self.tick()

    def resume(self, target=True):
        result = None
        for _ in range(20):
            result = self.tick(target=target)
            if result.get('resumed'):return result
        raise AssertionError('fresh localization and views never resumed')


@pytest.mark.parametrize('target', [True, False])
def test_target_entering_or_leaving_the_gap_is_observed_again(tmp_path, target):
    plant = EnginePlant(tmp_path);plant.warm()
    previous = plant.gate.ledger.last['sequence']
    decision = plant.tick(ready=False)
    assert not decision['allow'] and decision['reset']
    plant.x += .1
    decision = plant.resume(target=target)
    assert decision['reset'] and decision['target_lost'] is (not target)
    assert plant.gate.ledger.last['sequence'] > previous
    assert not plant.gate.ledger.pending


def test_old_localization_and_repeated_images_cannot_release_a_worker(tmp_path):
    plant = EnginePlant(tmp_path);plant.warm();plant.tick(ready=False)
    for _ in range(8):
        assert not plant.tick(stamp=1.)['allow']
    for _ in range(8):
        assert not plant.tick(sequence=1, frame=10.)['allow']
    assert plant.resume()['resumed']


def test_continuous_failure_and_flapping_expire_in_the_same_engine_allowance(tmp_path):
    plant = EnginePlant(tmp_path);plant.warm()
    with pytest.raises(task.Fault, match='RECOVERY_TIMEOUT'):
        for i in range(400):plant.tick(ready=(i % 4 != 0))
    assert plant.clock.t < 42.


def test_paused_parent_timeout_is_bounded_and_does_not_refresh_stale_evidence(tmp_path):
    path = tmp_path/'worker.json'
    task.atomic(path, dict(paused_total_s=5., paused_at=10.))
    assert task.effective_pause(str(path), now=20.) == 15.
    assert task.effective_pause(str(path), now=100.) == 30.
    task.atomic(path, dict(paused_total_s=5., paused_at=None))
    assert task.effective_pause(str(path), now=100.) == 5.


@pytest.mark.parametrize('when', ['before', 'halfway', 'coasted'])
def test_native_inner_localization_failure_revalidates_actual_remaining_back(tmp_path, monkeypatch, when):
    plant = fakes.Plant(1.5)
    mission, health = fakes.fake_mission(plant)
    clock = Clock()
    native_reading = [False]
    failed = [False]
    old_read = mission.dock.read_localization
    def read(host):
        value = old_read(host)
        trigger = len(plant.commands) == (0 if when == 'before' else 1)
        if native_reading[0] and trigger and not failed[0]:
            failed[0] = True
            if when == 'coasted':
                plant.pose[0] -= .1;plant.odom[0] -= .1
            value.update(ready=False, reason_code='AMCL_STALE', reason='waiting for fresh scan')
        return value
    mission.dock.read_localization = read
    def native_back(host, metres, walls, localize=None, **kw):
        native_reading[0] = True
        try:localize(host)  # the inner read that used to abort the whole mission
        finally:native_reading[0] = False
        plant.back(metres)
    mission.runner.back_straight = native_back
    def delegate(self, metres):mission.runner.back_straight(self.host, metres, self.walls, obstacles=self.obstacles)
    mission.Room._back = delegate
    wait = recovery.wait_snapshot
    monkeypatch.setattr(recovery, 'wait_snapshot', lambda read, stop, **kw:
        wait(read, stop, clock=clock.now, sleep=clock.sleep, **kw))
    reentry.bind_mission(mission, 'scene', tmp_path, lambda host: health, recovery=recovery)
    room = mission.Room('host', walls=[1], mask=fakes.Mask())
    room.episode = task.Episode(clock=clock.now)
    mission.runner.back_straight('host', .20, [1])
    assert failed[0]
    assert sum(plant.commands) == pytest.approx(.1 if when == 'coasted' else .2)
    assert any(e['status'] == 'WAIT_LOCALIZATION' for e in room.recovery_events)
    assert room.checkpoint.data['progress_m'] == pytest.approx(.2)


def test_deviation_or_excess_coast_is_not_retried():
    progress = task.MotionProgress([0., 0., 0.], .2)
    assert progress.remaining([-.2, 0., 0.])[0] == 0.
    with pytest.raises(task.Fault, match='MOTION_DEVIATED'):progress.remaining([-.1, .05, 0.])
    with pytest.raises(task.Fault, match='MOTION_OVERRUN'):progress.remaining([-.26, 0., 0.])


@pytest.mark.parametrize('intermediate',[False,True])
def test_patrol_replans_to_b_before_any_c_after_a_localization_pause(monkeypatch,intermediate):
    spec = importlib.util.spec_from_file_location('task_patrol_test',
        Path(__file__).resolve().parents[1]/'deploy/ros/g2_map_patrol.py')
    patrol = importlib.util.module_from_spec(spec);spec.loader.exec_module(patrol)
    runner = patrol.Runner.__new__(patrol.Runner)
    a, b, c = [0., 0.], [1., 0.], [1., 1.]
    runner.report = dict(remaining_route=[b, c])
    runner.ready = Mock(side_effect=[dict(pose=a+[0.]), dict(pose=[.4, 0., 0.])])
    p1,p2=[.2,.1],[.6,.1]
    planner=Mock(side_effect=[[a,p1,p2,b],[ [.4,0.],p2]] if intermediate else [[a,b],[[.4,0.],b]])
    runner.policy = SimpleNamespace(plan=planner)
    runner.stop = Mock()
    runner.navigate = Mock(side_effect=[None,patrol.Transient('replan required after localization correction'),None,None]
                          if intermediate else [patrol.Transient('replan required after localization correction'),None])
    monkeypatch.setattr(patrol, 'checkpoint', lambda report: None)
    runner.goto(b, 'A_to_B')
    assert [call.args[0] for call in runner.navigate.call_args_list] == ([p1,p2,p2,b] if intermediate else [b,b])
    assert planner.call_args.args[1] == (p2 if intermediate else b)
    assert runner.report['remaining_route'] == [b, c]


def test_actual_poll_hook_freezes_no_progress_and_discards_queued_actions():
    # Execute the actual injected hook with the project's real watchdog class.
    source = (Path(__file__).resolve().parents[1]/'integration/sugarbox_rl_approach_final2.py').read_text(encoding='utf-8')
    node = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == 'ProgressWatchdog')
    env = dict(math=math, NO_PROGRESS_WINDOW_S=10., NO_PROGRESS_MIN_GAIN_M=.02)
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<real watchdog>', 'exec'), env)
    progress = env['ProgressWatchdog']();progress.update(1., 0.)
    calls = [0]
    def poll(*args, **kwargs):
        calls[0] += 1
        env['now'] = 20.+calls[0]
        return dict(allow=calls[0]>1, reset=True)
    names = ('target', 'contract_action', 'anti_jitter', 'center_controller', 'final_approach',
             'pursuit', 'blind_finish', 'motor')
    env.update({name: Mock() for name in names})
    env.update(g2_recovery=SimpleNamespace(poll=poll), arrived_latched=False, blocked_latched=False,
        progress=progress, now=0., scan=SimpleNamespace(stamp=0.), odom=SimpleNamespace(stamp=0.),
        SCAN_STALE_TIMEOUT=.3, ODOM_STALE_TIMEOUT=.5, get_age=lambda *args: 0.,
        vision_sequence=1, vision_frame_time=1., vision_result={}, vision_reason='VALID',
        np=SimpleNamespace(zeros=lambda *args, **kw: [0., 0.], float32='float32'), MOTOR_DELAY_STEPS=2,
        SeedFacing=Mock(), SeedLook=Mock(), FarSighting=Mock(), headless=True,
        time=SimpleNamespace(sleep=lambda _: None), action_delay=[[.8, .4]], prev_raw_action=[.8, .4])
    hook = textwrap.dedent(patches.ENGINE_POLL)
    stores = {n.id for n in ast.walk(ast.parse('while True:\n'+textwrap.indent(hook, '    ')))
              if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store)}
    body = ('def drive():\n    global '+','.join(sorted(stores))+'\n    for tick in range(2):\n'
            +textwrap.indent(hook, '        ')
            +'\n        return progress.update(1., now), action_delay, prev_raw_action\n')
    exec(body, env)
    blocked, queue, previous = env['drive']()
    assert not blocked and queue == [[0., 0.], [0., 0.]] and previous == [0., 0.]
    env['motor'].stop.assert_called_once()
    env['motor'].send_velocity.assert_not_called()


def test_native_patrol_requires_the_new_observation_ack_before_any_motion(tmp_path, monkeypatch):
    import builtins
    spec = importlib.util.spec_from_file_location('task_patrol_ack',
        Path(__file__).resolve().parents[1]/'deploy/ros/g2_map_patrol.py')
    patrol = importlib.util.module_from_spec(spec);spec.loader.exec_module(patrol)
    runner = patrol.Runner.__new__(patrol.Runner)
    runner.pid=7;runner.vision_boot='boot';runner.report={};runner.end=200.;runner.last_health=100.
    runner.vision_permit=str(tmp_path/'permit.json')
    runner._task_observation_pending=True;runner._task_observation_generation=2
    runner.task_recovery_enabled=True
    runner.policy=SimpleNamespace(allowed=lambda *args: True)
    runner.sensors=SimpleNamespace(state=lambda: dict(pose=[1.,1.,0.],odom=[1.,1.,0.],
                                                     odom_stamp=3.,cov=[.000025]*3))
    monkeypatch.setattr(patrol.time, 'time', lambda: 100.)
    monkeypatch.setattr(patrol, 'available_mb', lambda: 2048.)
    original = builtins.open
    def read(path, *args, **kw):
        if str(path)=='/dev/shm/g2_localization_health.json':
            return io.StringIO(json.dumps(dict(ready=True,owner_pid=7,boot_id='boot',map_hash='map',stamp=99.9)))
        return original(path,*args,**kw)
    monkeypatch.setattr(builtins, 'open', read)
    permit=dict(owner_pid=7,boot_id='boot',source_stamp=99.9,allow=True,recovery_ack=1)
    Path(runner.vision_permit).write_text(json.dumps(permit))
    with pytest.raises(patrol.Transient, match='mission recovery pause'):runner.state()
    permit['recovery_ack']=2;Path(runner.vision_permit).write_text(json.dumps(permit))
    assert runner.state()['odom_stamp']==3.
    assert not runner._task_observation_pending


def test_real_opencv_hull_shape_and_arm_image_mask_are_respected():
    from integration.g2_task_coordinator import footprint
    watcher=SimpleNamespace(hull=[[[0.,0.]],[[2.,0.]],[[2.,2.]],[[0.,2.]]],
                             H=None,rear_cam_to_front=0.)
    mission=SimpleNamespace(ARM_CAMERA_REGION=(1.,1.,3.,3.),GOAL_ORIGIN_OFFSET_X_M=0.,
                            pixel_to_ground=lambda u,v,H:(u,v))
    views=footprint(mission,watcher)
    assert sum(task.area(poly) for poly in views)==pytest.approx(3.)
    assert len(views)>1  # never fill the excluded image region with one convex hull


@pytest.mark.parametrize('arm', ['empty', 'unknown'])
def test_completed_release_is_not_replayed_and_unknown_is_fatal(tmp_path, arm):
    from integration.g2_task_coordinator import Coordinator
    room=SimpleNamespace(_snapshot=lambda **kw: sample(1), episode=task.Episode(),
        checkpoint=task.Checkpoint('PLACE'), _anchor=None)
    native=Mock(return_value=0)
    mission=SimpleNamespace(run_stage=native, runner=SimpleNamespace(start_engine=Mock()),
                            arm_state=lambda host:arm)
    coordinator=Coordinator(mission,{'host':room},tmp_path,Mock())
    if arm=='unknown':
        with pytest.raises(task.Fault,match='ACTION_OUTCOME_UNKNOWN'):
            mission.run_stage('host',['--target','placement'],tmp_path/'stage.log')
        assert room.checkpoint.data['arm_outcome']=='unknown'
    else:
        assert mission.run_stage('host',['--target','placement'],tmp_path/'stage.log')==0
        assert room.checkpoint.data['arm_outcome']=='released'
        assert room.checkpoint.data['holding'] is False
    assert native.call_count==1


def test_unmeasured_frame_timing_keeps_the_acceptance_gate_closed(tmp_path):
    plant=EnginePlant(tmp_path);plant.warm()
    plant.gate.frame_bound_verified=False
    plant.tick(ready=False)
    with pytest.raises(task.Fault,match='OBSERVATION_PROVENANCE_REQUIRED'):
        for _ in range(400):
            assert not plant.tick()['allow']
    assert plant.gate.checkpoint.data['observation_complete'] is False


def test_frame_bound_evidence_binds_both_pipelines_and_the_measurement_file(tmp_path):
    import hashlib
    from integration.g2_task_coordinator import verified_frame_bound
    sender=tmp_path/'deploy/ros/start_rear_approach_stream.sh'
    receiver=tmp_path/'integration/rear_stream.py'
    sender.parent.mkdir(parents=True);receiver.parent.mkdir(parents=True)
    sender.write_text('synthetic source');receiver.write_text('synthetic decoder')
    proof=tmp_path/'measured.txt';proof.write_text('synthetic timing evidence')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    data=dict(schema=1,verified=True,upper_bound_s=.30,sender_sha256=sha(sender),
              receiver_sha256=sha(receiver),proof_file=proof.name,proof_sha256=sha(proof))
    record=tmp_path/'bound.json';record.write_text(json.dumps(data))
    assert verified_frame_bound(record,tmp_path)
    proof.write_text('changed')
    with pytest.raises(task.Fault,match='OBSERVATION_PROVENANCE_REQUIRED'):
        verified_frame_bound(record,tmp_path)
    data['upper_bound_s']=.5;record.write_text(json.dumps(data))
    with pytest.raises(task.Fault,match='OBSERVATION_PROVENANCE_REQUIRED'):
        verified_frame_bound(record,tmp_path)


def test_saved_patrol_keeps_unobserved_sublegs_before_the_next_waypoint():
    from integration.g2_task_coordinator import suspended_goals
    report=dict(task_checkpoint=dict(navigation_complete=False,remaining_path=[[.6,0.],[1.,0.]]),
                remaining_route=[dict(point=[1.,0.]),dict(point=[1.,1.])])
    assert [goal['point'] for goal in suspended_goals(report)]==[[.6,0.],[1.,0.],[1.,1.]]


def test_stalled_previously_ready_feed_is_a_recoverable_pause(tmp_path):
    plant=EnginePlant(tmp_path);plant.warm()
    value=json.loads(plant.state.read_text());value['reason_code']='READY'
    plant.state.write_text(json.dumps(value));plant.clock.sleep(.5)
    decision=plant.gate.poll(99,plant.clock.t,None)
    assert not decision['allow'] and decision['reset']
    assert plant.gate.checkpoint.data['status']=='WAIT_LOCALIZATION'
    assert plant.resume()['resumed']


def test_new_sequences_with_failed_inference_do_not_cover_the_gap(tmp_path):
    plant=EnginePlant(tmp_path);plant.warm();plant.tick(ready=False)
    original=plant.gate.poll
    plant.gate.poll=lambda *a,**kw: original(*a,observation_valid=False,**kw)
    for _ in range(20):
        assert not plant.tick(target=False)['allow']
    assert plant.gate.episode.started is not None
    assert plant.gate.vision_count==0
    plant.gate.poll=original
    assert plant.resume()['resumed']


def test_turning_gap_expands_instead_of_disappearing_from_coverage():
    ledger=task.ObservationLedger(FOOTPRINT)
    ledger.observe(1,10.,[0.,0.,0.],.04)
    ledger.interrupt([0.,0.,.2],11.)
    assert ledger.pending
    ledger.observe(2,11.5,[0.,0.,.2],.04)
    with pytest.raises(task.Fault,match='OBSERVATION_GAP_UNRESOLVED'):
        ledger.require_complete()


def test_success_exit_without_confirmed_grasp_is_never_saved_as_grasped(tmp_path):
    from integration.g2_task_coordinator import Coordinator
    room=SimpleNamespace(_snapshot=lambda **kw:sample(1),episode=task.Episode(),
                         checkpoint=task.Checkpoint('FETCH'),_anchor=None)
    native=Mock(return_value=0)
    mission=SimpleNamespace(run_stage=native,runner=SimpleNamespace(start_engine=Mock()),
                            arm_state=lambda host:'empty')
    Coordinator(mission,{'host':room},tmp_path,Mock())
    with pytest.raises(task.Fault,match='ACTION_STATE_MISMATCH'):
        mission.run_stage('host',['--target','sugarbox'],tmp_path/'stage.log')
    assert native.call_count==1
    assert room.checkpoint.data['arm_outcome']=='not_confirmed'
    assert 'last_confirmed_arm_outcome' not in room.checkpoint.data


def test_one_bad_native_pulse_sample_always_creates_observation_barrier(monkeypatch):
    spec=importlib.util.spec_from_file_location('task_native_pulse',
        Path(__file__).resolve().parents[1]/'deploy/ros/g2_map_patrol.py')
    patrol=importlib.util.module_from_spec(spec);spec.loader.exec_module(patrol)
    runner=patrol.Runner.__new__(patrol.Runner)
    runner.ready=Mock(return_value=dict(odom=[0.,0.,0.]))
    runner.state=Mock(side_effect=patrol.Transient('one stale scan'))
    runner.stop=Mock();runner.report=dict(pulses=0);runner.vision_permit='synthetic'
    monkeypatch.setattr(patrol,'checkpoint',lambda report:None)
    with pytest.raises(patrol.Transient,match='one stale scan'):
        runner.pulse(.15,0.,.1)
    assert runner._task_episode.started is not None
    assert runner._task_observation_pending
    assert runner.report['task_checkpoint']['requires_observation']


def test_missing_post_stage_anchor_cannot_bypass_the_jump_check():
    stop=Mock()
    def read(): raise task.Fault('AMCL_STALE','one bad sample',True)
    with pytest.raises(task.Fault,match='CHECKPOINT_MISSING'):
        recovery.wait_snapshot(read,stop,previous=sample(1),require_anchor=True)
    stop.assert_called_once()


def test_stage_archives_the_latest_worker_anchor_for_the_exit(tmp_path):
    from integration.g2_task_coordinator import Coordinator
    room=SimpleNamespace(_snapshot=lambda **kw:sample(1),episode=task.Episode(),
                         checkpoint=task.Checkpoint('PLACE'),_anchor=None)
    mission=SimpleNamespace(runner=SimpleNamespace(start_engine=Mock()),arm_state=lambda host:'empty')
    def stage(*args):
        task.atomic(coordinator.stage['folder']/'checkpoint.json',dict(last_valid_state=sample(9),wait_used_s=3.))
        return 0
    mission.run_stage=stage
    coordinator=Coordinator(mission,{'host':room},tmp_path,Mock())
    mission.run_stage('host',['--target','placement'],tmp_path/'stage.log')
    assert room._anchor==sample(9,context=['boot',7,'map'])
    assert room.episode.used==3.


def test_native_recovery_requires_new_stopped_samples_not_coasting_or_duplicates(monkeypatch):
    spec=importlib.util.spec_from_file_location('task_native_stability',
        Path(__file__).resolve().parents[1]/'deploy/ros/g2_map_patrol.py')
    patrol=importlib.util.module_from_spec(spec);spec.loader.exec_module(patrol)
    runner=patrol.Runner.__new__(patrol.Runner);clock=Clock()
    runner.report={};runner.end=1000.;runner.task_recovery_enabled=True
    runner._task_context=('boot',7,'map');runner._task_last_good=sample(10)
    runner._task_episode=task.Episode(clock=clock.now);runner._task_episode.pause('SCAN_STALE')
    frames=[(10,0.),(11,.05),(12,0.),(12,0.),(13,0.),(14,0.),(15,0.),(16,0.)]
    def read():
        stamp,vx=frames.pop(0)
        return dict(pose=[1.,1.,0.],odom=[1.,1.,0.],odom_stamp=stamp,vx=vx,wz=0.)
    runner.state=read
    monkeypatch.setattr(patrol.time,'sleep',clock.sleep)
    monkeypatch.setattr(patrol.time,'time',clock.now)
    monkeypatch.setattr(patrol.task_recovery,'monotonic',clock.now)
    assert runner.ready()['odom_stamp']==16
    assert frames==[]
    assert runner._task_episode.started is None
    assert runner._task_paused_total>0


def test_stage_exception_is_an_unknown_result_even_if_holding_can_be_read(tmp_path):
    from integration.g2_task_coordinator import Coordinator
    room=SimpleNamespace(_snapshot=lambda **kw:sample(1),episode=task.Episode(),
                         checkpoint=task.Checkpoint('PLACE'),_anchor=None)
    native=Mock(side_effect=RuntimeError('release reply lost'))
    mission=SimpleNamespace(run_stage=native,runner=SimpleNamespace(start_engine=Mock()),
                            arm_state=lambda host:'holding')
    Coordinator(mission,{'host':room},tmp_path,Mock())
    with pytest.raises(task.Fault,match='ACTION_OUTCOME_UNKNOWN'):
        mission.run_stage('host',['--target','placement'],tmp_path/'stage.log')
    assert native.call_count==1 and room.checkpoint.data['arm_outcome']=='unknown'


def test_sensor_fault_classification_is_shared_and_board_state_is_not_retryable():
    assert recovery.TRANSIENT_CODES is task.TRANSIENT
    assert not recovery.localization_fault(dict(reason_code='BOARD_STALE')).retryable


def test_feed_bootstrap_waits_before_worker_start_without_promoting_not_ready(tmp_path):
    from integration.g2_task_coordinator import wait_feed_initialization
    path=tmp_path/'feed.json';task.atomic(path,dict(ready=False));clock=Clock()
    def sleep(seconds):
        clock.sleep(seconds)
        task.atomic(path,dict(ready=False,context=['boot',7,'map'],reason_code='AMCL_STALE'))
    wait_feed_initialization(path,1.,clock.now,sleep)
    assert json.loads(path.read_text())['ready'] is False
    task.atomic(path,dict(ready=False))
    with pytest.raises(task.Fault,match='LOCALIZATION_FEED_START_TIMEOUT'):
        wait_feed_initialization(path,.1,clock.now,clock.sleep)
