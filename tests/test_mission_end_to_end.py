#!/usr/bin/env python3
"""Drive a whole mission on stubs: patrol -> spot -> approach -> grasp -> bin -> patrol.

The per-module selftests each prove one piece. This proves they compose: the
real MissionFSM and the real MissionRunner._act run against fake hardware for a
complete cycle, and every state the mission is supposed to pass through is
actually visited in order.

It is not a physics simulation and does not pretend to be. Distances shrink
because the stub says so. What it does check is the part that has repeatedly
been wrong -- the bookkeeping between states: which flags are set, when the
tracker survives a reset, when the goal source flips, that the arm is folded
while driving and raised before the camera is used, and that the run ends where
it started rather than wedged in some state with no exit.

Run:
    python3 tests/test_mission_end_to_end.py
"""
from __future__ import annotations

import io
import math
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

_X3PLUS = Path(__file__).resolve().parent.parent
if str(_X3PLUS) not in sys.path:
    sys.path.insert(0, str(_X3PLUS))

from integration import map_goal_provider as mgp
from integration import mission_fsm as mfsm
from integration import mission_pipeline as mp


# ════════════════════════════════════════════════════════════════════════════
# Fakes
# ════════════════════════════════════════════════════════════════════════════

NAV_HOME = mp.NAV_HOME_DEG
C3 = mp.GRASP_HOME_DEG


class FakeCfg:
    home_deg = NAV_HOME
    grasp_home_deg = C3
    documented_x_range = (0.20, 0.28)
    documented_y_range = (-0.09, 0.08)
    envelope_tol_m = 0.001


class FakeMapper:
    def hw_deg_to_sim_arm(self, deg):
        return np.asarray([math.radians(d - 90.0) for d in deg[:5]], dtype=np.float64)

    def hw_deg_to_sim_grip(self, deg):
        return math.radians(float(deg) - 90.0)


class FakeController:
    """Records the arm poses it is asked for; never fails unless told to."""

    def __init__(self, *, grasp_ok=True, verify_ok=True):
        self.cfg = FakeCfg()
        self.mapper = FakeMapper()
        self._current_grip_rad = 0.0
        self.obj_provider = None
        self.moves = []           # labels, in order
        self.ran_grasp = 0
        self.ran_release = 0
        self.grasp_ok = grasp_ok
        self.verify_ok = verify_ok

    def move_guarded_and_verified(self, arm, grip, *, label, **kw):
        self.moves.append(label)
        return {"reached": True, "reason": "arrived", "iters": 1, "guard": ["pass"]}

    def move_home(self):
        self.moves.append("move_home")
        return True

    def run(self, max_steps=300):
        self.ran_grasp += 1
        return self.grasp_ok

    def run_release_only(self):
        self.ran_release += 1
        return "released"

    def close(self):
        pass


class FakeDevice:
    """The Rosmaster handle the self-check talks to.

    Implements exactly the two calls run_self_check makes: enabling auto
    reporting (v21's ServoController does not) and the liveness probe that
    proves the board is really streaming, without which get_motion_data()
    returns its initial zeros forever.
    """

    def __init__(self, *, reports=True):
        self.auto_report = None
        self._reports = reports

    def set_auto_report_state(self, enable, forever=False):
        self.auto_report = (enable, forever)

    def get_battery_voltage(self):
        return 12.4 if self._reports else 0.0


class FakeNav:
    """Chassis + cameras. The object closes in as the robot 'drives'."""

    def __init__(self, ncfg, *, start_dist=3.0, align_ok=True, reports=True):
        self.ncfg = ncfg
        self.device = FakeDevice(reports=reports)
        self.dry_run = True
        self.show = False
        # The object's real distance, which is NOT the tracker: the tracker is
        # an estimate that gets cleared on goal-source changes, while the camera
        # keeps seeing whatever is actually there. Conflating them hides exactly
        # the bug this suite exists to catch.
        self.obj_dist = start_dist
        self.tracker = _Tracker(start_dist)
        self.align_ok = align_ok
        self.stops = 0
        self.drives = 0
        self.arm_detects = 0
        self.reset_calls = []
        self.prev_action = np.zeros(2, dtype=np.float32)

    # chassis
    def stop(self):
        self.stops += 1

    def back_off(self):
        pass

    def _advance(self, step):
        self.obj_dist = max(0.2, self.obj_dist - step)
        self.tracker.close_in(step)

    def nav_tick(self, dist, bearing, dt, points):
        self.drives += 1
        self._advance(0.15)
        return mp.NavTick(0.3, 0.0, False, False, 4.0, float("inf"))

    def creep(self, vx, wz, dt):
        self.drives += 1
        self._advance(0.05)

    def reset_nav(self, clear_tracker=True):
        self.reset_calls.append(clear_tracker)
        if clear_tracker:
            self.tracker = _Tracker(float("inf"))

    # cameras -- these read the world, not the tracker
    def _detect_rear(self):
        return (True, self.obj_dist, 0.0)

    def _detect_arm(self):
        self.arm_detects += 1
        return (True, self.obj_dist, 0.0, 40.0, "sugarbox")

    def _arm_align(self):
        self.obj_dist = 0.24
        self.tracker.set(0.24)
        return self.align_ok

    def latch_arm_object(self):
        return ([0.24, 0.0, 0.0325], 0.023, 0.065)

    def verify_grasp(self, pos):
        return True

    def open_cameras(self):
        pass

    def release(self):
        pass


class _Tracker:
    def __init__(self, d):
        self._d = d
        self._age = 0.0

    def set(self, d):
        self._d = d

    def update(self, forward_dist, offset_right):
        self._d = forward_dist
        self._age = 0.0

    def close_in(self, step):
        if math.isfinite(self._d):
            self._d = max(0.2, self._d - step)

    @property
    def has_fix(self):
        return math.isfinite(self._d)

    def dist(self):
        return self._d

    def bearing(self):
        return 0.0

    def fix_age(self, now=None):
        return self._age


class FakeLidar:
    def get_points(self):
        return [(a - 90.0, 4.0) for a in range(0, 181, 5)]

    def age(self):
        return 0.0

    def scan_info(self, cfg):
        return {"frame_id": "laser_link", "warnings": [], "angle_min_deg": -85.0,
                "angle_max_deg": 85.0, "forward_fraction": 0.94, "count": 340}

    def close(self):
        pass


class FakeOdom:
    """Always valid, fresh and stationary -- the FSM's health inputs."""
    valid = fresh = stationary = True
    reason = ""
    x = y = yaw = vx = vy = wz = 0.0


class FakeOdomPub:
    ticks = 5

    def state(self):
        return FakeOdom()

    def stop(self):
        pass

    def join(self, timeout=None):
        pass


class FakeRos:
    def __init__(self, goals):
        self.goals = goals

    def latest_pose(self):
        return self.goals.pose or mgp.MapPose(0.0, 0.0, 0.0, 1e9)

    def pose_quality_ok(self):
        return True, ""

    def close(self):
        pass


class FakeArgs:
    real = False
    show = False
    max_steps = 300
    max_laps = 0
    no_deliver = False
    wait_start = False
    exit_on_pause = True
    detection_streak = 3
    detection_jump_m = 0.35
    blacklist_radius_m = 1.0
    handoff_aim_x = 0.24
    handoff_radius_m = 0.04
    nav_stop_dist = 0.75
    bin_rim_height = 0.0
    release_clearance = 0.03


def build_runner(*, grasp_ok=True, align_ok=True, deliver=True):
    try:
        from integration import nav_rl as nr
        ncfg = nr.NavRLConfig()
    except Exception:                                   # pragma: no cover
        ncfg = type("C", (), {"control_period_s": 1 / 6.0,
                              "lidar_stale_timeout_s": 0.7,
                              "safety_brake_dist": 0.25,
                              "motor_delay_steps": 2})()

    route = mgp.RouteSpec(
        waypoints=[mgp.Waypoint(f"p{i}", i * 0.8, 0.0) for i in range(8)],
        loop=True, bin_center=(6.0, 1.0), bin_approach=(6.0, 0.6, 0.0))
    goals = mgp.MapGoalProvider(route, arrival_radius_m=0.25, pose_max_age_s=1e12)
    goals.set_pose(mgp.MapPose(5.0, 5.0, 0.0, 1e9))       # far from every waypoint

    args = FakeArgs()
    args.no_deliver = not deliver
    runner = mp.MissionRunner(
        nav=FakeNav(ncfg, align_ok=align_ok), controller=FakeController(grasp_ok=grasp_ok),
        goals=goals, rio=FakeRos(goals), odom_pub=FakeOdomPub(), lidar=FakeLidar(),
        fsm=mfsm.MissionFSM(mfsm.MissionConfig(approach_to_align_m=args.nav_stop_dist),
                            deliver_enabled=deliver),
        args=args)
    runner.started = True
    return runner


def _step_map_pose(runner, t, step=0.4):
    """Move the robot in the MAP frame toward whatever goal it is driving to.

    The nav stub shrinks the camera distance to the object; this is the other
    half of the world -- without it the robot never arrives at a waypoint or at
    the bin, and the mission stalls in DELIVER for reasons that have nothing to
    do with the code under test.
    """
    pose = runner.goals.pose
    gx, gy, _yaw, _label = runner.goals.target()
    dx, dy = gx - pose.x, gy - pose.y
    d = math.hypot(dx, dy)
    if d <= 1e-9:
        return
    k = min(step, d) / d
    runner.goals.set_pose(mgp.MapPose(pose.x + dx * k, pose.y + dy * k,
                                      math.atan2(dy, dx), t))


def drive(runner, max_ticks=600):
    """Step the real FSM + real _act until PATROL is reached twice."""
    seen = []
    t = 1000.0
    laps_at_patrol = 0
    for _ in range(max_ticks):
        t += 0.2
        sense = runner._sense(t)
        tr = runner.fsm.step(sense)
        if not seen or seen[-1] is not tr.state:
            seen.append(tr.state)
            if tr.state is mfsm.State.PATROL:
                laps_at_patrol += 1
                if laps_at_patrol == 2:
                    return seen
        if tr.terminal or tr.state is mfsm.State.PAUSED:
            return seen
        runner._act(tr, t, 0.2)
        if tr.chassis_allowed and tr.state in mfsm.MAP_STATES:
            _step_map_pose(runner, t)
    return seen


# ════════════════════════════════════════════════════════════════════════════
# Tests
# ════════════════════════════════════════════════════════════════════════════

class MissionEndToEnd(unittest.TestCase):

    def _run(self, **kw):
        runner = build_runner(**kw)
        with redirect_stdout(io.StringIO()):
            states = drive(runner)
        return runner, states

    def test_full_cycle_reaches_every_state_and_returns_to_patrol(self):
        runner, states = self._run()
        S = mfsm.State
        for want in (S.SELF_CHECK, S.IDLE, S.PATROL, S.INVESTIGATE, S.APPROACH,
                     S.ALIGN, S.STATIONARY_GATE, S.LATCH, S.GRASP, S.VERIFY,
                     S.CARRY_HOME, S.DELIVER, S.PLACE_ALIGN, S.PLACE, S.RESUME):
            self.assertIn(want, states, f"{want.value} was never reached: {states}")
        self.assertIs(states[-1], S.PATROL, f"did not return to patrol: {states}")
        self.assertNotIn(S.PAUSED, states)
        self.assertNotIn(S.FAULT, states)

    def test_the_order_is_the_documented_one(self):
        _, states = self._run()
        S = mfsm.State
        order = [s for s in states if s in (S.PATROL, S.INVESTIGATE, S.APPROACH,
                                            S.ALIGN, S.GRASP, S.DELIVER, S.PLACE)]
        self.assertEqual(order, [S.PATROL, S.INVESTIGATE, S.APPROACH, S.ALIGN,
                                 S.GRASP, S.DELIVER, S.PLACE, S.PATROL], states)

    def test_grasp_and_release_each_run_exactly_once(self):
        runner, _ = self._run()
        self.assertEqual(runner.controller.ran_grasp, 1)
        self.assertEqual(runner.controller.ran_release, 1)

    def test_arm_is_folded_for_driving_and_raised_only_at_the_handoff(self):
        runner, _ = self._run()
        moves = runner.controller.moves
        self.assertIn("nav-home", moves, "the self-check never parked the arm")
        self.assertIn("nav-home->C3", moves, "the arm never raised to the grasp pose")
        self.assertLess(moves.index("nav-home"), moves.index("nav-home->C3"),
                        "raised to C3 before it was ever parked")
        # exactly one raise: mid-mission pose changes are how the arm ends up
        # somewhere nobody expects
        self.assertEqual(moves.count("nav-home->C3"), 1, moves)

    def test_arm_camera_is_not_consulted_while_the_arm_is_folded(self):
        # Its extrinsics belong to C3 and the ground model returns a plausible
        # number at any pose, so a consult here is silently wrong.
        runner, _ = self._run()
        self.assertEqual(runner.nav.arm_detects, 0, "arm cam used at the nav pose")

    def test_goal_source_switches_keep_the_camera_fix_but_clear_the_delay(self):
        runner, _ = self._run()
        # PATROL->INVESTIGATE must reset with clear_tracker False, DELIVER True
        self.assertIn(False, runner.nav.reset_calls,
                      "no reset preserved the tracker; approach can never start")
        self.assertIn(True, runner.nav.reset_calls,
                      "no reset cleared the tracker on the way to a map goal")

    def test_delivery_targets_the_bin_not_a_patrol_waypoint(self):
        runner, _ = self._run()
        # after PLACE the override is released; the recorded interrupt proves the
        # bin was actually selected at some point
        self.assertFalse(runner.goals.in_override)
        self.assertGreaterEqual(runner.goals.index, 0)
        self.assertEqual(runner.controller.ran_release, 1)

    def test_failed_alignment_gives_up_after_the_retry_budget(self):
        runner, states = self._run(align_ok=False)
        S = mfsm.State
        self.assertIn(S.RETRY, states, states)
        self.assertIn(S.RESUME, states, states)
        self.assertIs(states[-1], S.PATROL, f"did not resume patrol: {states}")
        self.assertEqual(runner.controller.ran_grasp, 0, "grasped despite bad align")
        self.assertTrue(runner._blacklist, "gave up without blacklisting the spot")

    def test_no_deliver_returns_to_patrol_without_the_bin(self):
        runner, states = self._run(deliver=False)
        S = mfsm.State
        self.assertIn(S.CARRY_HOME, states, states)
        self.assertNotIn(S.DELIVER, states, states)
        self.assertNotIn(S.PLACE, states, states)
        self.assertIs(states[-1], S.PATROL, states)
        self.assertEqual(runner.controller.ran_release, 0)

    def test_state_is_clean_when_patrol_resumes(self):
        # A stale latch makes the NEXT object's LATCH transition straight to
        # GRASP with the previous coordinates.
        runner, _ = self._run()
        self.assertIsNone(runner._latched)
        self.assertFalse(runner._align_failed)
        self.assertFalse(runner._handoff_ready)
        self.assertFalse(runner._grasp_finished)
        self.assertEqual(runner._det_streak, 0)
        self.assertTrue(runner._at_nav_home, "patrol resumed with the arm at C3")


if __name__ == "__main__":
    unittest.main(verbosity=2)
