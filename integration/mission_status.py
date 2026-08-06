#!/usr/bin/env python3
"""Publish what the mission is doing, for anything that wants to watch.

The mission already knows why it is doing what it is doing -- every FSM
transition carries a reason string -- but that knowledge only ever reached a
terminal.  This turns each tick into a JSON datagram so an operator console can
show it without reading the log over someone's shoulder.

Sent over UDP to localhost, deliberately:

  * The mission must never block on a watcher.  A UDP send to a socket nobody
    is reading returns immediately; a pipe or a TCP socket would fill up and
    stall the control loop, which is the one thing telemetry must not do.
  * If the console is not running, the datagrams evaporate.  No file grows on
    the Jetson's SD card, no reader has to be started first, no cleanup.
  * Dropping a frame costs nothing.  This is a 10 Hz repeating snapshot, not an
    event log -- the next one is 100 ms away and carries the whole state again.

Nothing here can raise into the caller.  A telemetry bug must not be able to
stop a robot that is holding an object.

    python3 integration/mission_status.py --selftest
    python3 integration/mission_status.py --listen 8099    # watch the stream
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from typing import Any, Dict, Optional

SCHEMA_VERSION = 1
DEFAULT_ENDPOINT = "127.0.0.1:8099"
# Every state change is published the moment it happens -- that is the
# information.  Between changes this is a heartbeat carrying a slowly moving
# position, and the control loop runs at 10 Hz, so publishing every tick would
# spend ten dict builds and ten json.dumps a second to redraw a number that
# barely moved.  On a Jetson Nano running YOLO and a policy at 90% memory, that
# is not free.  Half a second is well inside "roughly where is it".
DEFAULT_PERIOD_S = 0.5
# A datagram larger than the loopback MTU gets fragmented and is more likely to
# be dropped.  Every field below is small; this is a guard against a reason
# string that grew unexpectedly, not a routine limit.
MAX_DATAGRAM_BYTES = 1400


def parse_endpoint(text: str) -> tuple:
    """"host:port" -> ("host", port).  Raises ValueError with a usable message."""
    if ":" not in text:
        raise ValueError("expected HOST:PORT, e.g. %s" % DEFAULT_ENDPOINT)
    host, _, port = text.rpartition(":")
    if not host:
        raise ValueError("missing host in %r" % text)
    try:
        n = int(port)
    except ValueError:
        raise ValueError("port %r is not a number" % port)
    if not 1 <= n <= 65535:
        raise ValueError("port %d is out of range" % n)
    return host, n


def _f(value: Any) -> Optional[float]:
    """A float JSON can carry, or None.

    ``target_dist`` is ``inf`` until something is seen and ``json.dumps``
    happily writes bare ``Infinity``, which is not JSON and which every strict
    parser on the receiving end rejects.  One rejected frame would blank the
    whole console, so the conversion happens here rather than being trusted to
    the reader.
    """
    if value is None:
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


class StatusEmitter:
    """Fire-and-forget publisher for one mission run."""

    def __init__(self, endpoint: Optional[str] = None,
                 period: float = DEFAULT_PERIOD_S):
        self.seq = 0
        self.lifted = 0
        self.period = max(0.0, float(period))
        self.skipped = 0
        self._sock: Optional[socket.socket] = None
        self._addr: Optional[tuple] = None
        self._prev_state = ""
        self._last_sent = 0.0
        self._warned = False
        if endpoint:
            self._addr = parse_endpoint(endpoint)
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setblocking(False)

    @property
    def enabled(self) -> bool:
        return self._sock is not None

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    # ── building ──
    def frame(self, tr, sense=None, fix=None, pose=None, *,
              waiting: str = "", laps: int = 0, extra: Optional[Dict] = None) -> Dict:
        """One tick as a plain dict.  Safe to call whether or not sending is on."""
        self.seq += 1
        state = getattr(tr.state, "value", str(tr.state))
        action = getattr(tr.action, "value", str(tr.action))

        # CARRY_HOME is only ever entered after VERIFY confirmed the object left
        # the floor, so counting entries to it counts objects actually lifted --
        # not grasps attempted, and not objects delivered.
        if state == "CARRY_HOME" and self._prev_state != "CARRY_HOME":
            self.lifted += 1
        self._prev_state = state

        f: Dict[str, Any] = {
            "v": SCHEMA_VERSION,
            "seq": self.seq,
            "t": time.time(),
            "state": state,
            "action": action,
            "reason": tr.reason,
            "changed": bool(tr.changed),
            "chassis_allowed": bool(tr.chassis_allowed),
            "arm_allowed": bool(tr.arm_allowed),
            "terminal": bool(tr.terminal),
            "waiting": waiting or None,
            "laps": int(laps),
            "lifted": self.lifted,
        }

        if sense is not None:
            f["target"] = {
                "visible": bool(sense.target_visible),
                "dist": _f(sense.target_dist),
                "streak": int(sense.detection_streak),
                "age": _f(sense.target_fix_age),
            }
            f["grasp"] = {
                "latched": bool(sense.latched),
                "finished": bool(sense.grasp_finished),
                "verified": bool(sense.grasp_verified),
                "handoff_ready": bool(sense.handoff_ready),
                "align_failed": bool(sense.align_failed),
            }
            f["base"] = {
                "stationary": bool(sense.stationary),
                "arm_at_home": bool(sense.arm_at_home),
            }
            # The health object is what turns "it stopped" into "it stopped
            # because /scan went stale", so carry the reason the FSM would use.
            try:
                f["blocking"] = sense.health.blocking_reason(tr.state) or None
            except Exception:
                f["blocking"] = None

        if fix is not None:
            f["goal"] = {"id": getattr(fix, "goal_id", ""),
                         "dist": _f(getattr(fix, "dist", None)),
                         "bearing": _f(getattr(fix, "bearing", None))}
        if pose is not None:
            f["pose"] = {"x": _f(pose.x), "y": _f(pose.y), "yaw": _f(getattr(pose, "yaw", None))}

        if extra:
            f.update(extra)
        return f

    # ── sending ──
    def send(self, payload: Dict) -> bool:
        """Push one frame.  Returns whether it left the machine; never raises."""
        if self._sock is None or self._addr is None:
            return False
        try:
            data = json.dumps(payload, allow_nan=False,
                              ensure_ascii=False).encode("utf-8")
        except (TypeError, ValueError):
            return False
        if len(data) > MAX_DATAGRAM_BYTES:
            # Rather than dropping the frame, drop the field most likely to be
            # long.  The state and reason matter more to a watching human than
            # the goal id does.
            payload = dict(payload)
            payload["reason"] = str(payload.get("reason", ""))[:200]
            try:
                data = json.dumps(payload, allow_nan=False,
                                  ensure_ascii=False).encode("utf-8")
            except (TypeError, ValueError):
                return False
            if len(data) > MAX_DATAGRAM_BYTES:
                return False
        try:
            self._sock.sendto(data, self._addr)
            return True
        except OSError as exc:
            # ECONNREFUSED arrives here when no console is listening, which is
            # the normal case, not an error.  Say it once so a genuinely
            # misconfigured endpoint is still visible, then stay quiet.
            if not self._warned:
                self._warned = True
                print("[status] telemetry not delivered (%s); the mission is "
                      "unaffected" % exc)
            return False

    def due(self, changed: bool, now: Optional[float] = None) -> bool:
        """Whether this tick is worth publishing.

        A state change always is: it is the one thing a watching human needs
        immediately, and it is also what carries the reason the robot did
        something.  Everything else is a heartbeat and can wait.
        """
        if changed:
            return True
        return ((now or time.time()) - self._last_sent) >= self.period

    def publish(self, tr, sense=None, fix=None, pose=None, **kw) -> Optional[Dict]:
        """Build and send in one call, unless this tick is being skipped.

        The throttle is checked before the frame is built, not after.  Building
        it and then dropping it would cost the dict and the encode anyway, which
        is most of what there was to save.
        """
        if not self.enabled:
            return None
        now = time.time()
        if not self.due(bool(tr.changed), now):
            self.skipped += 1
            return None
        self._last_sent = now
        f = self.frame(tr, sense, fix, pose, **kw)
        self.send(f)
        return f


class _Step:
    """A transition-shaped object for pipelines that have no state machine."""

    __slots__ = ("state", "action", "reason", "changed",
                 "chassis_allowed", "arm_allowed", "terminal")

    def __init__(self, state, action, reason, changed, driving, arm, terminal):
        self.state = state
        self.action = action
        self.reason = reason
        self.changed = changed
        self.chassis_allowed = driving
        self.arm_allowed = arm
        self.terminal = terminal


class SimpleReporter:
    """Telemetry for the pipelines that are a sequence of steps, not an FSM.

    Modes B and C run straight through -- approach, latch, grasp, verify -- with
    no transition table to report.  They still get to appear on the console, and
    they do it by naming the same states the mission FSM uses rather than
    inventing a second vocabulary the operator would have to learn.  Passing a
    name the FSM does not define is a mistake worth hearing about, so it is
    checked rather than accepted quietly.
    """

    def __init__(self, endpoint: Optional[str], mode: str = ""):
        self.emitter = StatusEmitter(endpoint)
        self.mode = mode
        self._prev = ""
        self._driving, self._arm, self._known = self._state_sets()

    @staticmethod
    def _state_sets():
        # Imported here, not at module scope: this file is deliberately free of
        # project imports so a pipeline can publish telemetry without dragging
        # the FSM in.  Modes B and C already have it loaded anyway.
        try:
            import mission_fsm as mfsm
            return ({s.value for s in mfsm.DRIVING_STATES},
                    {s.value for s in mfsm.ARM_STATES},
                    {s.value for s in mfsm.State})
        except Exception:
            return set(), set(), set()

    @property
    def enabled(self) -> bool:
        return self.emitter.enabled

    def say(self, state: str, action: str, reason: str = "", **extra) -> Optional[Dict]:
        if not self.emitter.enabled:
            return None
        if self._known and state not in self._known:
            raise ValueError(
                "%r is not a mission state; the console has no wording for it. "
                "Use one of the names in mission_fsm.State." % state)
        step = _Step(state, action, reason, state != self._prev,
                     state in self._driving, state in self._arm,
                     state in ("FAULT", "ESTOP"))
        self._prev = state
        payload = {"mode": self.mode} if self.mode else {}
        payload.update(extra)
        return self.emitter.publish(step, extra=payload)

    def close(self) -> None:
        self.emitter.close()


# ════════════════════════════════════════════════════════════════════════════
# Tools
# ════════════════════════════════════════════════════════════════════════════

def listen(port: int, host: str = "127.0.0.1") -> int:
    """Print frames as they arrive.  Handy without the console running."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    print("listening on %s:%d — Ctrl+C to stop" % (host, port))
    try:
        while True:
            data, _ = s.recvfrom(65535)
            try:
                f = json.loads(data.decode("utf-8"))
            except ValueError:
                print("  (unparseable frame, %d bytes)" % len(data))
                continue
            print("  %-16s %-16s %s" % (f.get("state", "?"), f.get("action", "?"),
                                        f.get("reason", "")))
    except KeyboardInterrupt:
        return 0
    finally:
        s.close()


def run_selftest() -> None:
    import dataclasses

    print("== mission status selftest ==")

    @dataclasses.dataclass
    class _Tr:
        state: str
        action: str
        reason: str = ""
        changed: bool = False
        chassis_allowed: bool = False
        arm_allowed: bool = False
        terminal: bool = False

    # endpoint parsing
    assert parse_endpoint("127.0.0.1:8099") == ("127.0.0.1", 8099)
    for bad in ("8099", "host:", "host:0", "host:70000", ":9"):
        try:
            parse_endpoint(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("accepted bad endpoint %r" % bad)
    print("  endpoint parsing rejects the malformed forms")

    # A disabled emitter still builds frames, so the mission can log locally
    # even when nothing is subscribed.
    e = StatusEmitter(None)
    assert not e.enabled
    f = e.frame(_Tr("PATROL", "DRIVE_PATROL", "patrolling"))
    assert f["state"] == "PATROL" and f["seq"] == 1
    assert e.send(f) is False
    print("  disabled emitter builds frames and drops sends")

    # inf must not reach the wire
    class _Sense:
        target_visible = False
        target_dist = float("inf")
        detection_streak = 0
        target_fix_age = float("nan")
        latched = grasp_finished = grasp_verified = False
        handoff_ready = align_failed = False
        stationary = arm_at_home = True

        class health:
            @staticmethod
            def blocking_reason(_):
                return ""
    f = e.frame(_Tr("IDLE", "WAIT_OPERATOR"), _Sense())
    assert f["target"]["dist"] is None and f["target"]["age"] is None
    json.dumps(f, allow_nan=False)          # would raise on inf/nan
    print("  infinite and NaN measurements become null, so the frame stays JSON")

    # lifted counts entries into CARRY_HOME, not ticks spent there
    e2 = StatusEmitter(None)
    for state in ("GRASP", "VERIFY", "CARRY_HOME", "CARRY_HOME", "DELIVER",
                  "PATROL", "GRASP", "VERIFY", "CARRY_HOME"):
        e2.frame(_Tr(state, "STOP"))
    assert e2.lifted == 2, e2.lifted
    print("  lifted counts objects, not ticks (%d)" % e2.lifted)

    # a real send lands in a real socket
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    e3 = StatusEmitter("127.0.0.1:%d" % port)
    assert e3.enabled
    e3.publish(_Tr("GRASP", "RUN_GRASP", "stage 1 · gates passed", changed=True))
    rx.settimeout(2.0)
    got = json.loads(rx.recv(65535).decode("utf-8"))
    assert got["state"] == "GRASP" and got["reason"].startswith("stage 1")
    assert got["v"] == SCHEMA_VERSION
    rx.close()
    e3.close()
    print("  a frame sent over the loopback arrives intact")

    # an over-long reason is trimmed rather than dropping the frame
    e4 = StatusEmitter(None)
    big = e4.frame(_Tr("FAULT", "STOP", "x" * 4000))
    e4._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    e4._sock.setblocking(False)
    e4._addr = ("127.0.0.1", 9)             # discard port
    assert e4.send(big) is True
    e4.close()
    print("  an oversized reason is trimmed instead of losing the frame")

    # Throttling: state changes always go, heartbeats are rate limited
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    e5 = StatusEmitter("127.0.0.1:%d" % port, period=10.0)
    sent = sum(1 for _ in range(50)
               if e5.publish(_Tr("PATROL", "DRIVE_PATROL", "patrolling")) is not None)
    assert sent == 1, "50 unchanged ticks should collapse to one frame, got %d" % sent
    assert e5.skipped == 49, e5.skipped
    print("  50 unchanged ticks publish once (%d skipped)" % e5.skipped)

    # ...but a change is never withheld, however recently we published.
    got = e5.publish(_Tr("GRASP", "RUN_GRASP", "gates passed", changed=True))
    assert got is not None and got["state"] == "GRASP"
    print("  a state change is published immediately regardless of the throttle")

    # period=0 keeps every tick, for anyone who wants the old behaviour
    e6 = StatusEmitter("127.0.0.1:%d" % port, period=0.0)
    assert all(e6.publish(_Tr("PATROL", "DRIVE_PATROL")) is not None for _ in range(5))
    print("  --status-period 0 restores every-tick publishing")
    e5.close(); e6.close(); rx.close()

    # SimpleReporter: modes B and C must speak the FSM's vocabulary
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    rep = SimpleReporter("127.0.0.1:%d" % port, mode="C")
    rep.say("APPROACH", "DRIVE_TARGET", "rl nav running", target={"dist": 1.2})
    rx.settimeout(2.0)
    got = json.loads(rx.recv(65535).decode("utf-8"))
    assert got["state"] == "APPROACH" and got["mode"] == "C"
    assert got["chassis_allowed"] is True and got["arm_allowed"] is False
    assert got["target"]["dist"] == 1.2
    rep.say("GRASP", "RUN_GRASP", "controller.run()")
    got = json.loads(rx.recv(65535).decode("utf-8"))
    assert got["arm_allowed"] is True and got["chassis_allowed"] is False
    print("  SimpleReporter derives the drive/arm gates from the real FSM sets")

    try:
        rep.say("WANDERING", "DRIVE_PATROL")
    except ValueError:
        print("  an invented state name is refused, not silently published")
    else:
        raise AssertionError("accepted a state the console cannot describe")
    rep.close()
    rx.close()

    print("== all mission status selftests passed ==")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--listen", type=int, metavar="PORT",
                    help="print frames arriving on this UDP port")
    args = ap.parse_args()
    if args.selftest:
        run_selftest()
        return 0
    if args.listen:
        return listen(args.listen)
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
