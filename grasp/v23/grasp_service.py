#!/usr/bin/env python3
"""Resident grasp service: pay the 23-second startup once, then grasp on command.

Measured 2026-09-20 on the robot: a one-command grasp takes 42 s wall, of which
34 s is startup the robot spends motionless -- torch and PyBullet imports, the
URDF load (11.5 s by itself), the PPO weights, then YOLO and the camera. Only
about 8 s is the arm moving. Keeping the process alive removes all 34 s.

**It deliberately does not reimplement any of the launcher's setup.** The
controller's configuration lives inside its own ``main()``, behind a hundred
lines of validation -- the sha256 check, the contract check, the homography
gate, the candidate unlock, the E1 pose stamp. Copying that here would mean
maintaining a second copy of the safety gates, and a second copy is how they
drift apart. So this service:

  1. asks ``jetson_one_command_grasp`` to build the exact controller argv it
     would have used (``build_ctrl_cmd``), then
  2. replaces ``GraspController.run`` with a serve loop, and
  3. calls the controller's own ``main()``.

Every gate therefore runs exactly as it does today, at startup, and the first
time ``main()`` reaches ``run()`` we take over the process instead of grasping
once and exiting. Each command then calls the *original* ``run()``, which the
controller documents as safe to call repeatedly: "every piece of episode state
is reset at the top of this method".

The vision side is not touched at all: run ``vision_grasp_bridge.py`` without
``--once`` so a fresh detection is always waiting, and the controller latches it
at the home pose through the same socket path it already uses.

  start:   python3 grasp_service.py [any jetson_one_command_grasp flags]
  drive:   python3 graspctl.py grasp

G2: with GRASP_SERVICE_CHASSIS_PORT=7000 in the environment the service also
drives the wheels over the TCP velocity protocol (chassis_server.py), so the
robot can approach, grasp and drive away without handing /dev/myserial to
another process. Unset, the service is exactly the arm-only one validated on
2026-09-25.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_SOCKET = "/tmp/grasp_service.sock"
ARM_COMMANDS = ("grasp", "home", "release")


def rss_mb() -> float:
    """Resident size of this process, for watching the loop leak (or not)."""
    try:
        with open("/proc/self/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return -1.0


class GraspService:
    """Accepts one client at a time and runs one episode per `grasp` command.

    Single-threaded on purpose: there is one arm, and an episode that overlaps
    another would fight it for the servo bus.
    """

    def __init__(self, controller, run_episode, max_steps: int, sock_path: str,
                 chassis_server=None):
        self.controller = controller
        self._run_episode = run_episode
        self.max_steps = max_steps
        self.sock_path = sock_path
        self.chassis_server = chassis_server
        self.chassis = chassis_server.chassis if chassis_server is not None else None
        self.stop_requested = False
        self.episodes = 0
        self.last = None
        self.started = time.time()

    # ── detection ───────────────────────────────────────────────────────
    def detection(self):
        """(pos, height, fresh) from the controller's own receiver, or None.

        The receiver only exists on the --socket path, which is the one the
        launcher builds, but a caller could configure a fixed target instead.
        """
        recv = getattr(self.controller, "detection", None)
        if recv is None:
            return None
        pos, height, fresh, _stamp = recv.snapshot()
        return {"pos": [round(float(v), 4) for v in pos],
                "height": height, "fresh": bool(fresh)}

    def wait_for_detection(self, timeout_s: float):
        """Wait briefly for a fresh E1 detection.

        Returns the detection, or None when one never arrived. A controller
        configured with a fixed --obj-x/y/z has no receiver at all; that is not
        a missing detection, so `detection()` returning None means "not my
        business" and the caller proceeds.

        Checked BEFORE the episode starts, because `run()` would otherwise walk
        the arm to the home pose and sit there for the whole --latch-wait (120 s
        by default) waiting for a detection that nobody is sending. Refusing up
        front costs nothing and leaves the arm where it is.
        """
        deadline = time.time() + timeout_s
        while True:
            det = self.detection()
            if det is None or det["fresh"]:
                return det
            if time.time() >= deadline:
                return None
            time.sleep(0.2)

    # ── commands ────────────────────────────────────────────────────────
    def cmd_status(self) -> dict:
        jaw = None
        try:
            rd = self.controller.servo.read_degrees()
            jaw = rd.degrees if rd.valid else None
        except Exception as exc:                      # diagnostics must not kill the service
            jaw = "read failed: %s" % exc
        return {"ok": True, "episodes": self.episodes, "last": self.last,
                "servo_deg": jaw, "detection": self.detection(),
                "chassis": self.chassis.status() if self.chassis else None,
                "rss_mb": round(rss_mb(), 1),
                "uptime_s": round(time.time() - self.started, 1)}

    def run_arm_command(self, fn) -> dict:
        """R1/R2 around one arm command; a plain call when the chassis is off.

        The wheels are claimed before anything else, the detection wait
        included: the grasp latches what the camera sees, so the robot has to
        stand still from that moment on, not only once the arm starts moving.
        """
        if self.chassis is None:
            return fn()
        why = self.chassis.begin_arm()
        if why is not None:
            return {"ok": False, "reason": why,
                    "hint": "the wheels are turning or stopped less than 0.5 s ago; "
                            "the arm was not moved",
                    "chassis": self.chassis.status()}
        try:
            return fn()
        finally:
            self.chassis.end_arm()

    def cmd_home(self) -> dict:
        """Park at the grasp home pose with the jaw open, releasing anything held.

        Without this the only way to put the object back on the table is to stop
        the service, run move_arm.py and start it again -- a minute of reloading
        PyBullet to open a gripper. Uses the same guarded move run() uses, so the
        floor guard still applies.
        """
        ctl = self.controller
        cfg = ctl.cfg
        arm = ctl.mapper.hw_deg_to_sim_arm(list(cfg.home_deg[:5]))
        grip = ctl.mapper.hw_deg_to_sim_grip(cfg.home_deg[5])
        t0 = time.time()
        try:
            res = ctl.move_guarded_and_verified(
                arm, grip, label="service-home", run_time_ms=400, settle_s=0.35,
                tol_deg=3.0)
        except Exception as exc:
            # A hardware fault must not take the service down with it; the arm
            # holds position while powered, so reporting and staying up is safer
            # than dying and leaving no way to ask what happened.
            return {"ok": False, "reason": "exception: %s" % exc,
                    "elapsed_s": round(time.time() - t0, 2)}
        ctl._grip_hold_rad = None     # nothing is held any more
        return {"ok": bool(res.get("reached")), "reason": res.get("reason"),
                "iters": res.get("iters"), "elapsed_s": round(time.time() - t0, 2)}

    def cmd_release(self) -> dict:
        """Stage 3 on its own: reach forward, open over the bin, come home.

        The controller refuses to open while the pads sit below
        bin_rim_height + release_clearance, and refuses outright when the jaw
        does not look like it is holding anything. Note what the outcome does
        NOT mean: the jaw-stall proxy reads "holding" right up until the jaw is
        commanded open, so "released" says the motion ran, not that the object
        landed in the bin.
        """
        t0 = time.time()
        try:
            outcome = self.controller.run_release_only()
        except Exception as exc:
            return {"ok": False, "outcome": "exception: %s" % exc,
                    "elapsed_s": round(time.time() - t0, 2)}
        return {"ok": outcome == "released", "outcome": outcome,
                "rim_cm": round(self.controller.cfg.bin_rim_height * 100, 1),
                "clearance_cm": round(self.controller.cfg.release_clearance * 100, 1),
                "elapsed_s": round(time.time() - t0, 2),
                "rss_mb": round(rss_mb(), 1)}

    def cmd_grasp(self, wait_s: float = 3.0) -> dict:
        # E1 first: the resident bridge streams a detection every second, so a
        # visible object is already in hand and the episode costs ~8.5 s with no
        # arm rotation at all. Only when E1 sees nothing is the three-pose scan
        # worth its rotations -- and that path stays a separate, manually
        # unlocked flow (graspscan.sh), because a LEFT/RIGHT-only target rests
        # entirely on the S1 rotation the scan config still calls unvalidated.
        if self.detection() is not None and self.wait_for_detection(wait_s) is None:
            return {"ok": False, "reason": "no_fresh_e1_detection",
                    "hint": "nothing visible from E1 in %.0fs — object outside "
                            "the E1 window, or the vision service is down. The "
                            "arm was not moved. Run graspscan.sh to search with "
                            "the three-pose scan." % wait_s,
                    "episodes": self.episodes}
        t0 = time.time()
        self.episodes += 1
        try:
            confirmed = bool(self._run_episode(self.controller,
                                               max_steps=self.max_steps))
            reason = getattr(self.controller, "_outcome", None)
        except Exception as exc:
            confirmed, reason = False, "exception: %s" % exc
        elapsed = time.time() - t0
        self.last = {"confirmed": confirmed, "outcome": reason,
                     "elapsed_s": round(elapsed, 2)}
        # Printed as well as returned: the RSS trend across episodes is the
        # evidence that a long-lived torch + PyBullet process is not leaking.
        print("[service] episode %d: confirmed=%s outcome=%s %.2fs  rss %.0f MB"
              % (self.episodes, confirmed, reason, elapsed, rss_mb()), flush=True)
        return {"ok": True, **self.last, "rss_mb": round(rss_mb(), 1)}

    # ── loop ────────────────────────────────────────────────────────────
    def serve(self) -> None:
        if os.path.exists(self.sock_path):
            os.unlink(self.sock_path)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.sock_path)
        os.chmod(self.sock_path, 0o600)
        srv.listen(1)
        try:
            if self.chassis_server is not None:
                self.chassis_server.start()
            print("\n[service] ready on %s — startup cost is now paid.\n"
                  "[service] commands: grasp | release | home | status | quit"
                  "   chassis: %s   (rss %.0f MB)\n"
                  % (self.sock_path,
                     "TCP %d" % self.chassis_server.port if self.chassis_server
                     else "off", rss_mb()), flush=True)
            while not self.stop_requested:
                conn, _ = srv.accept()
                # A client that connects and never sends a line would otherwise
                # block the only thread that can drive the arm.
                conn.settimeout(10.0)
                try:
                    try:
                        with conn.makefile("r") as stream:
                            line = stream.readline().strip()
                    except (socket.timeout, OSError):
                        continue    # a client that said nothing costs us nothing
                    if not line:
                        continue
                    if line == "quit":
                        conn.sendall(b'{"ok": true, "bye": true}\n')
                        print("[service] quit requested", flush=True)
                        return
                    if line == "status":
                        reply = self.cmd_status()
                    elif line in ARM_COMMANDS:
                        reply = self.run_arm_command(
                            {"home": self.cmd_home, "release": self.cmd_release,
                             "grasp": self.cmd_grasp}[line])
                    else:
                        reply = {"ok": False, "error": "unknown command %r" % line}
                    conn.sendall((json.dumps(reply) + "\n").encode())
                finally:
                    conn.close()
        except KeyboardInterrupt:
            print("\n[service] interrupted — handing back to the controller's "
                  "own shutdown path", flush=True)
        finally:
            # Wheels first: once this process lets go of the port nothing else
            # will stop them.
            if self.chassis_server is not None:
                self.chassis_server.close()
            srv.close()
            if os.path.exists(self.sock_path):
                os.unlink(self.sock_path)


def install_sigterm_shutdown(service) -> None:
    """Make SIGTERM take the Ctrl+C path. Only installed when the wheels are ours.

    systemd stops a unit with SIGTERM, and Python's default for that is to exit
    on the spot, skipping every finally block. With the wheels turning that
    leaves them turning: the board keeps the last speed it was given. The flag
    covers an episode that swallows the KeyboardInterrupt itself -- the loop
    still ends after it instead of waiting for systemd's SIGKILL.
    """
    def on_sigterm(_signum, _frame):
        service.stop_requested = True
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)


def build_chassis_server(controller, port_text: str):
    """The TCP 7000 chassis front end, or None when it is not asked for.

    Fails loudly when it is asked for and cannot start (port taken, mapping
    missing): a service that quietly came up without wheels would leave the
    approach controller talking to nobody.
    """
    if not port_text:
        return None
    import threading
    import chassis_server as cs

    device = controller.servo.device
    if device is None:
        print("[service] chassis requested but there is no Rosmaster device "
              "(dry run) — chassis stays off", flush=True)
        return None
    to_motor, max_motor, watchdog_s = cs.load_motor_mapping(HERE.parents[1])
    cs.install_write_lock(device, threading.Lock())
    chassis = cs.Chassis(device, to_motor, max_motor=max_motor, watchdog_s=watchdog_s,
                         log=lambda msg: print(msg, flush=True))
    return cs.ChassisServer(chassis, port=int(port_text),
                            log=lambda msg: print(msg, flush=True))


def main() -> int:
    sys.path.insert(0, str(HERE))
    import jetson_one_command_grasp as launcher   # noqa: E402
    import x3plus_real_grasp as ctrl              # noqa: E402

    # Every flag goes to the launcher's own parser untouched, so this service can
    # never accept a flag the launcher would have rejected. The one setting that
    # is ours alone travels by environment instead of competing for flag space.
    argv = sys.argv[1:]
    sock_path = os.environ.get("GRASP_SERVICE_SOCKET", DEFAULT_SOCKET)
    chassis_port = os.environ.get("GRASP_SERVICE_CHASSIS_PORT", "").strip()

    launcher_args = launcher.parse_args_from(argv)
    ctrl_cmd = launcher.build_ctrl_cmd(launcher_args)
    # build_ctrl_cmd returns [python, script, *flags]; main() parses the flags.
    ctrl_argv = ctrl_cmd[2:]
    print("[service] controller flags (from the launcher, unmodified):")
    print("[service]   " + " ".join(ctrl_argv), flush=True)

    original_run = ctrl.GraspController.run
    state = {}

    def serving_run(self, max_steps: int = 300):
        """Stands in for the first (and only) run() call main() makes.

        Returning True keeps main() on its success path, so when the loop ends
        the controller closes the serial port and PyBullet exactly as it would
        after a normal run.
        """
        if state.get("serving"):
            # Defensive: a nested call would mean main() looped, which it does
            # not. Fall through to a real episode rather than recursing.
            return original_run(self, max_steps=max_steps)
        state["serving"] = True
        chassis_server = build_chassis_server(self, chassis_port)
        service = GraspService(self, original_run, max_steps, sock_path,
                               chassis_server=chassis_server)
        if chassis_server is not None:
            install_sigterm_shutdown(service)
        service.serve()
        return True

    ctrl.GraspController.run = serving_run
    sys.argv = [str(HERE / "x3plus_real_grasp.py")] + ctrl_argv
    return ctrl.main()


if __name__ == "__main__":
    raise SystemExit(main())
