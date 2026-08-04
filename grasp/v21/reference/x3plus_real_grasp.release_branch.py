#!/usr/bin/env python3
"""Standalone Jetson Nano deployment for X3Plus 6D grasp policy.

No ROS required. Uses PyBullet DIRECT (headless) for forward kinematics only.

Features
--------
- Loads trained weights from the same folder automatically
- TCP socket (port 5555) for object detection input — or defaults to 0.26 m front
- Yahboom Arm_Lib servo control via UART
- 28D observation matching student training env exactly
- Automatic 3-stage grasp management

Usage
-----
    python x3plus_real_grasp.py                     # dry-run, default obj at 0.26 m
    python x3plus_real_grasp.py --real              # real servo mode
    python x3plus_real_grasp.py --real --socket     # real servo + listen for detection

Sim-to-real calibration note (HARDWARE-CALIBRATED 2026-05-25; v18 update 2026-07-17)
-----------------------------
  Rosmaster API command = 90 + sim_deg for EVERY arm joint (arm_hw_invert all
  False). The Rosmaster/App mirrors S2/S3/S4 internally, so the teleop App
  DISPLAYS physical = 180 - API for those joints — do not feed App angles here.
  S6 gripper: API 30° = OPEN, 180° = CLOSED (verified with real grasps).

  ⚠ An earlier revision of this file used S2-inverted mapping and open=180 —
  that convention was a double-mirror artifact and moved the policy backwards.

  All mappings are defined in JointMapper and can be adjusted without touching
  the control logic.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, NamedTuple, Optional, Tuple

import numpy as np

if "KMP_DUPLICATE_LIB_OK" not in os.environ:
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
os.environ.setdefault("OMP_NUM_THREADS", "1")

import pybullet as p

# stable_baselines3 / gymnasium are imported where they are used, not here. A
# release-only run (--release-only) executes a fixed joint sequence and never touches
# the policy, so the machine standing at the bin should not need the training stack
# installed just to import this module. Type annotations referring to VecNormalize are
# safe because of `from __future__ import annotations` above.

import deploy_contract as dc
from action_execution_v21 import ActionExecutionState
from deploy_contract import ContractMismatchError, MissingObjectHeightError

# ── Yahboom Arm_Lib (installed on Jetson Nano by Yahboom SDK) ──────────────
try:
    import Arm_Lib
    _arm_device = Arm_Lib.Arm_Device()
    HAS_ARM_LIB = True
    print("[INFO] Arm_Lib loaded — real servo control available.")
except Exception:
    HAS_ARM_LIB = False
    _arm_device = None
    print("[WARNING] Arm_Lib not found or failed to init. Servo output will be printed only.")


# ═══════════════════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class DeployConfig:
    # ── model paths (relative to this script or absolute) ──
    # v18 (2026-07-17) = "C3 high-FOV grasp-home" model. Episode start = C3 pose
    #   (sim (0,-0.4,-1.4,-1.4,0) = API (90,67.1,9.8,9.8,90)+S6=30): camera at
    #   0.227m sees ground abs x 0.196-0.330m (84% of the graspable band; v17's
    #   low home saw only 0.185-0.274 — objects ~10cm ahead were OUT of frame).
    #   Formal acceptance: 97/100 deterministic, held-out seed, deployable region
    #   x(0.20,0.33) y(±0.10); nominal (0.25,0) 10/10; cracker_box 20/20.
    #   Verify: python training/eval_6d_grasp.py \
    #     --model trained_6d_models_v18/ppo_6d_final_ready_for_real_robot.zip \
    #     --vecnorm trained_6d_models_v18/vecnormalize_6d_final.pkl \
    #     --episodes 30 --randomize --x-lo 0.20 --x-hi 0.33 --y-lo -0.10 --y-hi 0.10
    # ⚠ model 與 VecNormalize 必須成對使用；v18 訓練起點=C3，絕不可拿 v17 權重
    #   從 C3 起跳（OOD），也不可拿 v18 從舊低姿態起跳。
    # ⚠ Stage-2 (return) arm actions are SCRIPTED in sim — on the real robot,
    #   script the return once a stable grasp is detected.
    # Rollback: v17 pair at trained_6d_models_v17/ (its home = API 90,32.7,9.8,32.7,90).
    # No v18 fallback: 28D absolute and 28D incremental models have identical
    # tensor dimensions, so loading the old pair under the v21 contract cannot be
    # detected from the zip. A selected v21 pair must be supplied explicitly.
    model_path: str = "trained_6d_models_v21/SELECTED_MODEL_REQUIRED.zip"
    vecnorm_path: str = "trained_6d_models_v21/SELECTED_VECNORMALIZE_REQUIRED.pkl"
    urdf_path: str = "x3plus/yahboomcar.urdf"

    # ── observation / action contract ──
    # MUST match the generation the weights were trained under. It is not inferable
    # from the .zip: 28D-vs-32D is checked, but absolute-vs-incremental arm semantics
    # are invisible in the file, and decoding one as the other drives the arm to the
    # wrong place at full range. Look it up in the model's manifest.json.
    #   v18, v19 → "obs_28_absolute"
    #   v20       → "obs_32_goal_incremental"  (also requires object height per detection)
    contract_name: str = "obs_28_incremental"

    # Floor plane height in the robot base frame; used for the hover target.
    ground_height: float = 0.0

    # ── floor safety (real robot) ──
    # The sim-side guard can restore a violating state; hardware cannot be teleported
    # back, so on the robot the ONLY workable form is prevention: sweep the candidate
    # path with FK before anything is sent, and shorten or refuse the motion. There is
    # no backstop here by design — if this layer fails, the gripper hits the floor.
    floor_guard_enable: bool = True
    floor_safety_margin: float = 0.008   # m, deliberately looser than sim's 0.005:
                                         # real servos overshoot more than the model
    floor_sweep_samples: int = 8         # points checked along current → target
    floor_bisect_iters: int = 6          # refinement of the largest safe fraction
    emergency_raise_rad: float = 0.05    # per-attempt lift when already below the line
    emergency_raise_max_iters: int = 12  # bound on repeated raise attempts

    # ── scripted stage 2 (no policy, no tcp_pos) ──
    scripted_lift_steps: int = 6
    scripted_lift_rad_per_step: float = 0.05
    # The jaw travels open→closed = 150 hw deg, and send_degrees moves at most
    # max_delta_deg (8) per command, so a full close needs ~19 iterations.
    close_max_iters: int = 40
    # Grasp proxy: this robot has no force/tactile sensor. A jaw closing on nothing
    # reaches the fully-closed stop; one closing on an object stalls short of it.
    # Require it to stall by at least this fraction of the open→closed span before
    # believing an object is held. Proxy only — see _grasp_looks_real().
    grasp_stall_min_fraction: float = 0.06

    # ── scripted stage 3: release into the bin (opt-in, --release-to-bin) ──
    # Off by default: if no bin is in front of the robot, this drops the object on the
    # floor. The bin opening is ~30 cm across and its rim only ~5 cm high, so there is
    # nothing to aim at — reach forward a little, open, come back. No policy involved.
    # The reach uses S2 only. S3/S4 move the gripper forward roughly twice as much per
    # radian, but their sim→hw mapping has never been calibrated against the real arm,
    # so FK there predicts a motion the servos may not make. S2 is the one calibrated
    # against real grasps (2026-05-25), and from the C3 home pose it also RAISES the
    # jaw as it extends — the right way to be wrong next to a 5 cm rim.
    # Measured from home: 6 x 0.05 rad ⇒ +2.4 cm forward, +3.6 cm up, pad 8.4→11.8 cm.
    release_enable: bool = False
    release_extend_steps: int = 6
    release_extend_rad_per_step: float = 0.05
    bin_rim_height: float = 0.05      # m, bin rim above the floor
    release_clearance: float = 0.03   # m, pad bottom must clear the rim by this much
    release_settle_s: float = 0.5     # pause after opening, before retracting

    # ── object detection ──
    # (dry-run/CLI fallback only; real mode must use fresh detections — fail closed.)
    default_obj_pos: Tuple[float, float, float] = (0.26, 0.0, 0.02)
    # Object height (full z extent, metres). None = detection must supply it.
    # Required by obs_32_goal_incremental; must never be silently defaulted.
    default_obj_height: Optional[float] = None
    socket_host: str = "0.0.0.0"
    socket_port: int = 5555

    # ── control timing ──
    control_hz: float = 10.0        # inference rate
    servo_run_time_ms: int = 100    # time for servo to reach target (ms)

    # ── stage thresholds ──
    # Mirrors of the training entry condition (_is_centered_for_grasp):
    #   entry_xy = min(xy_alignment_tolerance 0.035, stage0_entry_xy_tolerance 0.022)
    #   entry_z  = min(z_alignment_tolerance 0.030, stage0_entry_z_tolerance 0.018)
    #   radius   = min(stage0_tolerance_radius 0.040, stage0_entry_tolerance_radius 0.026)
    # Deployment must not close on an easier condition than training, or it closes in
    # states the policy never learned to close in.
    entry_xy_tol: float = 0.022
    entry_z_tol: float = 0.018
    entry_radius: float = 0.026
    stage2_dist_threshold: float = 0.05   # dist to home for "done"

    # ── safety ──
    max_delta_deg: float = 8.0            # max per-step servo change (deg)
    # v18 policy grasp-home = C3 高視野姿態（訓練 reset 與此完全一致，不可只改一邊）。
    #   sim (0,-0.4,-1.4,-1.4,0) → API (90, 67.08, 9.79, 9.79, 90)；S6=30 開爪。
    #   相機高 0.227m、近垂直俯視，地面可視 abs x 0.196-0.330m。
    #   舊值（v17 低姿態，相機只看到 0-7cm 前方）：API (90, 32.7, 9.8, 32.7, 90)。
    # nav→C3 轉場請走 validate_nav_to_grasp_transition.py 輸出的 6 個 waypoint，
    #   不可單段直跳（首次實機需人在旁、手扶急停）。
    home_deg: Tuple[float,...] = (90.0, 67.08, 9.79, 9.79, 90.0, 30.0)

    # ── gripper sim range (must match training env) ──
    gripper_sim_open: float = -1.5   # rad
    gripper_sim_closed: float = 0.0  # rad
    # 實機校正（2026-05-25）：API 30°=張開、180°=閉合。舊版 open=180/closed=30 是
    # 過時的顛倒設定，會讓 policy 一開始就把爪子夾死。
    gripper_hw_open: float = 30.0    # degrees (OPEN)
    gripper_hw_closed: float = 180.0 # degrees (CLOSED)

    # ── arm sim range (URDF limits in radians) ──
    arm_sim_limits: Tuple[Tuple[float, float], ...] = (
        (-1.5708, 1.5708),   # S1
        (-1.5708, 1.5708),   # S2
        (-1.5708, 1.5708),   # S3
        (-1.5708, 1.5708),   # S4
        (-1.5708, 3.14159),  # S5
    )
    # Invert flag for each arm joint: True = hw = center - sim_deg (inverted)
    # 實機校正定案（2026-05-25，實抓驗證）：全部 False，API = 90 + sim_deg。
    # 舊版 S2=True 是「把 App 鏡像顯示當物理角」校出來的雙重翻轉，會讓 S2 反向。
    arm_hw_invert: Tuple[bool, ...] = (False, False, False, False, False)
    arm_hw_center: Tuple[float, ...] = (90.0, 90.0, 90.0, 90.0, 90.0)
    arm_hw_range: Tuple[Tuple[float, float], ...] = (
        (0.0, 180.0),   # S1
        (0.0, 180.0),   # S2
        (0.0, 180.0),   # S3
        (0.0, 180.0),   # S4
        (0.0, 270.0),   # S5
    )


def resolve_object_height_for_contract(
    contract: dc.DeployContract,
    object_pos,
    reported_height: Optional[float],
    ground_height: float,
) -> Tuple[float, str]:
    """Resolve one episode's object height without weakening the 32D contract.

    The 28D observation does not contain height, but deployment still needs height
    for the stage-0 target and finger-pad close gate. If the detector omits it, the
    only geometry available is an explicit resting-object assumption: ``z`` is the
    centroid of a vertically symmetric object resting on the configured floor, hence
    ``height = 2 * (centroid_z - ground_z)``. The value is frozen for the episode.

    The 32D contract observes height directly and therefore never takes this fallback;
    a missing value remains a hard error.
    """
    pos = np.asarray(object_pos, dtype=np.float64)
    if pos.shape != (3,) or not np.all(np.isfinite(pos)):
        raise MissingObjectHeightError(
            "object position must be a finite 3D centroid before height can be resolved"
        )
    ground = float(ground_height)
    if not np.isfinite(ground):
        raise MissingObjectHeightError("ground height must be finite")

    if reported_height is not None:
        height = float(reported_height)
        if not np.isfinite(height) or height <= 0.0:
            raise MissingObjectHeightError(
                f"reported object height must be finite and positive, got {reported_height!r}"
            )
        return height, "reported"

    if contract.requires_object_height:
        raise MissingObjectHeightError(dc.DETECTION_CONTRACT_REQUIRED)

    height = 2.0 * (float(pos[2]) - ground)
    if not np.isfinite(height) or height <= 0.0:
        raise MissingObjectHeightError(
            "28D resting-centroid fallback requires centroid_z > ground_height; "
            f"got centroid_z={float(pos[2]):.6f}, ground_height={ground:.6f}"
        )
    return height, "resting_centroid_geometry"
# ═══════════════════════════════════════════════════════════════════════════
# Joint Mapper — sim radians ↔ hardware degrees
# ═══════════════════════════════════════════════════════════════════════════

class JointMapper:
    def __init__(self, cfg: DeployConfig):
        self.cfg = cfg

    def sim_arm_to_hw_deg(self, sim_angles_rad: np.ndarray) -> List[float]:
        """Convert 5 arm joint angles (radians, sim) → hardware degrees."""
        hw = []
        for i, (rad, invert, center, (hw_lo, hw_hi), (sim_lo, sim_hi)) in enumerate(zip(
            sim_angles_rad,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
            self.cfg.arm_sim_limits,
        )):
            sim_deg = math.degrees(float(rad))
            if invert:
                deg = center - sim_deg
            else:
                deg = center + sim_deg
            hw.append(float(np.clip(deg, hw_lo, hw_hi)))
        return hw

    def sim_grip_to_hw_deg(self, grip_rad: float) -> float:
        """Convert gripper joint angle (radians, sim) → hardware degree."""
        cfg = self.cfg
        grip_rad = float(np.clip(grip_rad, cfg.gripper_sim_open, cfg.gripper_sim_closed))
        ratio = (grip_rad - cfg.gripper_sim_open) / (cfg.gripper_sim_closed - cfg.gripper_sim_open)
        hw = cfg.gripper_hw_open + ratio * (cfg.gripper_hw_closed - cfg.gripper_hw_open)
        return float(np.clip(hw, min(cfg.gripper_hw_open, cfg.gripper_hw_closed),
                                  max(cfg.gripper_hw_open, cfg.gripper_hw_closed)))

    def hw_deg_to_sim_arm(self, hw_degs: List[float]) -> np.ndarray:
        """Convert 5 hardware degrees → sim radians (for observation building)."""
        sim = []
        for i, (deg, invert, center, (hw_lo, hw_hi), (sim_lo, sim_hi)) in enumerate(zip(
            hw_degs,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
            self.cfg.arm_sim_limits,
        )):
            if invert:
                sim_deg = center - deg
            else:
                sim_deg = deg - center
            rad = math.radians(sim_deg)
            sim.append(float(np.clip(rad, sim_lo, sim_hi)))
        return np.array(sim, dtype=np.float32)

    def hw_deg_to_sim_grip(self, hw_deg: float) -> float:
        """Convert hardware gripper degree → sim radians."""
        cfg = self.cfg
        ratio = (hw_deg - cfg.gripper_hw_open) / (cfg.gripper_hw_closed - cfg.gripper_hw_open + 1e-9)
        rad = cfg.gripper_sim_open + ratio * (cfg.gripper_sim_closed - cfg.gripper_sim_open)
        return float(np.clip(rad, cfg.gripper_sim_open, cfg.gripper_sim_closed))

    def norm_action_to_sim_angles(
        self,
        action: np.ndarray,
        current_arm_rads: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, float]:
        """Convert a normalized 6D policy action → (5 arm radians, gripper radian).

        Delegates to ``deploy_contract.decode_arm_action`` so eval and the robot share
        one implementation of the arm semantics. Under the incremental contract the
        action is a delta from ``current_arm_rads``, which is therefore required;
        decoding it as an absolute target instead would command the arm across its full
        range on every step.

        Only dims 0..4 are contract-dependent — the gripper is absolute everywhere.
        """
        contract = dc.get_contract(self.cfg.contract_name)
        if contract.arm_action_mode == "incremental" and current_arm_rads is None:
            raise ContractMismatchError(
                f"contract {contract.name!r} has incremental arm actions but no current "
                "arm pose was supplied to decode the delta against"
            )
        if current_arm_rads is None:
            current_arm_rads = np.zeros(5, dtype=np.float32)

        arm_rads = dc.decode_arm_action(
            contract, action, current_arm_rads, self.cfg.arm_sim_limits
        ).astype(np.float32)
        grip_rad = dc.decode_gripper_action(action)
        return arm_rads, float(grip_rad)


# ═══════════════════════════════════════════════════════════════════════════
# Servo Controller (Yahboom Arm_Lib wrapper)
# ═══════════════════════════════════════════════════════════════════════════

class ServoReadResult(NamedTuple):
    """Result of reading the servos.

    ``valid`` is False when any servo could not be read. Callers must treat an
    invalid read as "the robot's state is unknown" and stop, not as "assume the last
    command was reached" — that assumption is what lets a controller believe it is
    holding an object it never picked up.
    """
    degrees: List[float]
    valid: bool
    reason: str


class ServoController:
    def __init__(self, cfg: DeployConfig, dry_run: bool = False):
        self.cfg = cfg
        self.dry_run = dry_run or (not HAS_ARM_LIB)
        self._last_deg = list(cfg.home_deg)   # [S1..S5, S6]
        # Dry-run models the plant well enough to exercise the state machine, but its
        # readings are fabricated. Anything that treats a reading as evidence about the
        # physical world has to check this.
        self.readings_are_simulated = self.dry_run

    def send_degrees(self, deg6: List[float], run_time_ms: Optional[int] = None) -> bool:
        """Send a 6-DOF servo command with per-step rate limiting.

        Returns True only if the command actually went out. A failed write used to be
        printed and swallowed, which let the caller carry on updating its internal
        pose as though the arm had moved.

        NOTE: the command is rate-limited to ``max_delta_deg`` per call, so a single
        call does NOT reach a distant target. Use
        ``GraspController.move_guarded_and_verified`` for anything that must arrive.

        deg6: [S1, S2, S3, S4, S5, S6] in hardware degrees
        """
        if run_time_ms is None:
            run_time_ms = self.cfg.servo_run_time_ms

        # Rate limiting
        safe_deg = []
        for i, (target, prev) in enumerate(zip(deg6, self._last_deg)):
            delta = float(np.clip(target - prev, -self.cfg.max_delta_deg, self.cfg.max_delta_deg))
            safe_deg.append(float(np.clip(prev + delta, 0.0, 270.0)))

        if self.dry_run:
            self._last_deg = safe_deg[:]
            print(f"[DRY] S1={safe_deg[0]:.1f}° S2={safe_deg[1]:.1f}° "
                  f"S3={safe_deg[2]:.1f}° S4={safe_deg[3]:.1f}° "
                  f"S5={safe_deg[4]:.1f}° S6={safe_deg[5]:.1f}° "
                  f"t={run_time_ms}ms")
            return True

        try:
            _arm_device.Arm_serial_servo_write6_array(
                safe_deg[0], safe_deg[1], safe_deg[2],
                safe_deg[3], safe_deg[4], safe_deg[5],
                run_time_ms,
            )
        except Exception as e:
            # Do NOT advance _last_deg: the command never left, so the arm is still
            # wherever it was, and the rate limiter must keep working from there.
            print(f"[ERROR] Servo write failed: {e}")
            return False
        self._last_deg = safe_deg[:]
        return True

    def read_degrees(self) -> ServoReadResult:
        """Read the servos. Fails closed — no last-command fallback.

        The previous version substituted the last command whenever a read failed,
        which makes an unreachable servo look like a perfectly tracking one. Anything
        downstream that checks "did we arrive?" or "is something in the jaw?" would
        then be reading its own command back.
        """
        if self.dry_run or not HAS_ARM_LIB:
            return ServoReadResult(self._last_deg[:], True, "dry-run (simulated)")
        out, bad = [], []
        for i in range(1, 7):
            try:
                angle = _arm_device.Arm_serial_servo_read(i)
            except Exception as e:
                bad.append(f"S{i}: {e}")
                angle = None
            if angle is None or angle < 0:
                bad.append(f"S{i}: invalid reading {angle!r}")
                out.append(float("nan"))
            else:
                out.append(float(angle))
        if bad:
            return ServoReadResult(out, False, "; ".join(bad))
        return ServoReadResult(out, True, "ok")

    def emergency_stop(self):
        """Immediate stop: hold position rather than sweeping to home.

        Commanding a long move to home during an emergency is the opposite of
        stopping — it sends the arm on an unguarded sweep across the workspace while
        something is already wrong. Freeze instead; the operator decides what next.
        """
        print("[SAFETY] Emergency stop — freezing at the current commanded pose.")
        try:
            self.send_degrees(self._last_deg[:], run_time_ms=200)
        except Exception as e:
            print(f"[SAFETY] Could not send freeze command: {e}")


# ═══════════════════════════════════════════════════════════════════════════
# Forward Kinematics via PyBullet DIRECT
# ═══════════════════════════════════════════════════════════════════════════

class FKComputer:
    """Headless PyBullet instance used only for forward kinematics.

    The TCP this returns is ``gripper_center`` — the midpoint of the two finger-pad
    link COMs — which is what the training env feeds into obs[6:9]. It is NOT the
    arm_link5 COM: that point sits 6.7–8.1 cm below gripper_center depending on pose,
    and using it was the deployment-side TCP mismatch reported from the robot.

    Because the gripper is a parallel linkage, every one of its six joints has to be
    driven from ``grip_rad`` through its multiplier before the pad links are read;
    setting ``grip_joint`` alone leaves the pads at the wrong place entirely.
    """

    def __init__(self, urdf_path: str):
        self.physics_client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self.physics_client)

        urdf_abs = str(Path(__file__).parent / urdf_path)
        if not Path(urdf_abs).exists():
            raise FileNotFoundError(f"URDF not found: {urdf_abs}")

        # Loaded at the training-frame offset, not the origin: iGibson's loader merges
        # links and settles the base, so a bare origin load puts every reported
        # position ~2.2 cm away from what the policy saw in training. See
        # deploy_contract.URDF_TO_TRAINING_FRAME.
        self.body_id = p.loadURDF(
            urdf_abs,
            basePosition=list(dc.URDF_TO_TRAINING_FRAME),
            useFixedBase=True,
            physicsClientId=self.physics_client,
        )

        # Discover joint indices (same suffix-match logic as x3plus_ground_grasp_env.py)
        target_arm = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        target_grip = list(dc.GRIPPER_JOINT_MULTIPLIERS.keys())
        self.name2id = {}
        self.short2full = {}
        for j in range(p.getNumJoints(self.body_id, physicsClientId=self.physics_client)):
            info = p.getJointInfo(self.body_id, j, physicsClientId=self.physics_client)
            full = info[1].decode("utf-8")
            self.name2id[full] = j
            for t in target_arm + target_grip:
                if full.endswith(t):
                    self.short2full[t] = full

        self.arm_indices = [self.name2id[self.short2full[t]] for t in target_arm if t in self.short2full]
        if len(self.arm_indices) != 5:
            raise RuntimeError(f"URDF exposed {len(self.arm_indices)} arm joints, expected 5")

        # (joint index, multiplier) for the whole linkage, driven together from grip_rad
        self.grip_drive = []
        for short, mult in dc.GRIPPER_JOINT_MULTIPLIERS.items():
            full = self.short2full.get(short)
            if full is not None:
                self.grip_drive.append((self.name2id[full], float(mult)))

        self.tcp_link_r = self.name2id.get(self.short2full.get(dc.TCP_RIGHT_JOINT, ""))
        self.tcp_link_l = self.name2id.get(self.short2full.get(dc.TCP_LEFT_JOINT, ""))
        if self.tcp_link_r is None or self.tcp_link_l is None:
            raise RuntimeError(
                f"URDF is missing {dc.TCP_RIGHT_JOINT}/{dc.TCP_LEFT_JOINT}; cannot compute "
                "gripper_center, and falling back to arm_link5 would silently reintroduce "
                "the TCP mismatch. Refusing to run."
            )

        self.ee_link = self.arm_indices[-1]   # arm_joint5 child link — orientation only

        print(f"[FK] PyBullet DIRECT ready. tcp=gripper_center("
              f"{self.tcp_link_r},{self.tcp_link_l}) arm={self.arm_indices} "
              f"grip_linkage={[i for i, _ in self.grip_drive]}")

    def set_base_pose(self, position, orientation) -> None:
        """Move the FK model's base to a measured pose (odometry on the real robot).

        This is NOT cosmetic: the X3Plus sits on a wheeled base that is free to shift,
        and the policy consumes absolute tcp_pos. If the base moves and FK assumes it
        did not, every reported position is wrong by that drift. Measured in sim: 40
        steps of aggressive motion moved the base enough to push observation parity
        from 1.6 mm to 3.4 mm. Either hold the base still while grasping, or feed
        odometry in here.
        """
        p.resetBasePositionAndOrientation(
            self.body_id, list(position), list(orientation),
            physicsClientId=self.physics_client)

    def _set_state(self, arm_rads: np.ndarray, grip_rad: float) -> None:
        for idx, rad in zip(self.arm_indices, arm_rads):
            p.resetJointState(self.body_id, idx, float(rad), physicsClientId=self.physics_client)
        for idx, mult in self.grip_drive:
            p.resetJointState(self.body_id, idx, float(grip_rad) * mult,
                              physicsClientId=self.physics_client)

    def _gripper_center(self) -> np.ndarray:
        pos_r = p.getLinkState(self.body_id, self.tcp_link_r, computeForwardKinematics=True,
                               physicsClientId=self.physics_client)[0]
        pos_l = p.getLinkState(self.body_id, self.tcp_link_l, computeForwardKinematics=True,
                               physicsClientId=self.physics_client)[0]
        return (np.array(pos_r, dtype=np.float64) + np.array(pos_l, dtype=np.float64)) / 2.0

    def compute(self, arm_rads: np.ndarray, grip_rad: float) -> Tuple[np.ndarray, np.ndarray]:
        """Set joint states and return (gripper_center, arm_link5 quaternion xyzw).

        No stepSimulation: resetJointState + computeForwardKinematics is a pure
        kinematic query, whereas stepping would let gravity perturb the pose between
        the reset and the read.
        """
        self._set_state(arm_rads, grip_rad)
        tcp_pos = self._gripper_center().astype(np.float32)
        state = p.getLinkState(
            self.body_id, self.ee_link,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client,
        )
        tcp_quat = np.array(state[1], dtype=np.float32)   # (x, y, z, w)
        return tcp_pos, tcp_quat

    def pad_bottom_z(self, arm_rads: np.ndarray, grip_rad: float) -> float:
        """Lowest finger-pad AABB point, matching training ``_get_pad_bottom_z``."""
        self._set_state(arm_rads, grip_rad)
        bottoms = []
        for idx in (self.tcp_link_r, self.tcp_link_l):
            try:
                bottoms.append(float(p.getAABB(
                    self.body_id, idx, physicsClientId=self.physics_client)[0][2]))
            except Exception:
                continue
        if bottoms:
            return float(min(bottoms))
        return float(self._gripper_center()[2] - dc.GRASP_PAD_OFFSET)

    def min_gripper_link_z(self, arm_rads: np.ndarray, grip_rad: float) -> float:
        """Lowest point of any gripper link at this pose (same metric as training)."""
        self._set_state(arm_rads, grip_rad)
        lows = []
        for idx, _ in self.grip_drive:
            try:
                lows.append(float(p.getAABB(self.body_id, idx,
                                            physicsClientId=self.physics_client)[0][2]))
            except Exception:
                continue
        return float(min(lows)) if lows else float(self._gripper_center()[2])

    def sweep_min_z(self, arm_from, grip_from, arm_to, grip_to, samples: int = 8) -> float:
        """Lowest gripper point along the straight joint-space path from → to.

        Checking only the endpoint is not enough: the arm can dip below the floor
        partway through a move and come back up, and the servo interpolates through
        that path for real. Returns the minimum over the whole sweep.
        """
        a0 = np.asarray(arm_from, dtype=np.float64).reshape(-1)[:5]
        a1 = np.asarray(arm_to, dtype=np.float64).reshape(-1)[:5]
        worst = 1e9
        for k in range(samples + 1):
            t = k / float(samples)
            worst = min(worst, self.min_gripper_link_z(
                a0 + t * (a1 - a0), grip_from + t * (grip_to - grip_from)))
        return float(worst)

    def close_drop(self, arm_rads: np.ndarray, grip_rad: float) -> float:
        """How far gripper_center has already sunk because the jaw is partly closed.

        The hover target is defined in fully-open coordinates, so the controller has to
        add this back or it lifts the arm to "correct" the sink and the pads never reach
        the object. Measured by FK at both grip angles rather than assumed constant.
        """
        self._set_state(arm_rads, grip_rad)
        now_z = float(self._gripper_center()[2])
        self._set_state(arm_rads, dc.GRIPPER_ANGLE_OPEN)
        open_z = float(self._gripper_center()[2])
        self._set_state(arm_rads, grip_rad)   # restore
        return float(max(0.0, open_z - now_z))

    def close(self):
        p.disconnect(self.physics_client)


# ═══════════════════════════════════════════════════════════════════════════
# Object Detection Receiver (TCP socket)
# ═══════════════════════════════════════════════════════════════════════════

class DetectionReceiver:
    """Listen for JSON object-position messages on a TCP socket.

    Expected JSON format:
        {"x": 0.20, "y": 0.00, "z": 0.02, "height": 0.013}

    ``z`` is the object centroid height; ``height`` is its full z extent (top minus
    floor). They are different quantities — ``height`` cannot be derived from ``z``
    without assuming the object is symmetric and floor-resting, so the 32D contract
    treats a missing ``height`` as a hard error rather than guessing.

    The sender (camera node) connects, sends the JSON, and can disconnect.
    The latest received detection is kept until a new one arrives.
    """

    def __init__(self, host: str, port: int, default_pos: Tuple[float, float, float],
                 default_height: Optional[float] = None):
        self._pos = np.array(default_pos, dtype=np.float32)
        self._height = default_height
        self._lock = threading.Lock()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(5)
        self._server.settimeout(1.0)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[DetectionReceiver] Listening on {host}:{port} | default={list(default_pos)}")

    def _loop(self):
        while True:
            try:
                conn, addr = self._server.accept()
                data = b""
                while True:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    data += chunk
                conn.close()
                msg = json.loads(data.decode("utf-8"))
                pos = np.array([msg["x"], msg["y"], msg.get("z", 0.02)], dtype=np.float32)
                # Absent height stays None — the 32D contract then fails closed rather
                # than grasping at a hover height computed from a made-up number.
                height = msg.get("height", None)
                height = float(height) if height is not None else None
                with self._lock:
                    self._pos = pos
                    self._height = height
                print(f"[DetectionReceiver] New detection from {addr}: "
                      f"pos={pos.tolist()} height={height}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[DetectionReceiver] Parse error: {e}")

    def get(self) -> np.ndarray:
        with self._lock:
            return self._pos.copy()

    def get_height(self) -> Optional[float]:
        with self._lock:
            return self._height

    def close(self):
        self._server.close()


# ═══════════════════════════════════════════════════════════════════════════
# Observation Builder
# ═══════════════════════════════════════════════════════════════════════════

class FloorGuard:
    """Preventive floor protection for the real robot.

    The simulation guard restores a safe state after a violating step. That is not
    available on hardware — you cannot teleport a robot that has already driven its
    gripper into the floor. So the only sound form here is *prevention*: before a
    command is sent, sweep the joint-space path from the current pose to the target,
    and if any point along it puts a gripper link below the safety line, shorten the
    motion to the largest safe fraction. If nothing is safe, hold position.

    This layer is the last thing between the policy and the servos, and it has no net
    beneath it. It is deliberately more conservative than the sim guard.
    """

    def __init__(self, cfg: DeployConfig, fk: FKComputer):
        self.cfg = cfg
        self.fk = fk
        self.interventions = 0
        self.refusals = 0

    @property
    def line(self) -> float:
        return float(self.cfg.ground_height + self.cfg.floor_safety_margin)

    def project(self, arm_now, grip_now, arm_target, grip_target):
        """Return (safe_arm_target, safe_grip_target, info).

        Never raises: a guard that throws mid-motion leaves the arm wherever it was.
        """
        if not self.cfg.floor_guard_enable:
            return arm_target, grip_target, {"action": "disabled"}

        a_now = np.asarray(arm_now, dtype=np.float64).reshape(-1)[:5]
        a_tgt = np.asarray(arm_target, dtype=np.float64).reshape(-1)[:5]

        z_now = self.fk.min_gripper_link_z(a_now, grip_now)
        if z_now < self.line:
            # Already too low — the only safe move is up. Which joint direction is
            # "up" is pose-dependent: a fixed S2 -= 0.05 was wrong here, and the same
            # hard-coded sign was already caught once in the scripted lift. Search the
            # candidates with FK and take the one that genuinely gains height.
            self.refusals += 1
            step = float(self.cfg.emergency_raise_rad)
            best, best_z = None, z_now
            for joint in (1, 2, 3):          # S2/S3/S4 carry the vertical motion
                for sign in (+1.0, -1.0):
                    cand = a_now.copy()
                    cand[joint] = float(np.clip(cand[joint] + sign * step,
                                                *self.cfg.arm_sim_limits[joint]))
                    z = self.fk.min_gripper_link_z(cand, grip_now)
                    if z > best_z:
                        best, best_z = cand, z
            if best is None:
                # Nothing gains height from here. Hold still rather than guess — an
                # arbitrary move while already through the floor can only make it worse.
                return (a_now.astype(np.float32), grip_now,
                        {"action": "emergency_hold", "z_now": z_now, "z_after": z_now})
            return (best.astype(np.float32), grip_now,
                    {"action": "emergency_raise", "z_now": z_now, "z_after": best_z})

        z_sweep = self.fk.sweep_min_z(a_now, grip_now, a_tgt, grip_target,
                                      self.cfg.floor_sweep_samples)
        if z_sweep >= self.line:
            return arm_target, grip_target, {"action": "pass", "z_sweep": z_sweep}

        # Largest fraction of the requested motion that keeps the whole sweep safe.
        lo, hi = 0.0, 1.0
        best_a, best_g = a_now.copy(), grip_now
        for _ in range(self.cfg.floor_bisect_iters):
            mid = 0.5 * (lo + hi)
            cand_a = a_now + mid * (a_tgt - a_now)
            cand_g = grip_now + mid * (grip_target - grip_now)
            if self.fk.sweep_min_z(a_now, grip_now, cand_a, cand_g,
                                   self.cfg.floor_sweep_samples) >= self.line:
                best_a, best_g, lo = cand_a, cand_g, mid
            else:
                hi = mid
        self.interventions += 1
        return (best_a.astype(np.float32), float(best_g),
                {"action": "clamped", "fraction": lo, "z_sweep": z_sweep})


class ObsBuilder:
    """Build the observation for the active contract, matching robot_grasp_env.py.

    The layout is owned by ``deploy_contract`` so training-side eval and the robot
    cannot drift apart; see ``DeployContract.slices`` for the authoritative table.
    ``tcp_pos`` is gripper_center (from FKComputer), not arm_link5.
    """

    def __init__(self, fk: FKComputer, mapper: JointMapper,
                 vec_normalize: Optional[VecNormalize],
                 contract: dc.DeployContract):
        self.fk = fk
        self.mapper = mapper
        self.vec_normalize = vec_normalize
        self.contract = contract

    def build(
        self,
        arm_sim_rads: np.ndarray,
        grip_sim_rad: float,
        obj_pos: np.ndarray,
        stage: int,
        prev_action: np.ndarray,
        obj_height: Optional[float] = None,
        wrist_z_offset: Optional[float] = None,
    ) -> np.ndarray:
        tcp_pos, tcp_quat = self.fk.compute(arm_sim_rads, grip_sim_rad)

        # Only measured when the contract needs it — it costs two extra FK queries.
        close_drop = (self.fk.close_drop(arm_sim_rads, grip_sim_rad)
                      if self.contract.requires_object_height else 0.0)

        obs = dc.build_observation(
            self.contract,
            arm_rads=arm_sim_rads,
            grip_rad=grip_sim_rad,
            tcp_pos=tcp_pos,
            tcp_quat=tcp_quat,
            object_pos=obj_pos,
            stage=stage,
            prev_action=prev_action,
            object_height=obj_height,
            close_drop=close_drop,
            wrist_z_offset=wrist_z_offset,
        )

        if self.vec_normalize is not None and self.vec_normalize.norm_obs:
            obs = self.vec_normalize.normalize_obs(obs.reshape(1, -1)).reshape(-1)

        return obs.astype(np.float32)


# ═══════════════════════════════════════════════════════════════════════════
# Mock environment for VecNormalize loading (no PyBullet needed)
# ═══════════════════════════════════════════════════════════════════════════

def _make_mock_env(obs_dim: int):
    """VecNormalize.load needs an env with the right spaces, nothing more."""
    import gymnasium as gym

    class _MockEnv(gym.Env):
        observation_space = gym.spaces.Box(low=-np.inf, high=np.inf,
                                           shape=(obs_dim,), dtype=np.float32)
        action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)

        def step(self, a):
            return np.zeros(obs_dim, dtype=np.float32), 0.0, False, False, {}

        def reset(self, **kw):
            return np.zeros(obs_dim, dtype=np.float32), {}

    return _MockEnv


# ═══════════════════════════════════════════════════════════════════════════
# Main Controller
# ═══════════════════════════════════════════════════════════════════════════

class GraspController:
    def __init__(self, cfg: DeployConfig, real_servo: bool, use_socket: bool,
                 load_policy: bool = True):
        self.cfg = cfg
        self.script_dir = Path(__file__).resolve().parent

        # ── Contract ──────────────────────────────────────────────────────
        self.contract = dc.get_contract(cfg.contract_name)
        print(f"[Init] Contract: {self.contract.name} "
              f"(obs {self.contract.obs_dim}D, arm action {self.contract.arm_action_mode})")

        # ── Load model ────────────────────────────────────────────────────
        # Stage 3 does not consult the policy — it is a fixed joint sequence. Requiring
        # the weights and VecNormalize statistics just to open a gripper would mean
        # shipping them to the machine at the bin for nothing, so a release-only run
        # skips the whole policy stack. Anything that needs an observation is then
        # unavailable by construction rather than silently unnormalised.
        if not load_policy:
            self.model = None
            self.obs_builder = None
            print("[Init] Policy NOT loaded — scripted stages only (release-only run).")
            self._init_hardware(cfg, real_servo, use_socket=False,
                                need_detection=False)
            return

        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

        model_path = self._abs(cfg.model_path)
        vecnorm_path = self._abs(cfg.vecnorm_path)
        print(f"[Init] Loading model: {model_path}")
        self.model = PPO.load(str(model_path), device="cpu")

        # Check the policy against the contract BEFORE touching VecNormalize. A
        # mismatched pair otherwise surfaces as "VecNormalize failed to load (spaces
        # must have the same shape)", which sends you looking at the wrong file.
        dc.validate_model(self.contract, self.model, model_path=str(model_path))

        print(f"[Init] Loading VecNormalize: {vecnorm_path}")
        mock_venv = DummyVecEnv([_make_mock_env(self.contract.obs_dim)])
        # A failed VecNormalize load is fatal: the policy was trained on normalised
        # observations, so feeding it raw ones produces confident nonsense. The old
        # warn-and-continue path silently did exactly that.
        try:
            loaded = VecNormalize.load(str(vecnorm_path), mock_venv)
        except Exception as e:
            raise ContractMismatchError(
                f"{vecnorm_path}: VecNormalize failed to load ({e}). Refusing to run "
                "unnormalised — the policy expects normalised observations."
            ) from e
        vec_norm = VecNormalize(mock_venv, norm_obs=True, norm_reward=False)
        vec_norm.obs_rms = loaded.obs_rms
        vec_norm.training = False
        vec_norm.norm_reward = False

        # Fail closed on any model / VecNormalize / contract disagreement.
        dc.validate_model(
            self.contract, self.model,
            vecnorm_obs_rms=vec_norm.obs_rms,
            model_path=str(model_path), vecnorm_path=str(vecnorm_path),
        )

        self._init_hardware(cfg, real_servo, use_socket, need_detection=True,
                            vec_norm=vec_norm)

    def _init_hardware(self, cfg: DeployConfig, real_servo: bool, use_socket: bool,
                       *, need_detection: bool, vec_norm=None) -> None:
        """FK, mapper, guard, servos and pose state — everything below the policy."""
        # ── FK computer ───────────────────────────────────────────────────
        self.fk = FKComputer(cfg.urdf_path)

        # ── Mapper, obs builder, floor guard ──────────────────────────────
        self.mapper = JointMapper(cfg)
        if vec_norm is not None:
            self.obs_builder = ObsBuilder(self.fk, self.mapper, vec_norm, self.contract)
        self.floor_guard = FloorGuard(cfg, self.fk)

        # ── Servo controller ──────────────────────────────────────────────
        self.servo = ServoController(cfg, dry_run=not real_servo)

        # ── Object detection ──────────────────────────────────────────────
        if use_socket:
            self.detection = DetectionReceiver(cfg.socket_host, cfg.socket_port,
                                               cfg.default_obj_pos, cfg.default_obj_height)
        else:
            self.detection = None
            self._fixed_obj = np.array(cfg.default_obj_pos, dtype=np.float32)
            self._fixed_height = cfg.default_obj_height

        if (need_detection and self.contract.requires_object_height
                and cfg.default_obj_height is None and not use_socket):
            raise MissingObjectHeightError(
                f"contract {self.contract.name!r} needs an object height and none was "
                f"given (--object-height). " + dc.DETECTION_CONTRACT_REQUIRED
            )

        # ── State ─────────────────────────────────────────────────────────
        self._stage = 0
        self._outcome = "not_run"     # confirmed | unverified | dropped | rejected | ...
        self._wrist_z_offset = None   # frozen at first detection; see run()
        self._episode_object_height = None
        self._object_height_source = None
        self._prev_action = np.zeros(6, dtype=np.float32)  # raw, clipped policy action
        self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(list(cfg.home_deg[:5]))
        self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(cfg.home_deg[5])
        self._action_execution = ActionExecutionState()
        self._action_execution.reset(self._current_grip_rad)

        print("[Init] All systems ready.")

    def _abs(self, path: str) -> Path:
        p_abs = Path(path)
        if not p_abs.is_absolute():
            cand = self.script_dir / path
            if not cand.exists():
                # repo 佈局：權重目錄在 training/ 的上一層（Jetson 佈局＝腳本旁，優先）。
                cand = self.script_dir.parent / path
            p_abs = cand
        if not p_abs.exists():
            raise FileNotFoundError(f"File not found: {p_abs}")
        return p_abs

    def _get_obj_pos(self) -> np.ndarray:
        if self.detection is not None:
            return self.detection.get()
        return self._fixed_obj.copy()

    def _get_obj_height(self, obj_pos) -> float:
        """Resolve and freeze the height used by target/gate geometry this episode."""
        if self._episode_object_height is not None:
            return float(self._episode_object_height)

        reported = (self.detection.get_height()
                    if self.detection is not None else self._fixed_height)
        height, source = resolve_object_height_for_contract(
            self.contract, obj_pos, reported, self.cfg.ground_height)
        self._episode_object_height = float(height)
        self._object_height_source = source
        if source == "resting_centroid_geometry":
            print("[Init] Object height absent under 28D contract; using explicit "
                  "resting symmetric-object geometry: "
                  f"height=2*(centroid_z-ground_z)={height:.4f} m")
        return float(height)

    def move_guarded_and_verified(
        self,
        arm_target_rad,
        grip_target_rad: float,
        *,
        label: str,
        run_time_ms: int = 300,
        settle_s: float = 0.35,
        max_iters: int = 40,
        tol_deg: float = 2.0,
    ) -> dict:
        """The one way this controller is allowed to move the arm.

        Every other path — home, stage-1 close, the scripted lift, the return, the
        failure retreat — goes through here. Three things it guarantees that a bare
        ``send_degrees`` call does not:

        1. **It actually arrives.** ``send_degrees`` rate-limits to ``max_delta_deg``
           per call, so one call moves at most 8 deg. Callers used to issue a single
           command for a 40 deg move and then set their internal pose to the target,
           after which every subsequent FK, guard check and observation was computed
           for a pose the arm was nowhere near.
        2. **Every intermediate command is guarded.** The floor guard is applied per
           iteration, against the pose actually read back.
        3. **Internal state follows the encoders, not the command.** If a read fails,
           the move aborts and the state is left untouched.

        Returns a dict with ``reached`` (bool), ``reason``, ``iters``, and the guard
        actions seen. Never raises for hardware trouble — it reports and stops.
        """
        target = np.asarray(arm_target_rad, dtype=np.float64).reshape(-1)[:5]
        guard_actions = []
        last_err = None

        for it in range(max_iters):
            cur_arm = np.asarray(self._current_arm_rads, dtype=np.float64).reshape(-1)[:5]
            cur_grip = float(self._current_grip_rad)

            safe_arm, safe_grip, ginfo = self.floor_guard.project(
                cur_arm, cur_grip, target, grip_target_rad)
            guard_actions.append(ginfo["action"])

            if ginfo["action"] == "emergency_hold":
                # Nothing gains height from here; holding is the safe action, and it
                # is already what the arm is doing. Report and stop.
                return {"reached": False, "reason": "floor guard emergency_hold",
                        "iters": it, "guard": guard_actions, "guard_info": ginfo}

            if ginfo["action"] == "emergency_raise":
                # The raise is a real command and must actually be SENT — returning
                # here without sending left the arm sitting below the floor line while
                # the log claimed an emergency raise had happened.
                hw = self.mapper.sim_arm_to_hw_deg(safe_arm)
                hw.append(self.mapper.sim_grip_to_hw_deg(safe_grip))
                if not self.servo.send_degrees(hw, run_time_ms=run_time_ms):
                    return {"reached": False, "reason": "servo write failed during "
                                                        "emergency raise",
                            "iters": it, "guard": guard_actions}
                time.sleep(settle_s)
                rd = self.servo.read_degrees()
                if not rd.valid:
                    return {"reached": False,
                            "reason": f"servo read failed during emergency raise: {rd.reason}",
                            "iters": it, "guard": guard_actions}
                self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
                self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])
                # Keep raising until clear of the line, but never silently forever.
                if it + 1 >= self.cfg.emergency_raise_max_iters:
                    return {"reached": False,
                            "reason": "emergency raise exhausted its attempts",
                            "iters": it + 1, "guard": guard_actions, "guard_info": ginfo}
                continue

            hw = self.mapper.sim_arm_to_hw_deg(safe_arm)
            hw.append(self.mapper.sim_grip_to_hw_deg(safe_grip))
            if not self.servo.send_degrees(hw, run_time_ms=run_time_ms):
                return {"reached": False, "reason": "servo write failed",
                        "iters": it, "guard": guard_actions}

            time.sleep(settle_s)

            rd = self.servo.read_degrees()
            if not rd.valid:
                # Fail closed: we no longer know where the arm is, so we must not
                # update our idea of it and must not keep commanding.
                return {"reached": False, "reason": f"servo read failed: {rd.reason}",
                        "iters": it, "guard": guard_actions}

            self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
            self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])

            want_hw = self.mapper.sim_arm_to_hw_deg(target)
            want_hw.append(self.mapper.sim_grip_to_hw_deg(grip_target_rad))
            err = max(abs(float(a) - float(b)) for a, b in zip(rd.degrees, want_hw))
            if err <= tol_deg:
                return {"reached": True, "reason": "arrived", "iters": it + 1,
                        "guard": guard_actions, "residual_deg": err}

            # If the guard is clamping, the arm is being deliberately held short of the
            # request — that is not a stall, so only treat lack of progress as failure
            # when the guard is passing the command through untouched.
            if (last_err is not None and abs(last_err - err) < 0.05
                    and ginfo["action"] == "pass"):
                return {"reached": False,
                        "reason": f"no progress (residual {err:.1f} deg) — servo stalled "
                                  f"or target unreachable",
                        "iters": it + 1, "guard": guard_actions, "residual_deg": err}
            last_err = err

        return {"reached": False, "reason": f"did not arrive in {max_iters} iterations",
                "iters": max_iters, "guard": guard_actions, "residual_deg": last_err}

    def _grasp_geometry(self, obj_pos, obj_height: float) -> dict:
        """Geometry shared by action preprocessing and the stage-0 close gate."""
        height = float(obj_height)
        if not np.isfinite(height) or height <= 0.0:
            raise MissingObjectHeightError(
                f"object height must be finite and positive, got {obj_height!r}")
        if self._wrist_z_offset is None:
            raise ContractMismatchError(
                "episode wrist_z_offset must be frozen before action preprocessing")

        arm = np.asarray(self._current_arm_rads, dtype=np.float64)
        grip = float(self._current_grip_rad)
        tcp, _ = self.fk.compute(arm, grip)
        close_drop = self.fk.close_drop(arm, grip)
        target = dc.stage0_target(obj_pos, self._wrist_z_offset, close_drop)

        delta = np.asarray(target, dtype=np.float64) - np.asarray(tcp, dtype=np.float64)
        xy_err = float(np.linalg.norm(delta[:2]))
        z_err = float(abs(delta[2]))
        radius = float(np.linalg.norm(delta))
        centred = (xy_err < self.cfg.entry_xy_tol
                   and z_err < self.cfg.entry_z_tol
                   and radius < self.cfg.entry_radius)

        # Same quantity as training _pads_ready_to_close(): actual pad AABB bottom,
        # minus only the close-drop that has not happened yet.
        remaining_drop = max(0.0, dc.GRASP_CLOSE_DROP - close_drop)
        pad_bottom = self.fk.pad_bottom_z(arm, grip)
        pad_after_close = pad_bottom - remaining_drop
        object_top = float(obj_pos[2]) + 0.5 * height
        pads_ready = bool(
            pad_after_close <= object_top - dc.STAGE1_MIN_ENGAGE_DEPTH)
        return {
            "target": np.asarray(target, dtype=np.float64),
            "tcp": np.asarray(tcp, dtype=np.float64),
            "target_distance_m": radius,
            "xy_error_m": xy_err,
            "z_error_m": z_err,
            "centred": bool(centred),
            "pads_ready": pads_ready,
            "pad_after_close_m": float(pad_after_close),
            "object_top_m": float(object_top),
        }

    def _ready_to_close(self, obj_pos, obj_height) -> Tuple[bool, str]:
        """Deployment close gate: centred on target AND finger pads ready."""
        g = self._grasp_geometry(obj_pos, obj_height)
        ok = bool(g["centred"] and g["pads_ready"])
        why = (f"xy={g['xy_error_m']*1000:.1f}/{self.cfg.entry_xy_tol*1000:.0f}mm "
               f"z={g['z_error_m']*1000:.1f}/{self.cfg.entry_z_tol*1000:.0f}mm "
               f"r={g['target_distance_m']*1000:.1f}/{self.cfg.entry_radius*1000:.0f}mm "
               f"centred={g['centred']} pads_ready={g['pads_ready']} "
               f"(pad_after_close={g['pad_after_close_m']*1000:.1f}mm, "
               f"object_top={g['object_top_m']*1000:.1f}mm)")
        return ok, why

    def _prepare_policy_action(
        self,
        raw_action,
        obj_pos,
        obj_height: float,
        *,
        contact_detected: bool = False,
    ) -> dict:
        """Apply the frozen policy-action semantics once for the current step."""
        geometry = self._grasp_geometry(obj_pos, obj_height)
        raw = np.asarray(raw_action, dtype=np.float32)
        if raw.shape != (6,) or not np.all(np.isfinite(raw)):
            raise ValueError(f"policy action must be a finite (6,) vector, got {raw.shape}")
        raw = np.clip(raw, -1.0, 1.0)

        if self.contract.arm_action_mode == "incremental":
            floor_clearance = (
                self.fk.min_gripper_link_z(
                    self._current_arm_rads, self._current_grip_rad)
                - float(self.cfg.ground_height)
            )
            prepared = self._action_execution.prepare(
                raw,
                self._current_arm_rads,
                self._current_grip_rad,
                self.cfg.arm_sim_limits,
                target_distance_m=geometry["target_distance_m"],
                floor_clearance_m=floor_clearance,
                pads_ready=geometry["pads_ready"],
                contact_detected=contact_detected,
            )
        else:
            # Legacy reference only. v21 uses the stateful incremental path above.
            arm, grip = self.mapper.norm_action_to_sim_angles(
                raw, self._current_arm_rads)
            prepared = {
                "raw_action_for_observation": raw.copy(),
                "filtered_action": raw.copy(),
                "arm_target_rads_pre_floor_guard": arm,
                "grip_target_rad_pre_floor_guard": float(grip),
            }
        prepared["grasp_geometry"] = geometry
        return prepared
    def attempt_close(self) -> dict:
        """Close the jaw and decide whether stage 2 may run. Testable in isolation.

        Only ONE outcome permits a lift: the jaw was driven by an unobstructed,
        fully-fed-back command chain and stopped short of the closed stop because
        something physically blocked it. Everything else — the guard clamping the
        close, an emergency raise/hold, a timeout, a write or read failure, or the jaw
        closing all the way onto nothing — means we do not know that we are holding
        anything, and stage 2 must not run.

        The previous gate advanced whenever the failure reason merely lacked the word
        "failed", so a guard-clamped, never-completed close still went on to lift and
        print "Scripted return complete".
        """
        res = self.move_guarded_and_verified(
            self._current_arm_rads, dc.GRIPPER_ANGLE_CLOSED,
            label="stage1-close", run_time_ms=300, settle_s=0.30,
            max_iters=self.cfg.close_max_iters, tol_deg=3.0)

        actions = set(res.get("guard", []))
        blocking = actions - {"pass", "disabled"}
        out = {"may_lift": False, "close": res, "guard_actions": sorted(actions)}

        if blocking:
            out["reason"] = (f"floor guard intervened during the close ({sorted(blocking)}) "
                             f"— the jaw never reached a known state")
            return out
        if "failed" in res["reason"]:
            out["reason"] = f"hardware fault during the close: {res['reason']}"
            return out
        if res["reached"]:
            out["reason"] = ("jaw reached the fully-closed stop — nothing blocked it, "
                             "so there is no object between the fingers")
            return out
        if "no progress" not in res["reason"]:
            out["reason"] = f"close did not resolve to a stall: {res['reason']}"
            return out

        # Stalled with the guard passing and feedback valid — a real obstruction.
        out["may_lift"] = True
        out["reason"] = f"jaw stalled against something ({res['reason']})"
        return out

    def _grasp_looks_real(self) -> Tuple[str, str]:
        """Best available check that something is actually between the fingers.

        This robot has no force or tactile sensor, so a true grasp signal does not
        exist. The one honest proxy available is the gripper servo's settled angle: a
        jaw closing on nothing reaches the fully-closed stop, while a jaw closing on an
        object stalls short of it. We require it to stall by a clear margin.

        This is a proxy, not a measurement. It cannot distinguish an object from a
        finger fouling, and it will call an over-soft object a miss. It exists so the
        controller does not simply assume "I sent a close command, therefore I am
        holding something" and then lift and drive off with nothing.
        """
        if self.servo.readings_are_simulated:
            # Three states, not two. A dry-run reading is fabricated, so it can neither
            # confirm nor refute a grasp. Returning True here previously let the run
            # print "Scripted return complete" as though something had been picked up.
            return "unverified", ("DRY-RUN — simulated readings; grasp can be neither "
                                  "confirmed nor refuted")
        rd = self.servo.read_degrees()
        if not rd.valid:
            return "rejected", (f"gripper servo unreadable ({rd.reason}) — grasp "
                                f"unconfirmed, treating as failure")
        grip_deg = float(rd.degrees[5])
        if not np.isfinite(grip_deg):
            return "rejected", "gripper servo returned a non-finite angle"
        closed = float(self.cfg.gripper_hw_closed)
        open_ = float(self.cfg.gripper_hw_open)
        span = abs(closed - open_)
        stall = abs(closed - grip_deg)
        frac = stall / span if span > 1e-6 else 0.0
        if frac < self.cfg.grasp_stall_min_fraction:
            return "rejected", (f"jaw reached {grip_deg:.1f}deg, only {frac*100:.1f}% short of "
                           f"fully closed ({closed:.0f}deg) — nothing detected between "
                           f"the fingers")
        return "confirmed", (f"jaw stalled at {grip_deg:.1f}deg "
                             f"({frac*100:.1f}% short of closed)")

    def _scripted_lift_and_return(self) -> str:
        """Open-loop lift and return home, mirroring what training scripts.

        Runs without policy input and without trusting tcp_pos, both of which are
        unreliable once the jaw is loaded. Every waypoint still goes through the floor
        guard. Returns True if it believes it delivered an object.
        """
        status, why = self._grasp_looks_real()
        print(f"[Stage 2] grasp check: {status.upper()} — {why}")
        if status == "rejected":
            print("[Stage 2] Not lifting. Opening jaw and returning home empty.")
            self._retreat_home_empty()
            self._outcome = "grasp_rejected"
            return "rejected"

        # Lift straight up first, in small guarded steps, before travelling home —
        # swinging toward home at grasp height would drag the object along the floor.
        arm = np.asarray(self._current_arm_rads, dtype=np.float64).copy()

        # Which way does S2 raise the gripper? Ask FK instead of trusting a sign
        # convention — getting it backwards commands a descent into the floor, which
        # is exactly what happened the first time this was written (the floor guard
        # caught it and clamped every lift step to zero).
        step_rad = float(self.cfg.scripted_lift_rad_per_step)
        probe_up = arm.copy();   probe_up[1] += step_rad
        probe_dn = arm.copy();   probe_dn[1] -= step_rad
        z_up = self.fk.compute(probe_up, self._current_grip_rad)[0][2]
        z_dn = self.fk.compute(probe_dn, self._current_grip_rad)[0][2]
        lift_sign = 1.0 if z_up > z_dn else -1.0
        print(f"[Stage 2] lift direction: S2 {'+' if lift_sign > 0 else '-'}"
              f"{step_rad} rad raises gripper_center "
              f"({max(z_up, z_dn)*100:.1f}cm vs {min(z_up, z_dn)*100:.1f}cm)")

        for i in range(self.cfg.scripted_lift_steps):
            target = np.asarray(self._current_arm_rads, dtype=np.float64).copy()
            target[1] += lift_sign * step_rad
            res = self.move_guarded_and_verified(
                target, self._current_grip_rad, label=f"lift-{i+1}",
                run_time_ms=250, settle_s=0.28, max_iters=8, tol_deg=2.5)
            print(f"[Stage 2] lift {i+1}/{self.cfg.scripted_lift_steps}: "
                  f"reached={res['reached']} ({res['reason']}) guard={set(res['guard'])}")
            if not res["reached"]:
                print("[SAFETY] Aborting lift: the lift step was not confirmed.")
                self.servo.emergency_stop()
                self._outcome = "lift_not_confirmed"
                return "aborted"

        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, self._current_grip_rad, label="return-home",
            run_time_ms=400, settle_s=0.35)
        print(f"[Stage 2] return home: reached={res['reached']} ({res['reason']}, "
              f"{res['iters']} steps)")
        if not res["reached"]:
            print("[SAFETY] Did not confirm arrival at home. Refusing to report a "
                  "completed delivery.")
            self.servo.emergency_stop()
            self._outcome = "return_home_not_confirmed"
            return "aborted"

        status2, why2 = self._grasp_looks_real()
        print(f"[Stage 2] post-return check: {status2.upper()} — {why2}")
        if status2 == "unverified":
            # Exercised the whole flow, proved nothing about a physical object.
            print("[Done] Scripted return executed. OUTCOME: UNVERIFIED — simulated "
                  "readings cannot establish that anything was picked up.")
            self._outcome = "unverified"
            return "unverified"
        if status2 == "confirmed":
            print("[Done] Scripted return complete; object still detected in the jaw.")
            self._outcome = "confirmed"
            return "confirmed"
        print("[Done] Scripted return complete, but the object is NO LONGER detected — "
              "it appears to have been dropped en route.")
        self._outcome = "dropped"
        return "dropped"

    def _scripted_release(self) -> str:
        """Reach forward a little, open the jaw, come back home. No policy, no aiming.

        The bin opening is ~30 cm wide and its rim ~5 cm high, so horizontal placement
        has ~10 cm of slack in every direction and the object can simply fall. The only
        thing that can actually fail is releasing with the pad BELOW the rim, which
        drops the object against the outside of the bin — so the forward reach stops as
        soon as FK says the pad is no longer clear of the rim.

        Note on verification: there is none, by necessity. _grasp_looks_real() infers a
        grasp from the jaw stalling short of fully closed, so once the jaw is commanded
        open it reads "confirmed" whether or not anything fell out. Re-closing on empty
        air would be a real check; it is deliberately not done here to keep the motion
        minimal. Treat the reported outcome as "the release motion ran", not "the bin
        contains the object".
        """
        arm0 = np.asarray(self._current_arm_rads, dtype=np.float64).copy()
        grip0 = float(self._current_grip_rad)

        # Which way does S2 reach FORWARD? Ask FK rather than trusting a sign — the
        # lift got this backwards the first time it was written.
        step_rad = float(self.cfg.release_extend_rad_per_step)
        probe_p = arm0.copy(); probe_p[1] += step_rad
        probe_n = arm0.copy(); probe_n[1] -= step_rad
        x_p = self.fk.compute(probe_p, grip0)[0][0]
        x_n = self.fk.compute(probe_n, grip0)[0][0]
        fwd_sign = 1.0 if x_p > x_n else -1.0
        print(f"[Stage 3] reach direction: S2 {'+' if fwd_sign > 0 else '-'}{step_rad} rad "
              f"extends forward ({max(x_p, x_n)*100:.1f}cm vs {min(x_p, x_n)*100:.1f}cm)")

        min_pad_z = float(self.cfg.bin_rim_height) + float(self.cfg.release_clearance)
        for i in range(self.cfg.release_extend_steps):
            cand = np.asarray(self._current_arm_rads, dtype=np.float64).copy()
            cand[1] += fwd_sign * step_rad
            pad_z = self.fk.pad_bottom_z(cand, float(self._current_grip_rad))
            if pad_z < min_pad_z:
                # Reaching further would put the jaw below the rim. Releasing from where
                # we are is fine — the bin is 30 cm wide — so stop extending, do not abort.
                print(f"[Stage 3] stopping the reach at step {i+1}: pad would sit at "
                      f"{pad_z*100:.1f}cm, below rim+clearance ({min_pad_z*100:.1f}cm). "
                      f"Releasing from here.")
                break
            res = self.move_guarded_and_verified(
                cand, float(self._current_grip_rad), label=f"release-reach-{i+1}",
                run_time_ms=250, settle_s=0.28, max_iters=8, tol_deg=2.5)
            print(f"[Stage 3] reach {i+1}/{self.cfg.release_extend_steps}: "
                  f"reached={res['reached']} ({res['reason']}) guard={set(res['guard'])}")
            if not res["reached"]:
                # Same reasoning: an unfinished reach is not a reason to keep holding
                # the object. Release from wherever the arm actually stopped.
                print("[Stage 3] Reach not confirmed — releasing from the current pose.")
                break

        pad_now = self.fk.pad_bottom_z(
            np.asarray(self._current_arm_rads, dtype=np.float64),
            float(self._current_grip_rad))
        if pad_now < min_pad_z:
            # Opening here would let go with the jaw at or below the rim, which drops the
            # object against the outside of the bin. Keep holding it and report — that is
            # recoverable, a misplaced drop is not. (From the C3 home pose the pad sits at
            # 8.4 cm, so this fires only if the home pose or the rim height changes.)
            print(f"[SAFETY] Pad bottom is {pad_now*100:.1f}cm, not clear of the "
                  f"{self.cfg.bin_rim_height*100:.0f}cm rim + "
                  f"{self.cfg.release_clearance*100:.0f}cm clearance. Refusing to open "
                  f"the jaw here; keeping hold of the object.")
            self._outcome = "release_pose_below_rim"
            return "aborted"
        x_now = self.fk.compute(
            np.asarray(self._current_arm_rads, dtype=np.float64),
            float(self._current_grip_rad))[0][0]
        print(f"[Stage 3] releasing at x={x_now*100:.1f}cm "
              f"({(x_now - self.fk.compute(arm0, grip0)[0][0])*100:+.1f}cm forward of the "
              f"post-lift pose), pad bottom {pad_now*100:.1f}cm vs rim "
              f"{self.cfg.bin_rim_height*100:.0f}cm")

        res = self.move_guarded_and_verified(
            np.asarray(self._current_arm_rads, dtype=np.float64).copy(),
            dc.GRIPPER_ANGLE_OPEN, label="release-open",
            run_time_ms=300, settle_s=0.30, max_iters=self.cfg.close_max_iters,
            tol_deg=3.0)
        print(f"[Stage 3] open jaw: reached={res['reached']} ({res['reason']}, "
              f"{res['iters']} steps)")
        if not res["reached"]:
            print("[SAFETY] The jaw was not confirmed open — not retracting with a "
                  "possibly still-held object.")
            self.servo.emergency_stop()
            self._outcome = "release_not_confirmed"
            return "aborted"

        # Let it fall clear before anything moves; retracting mid-open flicks the object.
        time.sleep(float(self.cfg.release_settle_s))

        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, self.mapper.hw_deg_to_sim_grip(self.cfg.home_deg[5]),
            label="release-home", run_time_ms=400, settle_s=0.35)
        print(f"[Stage 3] return home: reached={res['reached']} ({res['reason']})")
        if not res["reached"]:
            self._outcome = "release_home_not_confirmed"
            return "aborted"

        print("[Done] Release motion complete (drop itself is not sensed).")
        self._outcome = "released"
        return "released"

    def _retreat_home_empty(self) -> None:
        """Open the jaw and go home, guarded. Used on every abort path."""
        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, dc.GRIPPER_ANGLE_OPEN, label="retreat-home",
            run_time_ms=400, settle_s=0.35)
        print(f"[Retreat] home: reached={res['reached']} ({res['reason']})")

    def _sync_joint_state_from_servos(self) -> bool:
        """Refresh internal joint state from the encoders. False if unreadable."""
        rd = self.servo.read_degrees()
        if not rd.valid:
            print(f"[SAFETY] Servo read failed: {rd.reason}")
            return False
        self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
        self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])
        return True

    def run_release_only(self) -> str:
        """Stage 3 on its own, for the machine standing at the bin.

        The deployed sequence is grasp → drive to the bin → release, and the drive
        happens between two separate invocations of this script. This entry point is
        the second one: it does NOT go to home first (the arm is already holding
        something, and a home move would be a large unguarded-looking swing with a
        load), it reads the pose off the encoders and reaches from wherever it is.
        """
        print("\n" + "=" * 60)
        print("X3Plus Release Controller — Stage 3 only")
        print(f"  reach forward, open jaw, return home "
              f"(bin rim {self.cfg.bin_rim_height*100:.0f}cm)")
        print("=" * 60 + "\n")

        if not self._sync_joint_state_from_servos():
            print("[SAFETY] Cannot read the arm pose. Refusing to move.")
            self._outcome = "servo_unreadable"
            return "aborted"

        hw = self.mapper.sim_arm_to_hw_deg(self._current_arm_rads)
        hw.append(self.mapper.sim_grip_to_hw_deg(self._current_grip_rad))
        print(f"[Start] Measured pose S1-6 = {[round(d, 1) for d in hw]}")

        status, why = self._grasp_looks_real()
        print(f"[Start] grasp check: {status.upper()} — {why}")
        if status == "rejected":
            # Nothing in the jaw. Opening it here would be theatre, and would also hide
            # the fact that the object was lost somewhere between the grasp and the bin.
            print("[Start] Nothing detected in the jaw — there is nothing to release.")
            self._outcome = "nothing_to_release"
            return "rejected"

        self._stage = 3
        return self._scripted_release()

    def run(self, max_steps: int = 300):
        if self.obs_builder is None:
            raise RuntimeError(
                "This controller was built without the policy (release-only). "
                "Call run_release_only(), or construct it with load_policy=True."
            )
        print("\n" + "="*60)
        print("X3Plus Real Grasp Controller")
        print(f"  Stage 0 → align gripper above object")
        print(f"  Stage 1 → close gripper (grasp)")
        print(f"  Stage 2 → lift and return home")
        if self.cfg.release_enable:
            print(f"  Stage 3 → reach forward, open jaw, return home "
                  f"(bin rim {self.cfg.bin_rim_height*100:.0f}cm)")
        print("="*60 + "\n")

        # Move to home first — guarded and verified like every other motion. This is
        # the largest single move the robot makes, so a one-shot 8 deg-limited command
        # gets nowhere near it.
        print("[Start] Moving to home position...")
        rd = self.servo.read_degrees()
        if rd.valid:
            self._current_arm_rads = self.mapper.hw_deg_to_sim_arm(rd.degrees[:5])
            self._current_grip_rad = self.mapper.hw_deg_to_sim_grip(rd.degrees[5])
        elif not self.servo.dry_run:
            print(f"[SAFETY] Cannot read servos at startup ({rd.reason}). Refusing to move.")
            return
        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, self.mapper.hw_deg_to_sim_grip(self.cfg.home_deg[5]),
            label="startup-home", run_time_ms=400, settle_s=0.35)
        print(f"[Start] home: reached={res['reached']} ({res['reason']}, "
              f"{res['iters']} steps)")
        if not res["reached"]:
            print("[SAFETY] Did not confirm the home pose. Aborting before any grasp.")
            return
        self._stage = 0
        self._wrist_z_offset = None
        self._episode_object_height = None
        self._object_height_source = None
        self._prev_action = np.zeros(6, dtype=np.float32)
        self._action_execution.reset(self._current_grip_rad)

        dt = 1.0 / self.cfg.control_hz
        obj_pos = self._get_obj_pos()
        print(f"[Start] Object position: {obj_pos.tolist()}")
        print(f"[Start] Running for up to {max_steps} steps at {self.cfg.control_hz} Hz\n")

        for step in range(max_steps):
            t0 = time.time()

            obj_pos = self._get_obj_pos()
            obj_height = self._get_obj_height(obj_pos)
            # Frozen at the first detection, while the object is still resting on the
            # floor — matching the training env, which fixes it at reset. Recomputing
            # it from the object's live z drifts the target once the object lifts.
            if self._wrist_z_offset is None:
                self._wrist_z_offset = dc.episode_wrist_z_offset(
                    obj_height, float(obj_pos[2]))
                print(f"[Init] episode wrist_z_offset = {self._wrist_z_offset:.4f} m "
                      f"(object height {obj_height}, resting z {float(obj_pos[2]):.4f})")
            obs = self.obs_builder.build(
                self._current_arm_rads,
                self._current_grip_rad,
                obj_pos,
                self._stage,
                self._prev_action,
                obj_height=obj_height,
                wrist_z_offset=self._wrist_z_offset,
            )

            # ── Model inference ───────────────────────────────────────────
            action, _ = self.model.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float32)
            if action.shape != (6,) or not np.all(np.isfinite(action)):
                raise ContractMismatchError(
                    "policy returned a non-finite or wrong-shaped action: "
                    f"{action.shape}"
                )
            prepared = self._prepare_policy_action(
                action, obj_pos, obj_height, contact_detected=False)
            arm_sim = prepared["arm_target_rads_pre_floor_guard"]
            grip_sim = prepared["grip_target_rad_pre_floor_guard"]
            dist_to_target = prepared["grasp_geometry"]["target_distance_m"]

            # ── Stage management ──────────────────────────────────────────


            # Stage transitions
            if self._stage == 0:
                ready, why = self._ready_to_close(obj_pos, obj_height)
                if ready:
                    print(f"\n[Stage] 0→1  {why}")
                    self._stage = 1

            elif self._stage == 1:
                verdict = self.attempt_close()
                res = verdict["close"]
                print(f"\n[Stage] 1: jaw close — reached={res['reached']} "
                      f"({res['reason']}, {res['iters']} steps) "
                      f"guard={verdict['guard_actions']}")
                if not verdict["may_lift"]:
                    print(f"[Stage] 1 ABORT — {verdict['reason']}")
                    print("[Stage] Not lifting. Opening the jaw and retreating home.")
                    self._retreat_home_empty()
                    self._outcome = "close_not_confirmed"
                    return
                print(f"[Stage] 1→2  {verdict['reason']}")
                self._prev_action = prepared[
                    "raw_action_for_observation"].copy()
                self._stage = 2
                continue

            elif self._stage == 2:
                # ── SCRIPTED, not policy-driven ───────────────────────────
                # Training scripts the arm from the moment the grasp latches, so the
                # policy was never trained to act here. Worse, this is exactly where
                # deployment FK is least trustworthy: once the jaw squeezes an object
                # the linkage leaves the grip_joint*multiplier relation (measured up to
                # 0.199 rad), and tcp_pos drifts by ~4 cm. Feeding that observation back
                # into the policy would be driving on a broken sensor. So stage 2 runs
                # open-loop, matching training.
                status = self._scripted_lift_and_return()
                # Only release when the lift believes something is still in the jaw.
                # "dropped"/"rejected"/"aborted" mean there is nothing to put in the bin.
                if self.cfg.release_enable and status in ("confirmed", "unverified"):
                    self._stage = 3
                    self._scripted_release()
                break

            # ── Convert action → servo degrees ────────────────────────────
            # Last gate before the servos. Prevention only — there is no undo on hardware.
            arm_sim, grip_sim, ginfo = self.floor_guard.project(
                self._current_arm_rads, self._current_grip_rad, arm_sim, grip_sim)
            if ginfo["action"] != "pass":
                print(f"  [floor guard] {ginfo['action']} {ginfo}")

            # Through the same primitive as every other motion, with max_iters=1: a
            # policy step is one small increment, so it should not be chased to
            # convergence — but it still gets the identical guard, write-failure and
            # read-failure handling, and its state still comes from the encoders.
            res = self.move_guarded_and_verified(
                arm_sim, grip_sim, label="policy-step",
                run_time_ms=self.cfg.servo_run_time_ms, settle_s=0.0,
                max_iters=1, tol_deg=self.cfg.max_delta_deg + 1.0)
            if "failed" in res["reason"]:
                print(f"[SAFETY] {res['reason']} during approach — stopping.")
                self.servo.emergency_stop()
                return
            self._prev_action = prepared[
                "raw_action_for_observation"].copy()

            # Status — report the MEASURED pose, not the command we sent.
            meas = self.mapper.sim_arm_to_hw_deg(self._current_arm_rads)
            meas.append(self.mapper.sim_grip_to_hw_deg(self._current_grip_rad))
            print(f"\r[Step {step+1:3d}] Stage={self._stage} "
                  f"target_dist={dist_to_target:.3f}m "
                  f"grip_cmd={float(action[5]):.2f} "
                  f"S1-6={[round(d,1) for d in meas]}",
                  end="", flush=True)

            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
        else:
            print(f"\n[End] Max steps ({max_steps}) reached.")

        # Return to home — through the guarded primitive like everything else.
        print("\n[End] Moving back to home...")
        home = self.mapper.hw_deg_to_sim_arm(list(self.cfg.home_deg[:5]))
        res = self.move_guarded_and_verified(
            home, self.mapper.hw_deg_to_sim_grip(self.cfg.home_deg[5]),
            label="end-home", run_time_ms=400, settle_s=0.35)
        print(f"[End] home: reached={res['reached']} ({res['reason']})")

    def close(self):
        if self.detection is not None:
            self.detection.close()
        self.fk.close()


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(description="X3Plus real-robot grasp deployment")
    p.add_argument("--model", type=str, default=None,
                   help="Path to .zip model (default: trained_6d_models_v18/ppo_6d_final_ready_for_real_robot.zip)")
    p.add_argument("--vecnorm", type=str, default=None,
                   help="Path to vecnormalize .pkl")
    p.add_argument("--real", action="store_true",
                   help="Actually send commands to servos (default: dry-run print only)")
    p.add_argument("--socket", action="store_true",
                   help="Listen for object detection on TCP socket (port 5555)")
    p.add_argument("--obj-x", type=float, default=0.26,
                   help="Object X position in metres (forward, default 0.26; v18 deployable x 0.20-0.33)")
    p.add_argument("--obj-y", type=float, default=0.00,
                   help="Object Y position in metres (lateral, default 0.00)")
    p.add_argument("--obj-z", type=float, default=0.02,
                   help="Object centroid Z in metres (NOT its height; default 0.02)")
    p.add_argument("--object-height", type=float, default=None,
                   help="Object height = full z extent, metres (e.g. 0.013 for a bottle "
                        "cap). Required by obs_32_goal_incremental. Distinct from --obj-z.")
    p.add_argument("--contract", type=str, default=None,
                   choices=sorted(dc.CONTRACTS),
                   help="Observation/action contract the weights were trained under. "
                        "v18/v19 = obs_28_absolute, v20 = obs_32_goal_incremental. "
                        "Read it off the model's manifest.json — it cannot be inferred.")
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--release-to-bin", action="store_true",
                   help="After a successful lift, reach forward, open the jaw and return "
                        "home (stage 3). Off by default — with no bin in front of the "
                        "robot this drops the object on the floor. Use this only when "
                        "the bin is already in front of the robot at grasp time.")
    p.add_argument("--release-only", action="store_true",
                   help="Run ONLY stage 3 and exit: read the current pose, reach forward, "
                        "open the jaw, return home. This is the command to run at the bin "
                        "after navigating there with an object already in the jaw. Needs "
                        "no model weights — the release is a fixed sequence.")
    p.add_argument("--bin-rim-height", type=float, default=0.05,
                   help="Bin rim height above the floor in metres (default 0.05). The "
                        "forward reach stops before the jaw pad drops below it.")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = DeployConfig(
        control_hz=args.hz,
        default_obj_pos=(args.obj_x, args.obj_y, args.obj_z),
        default_obj_height=args.object_height,
        release_enable=args.release_to_bin or args.release_only,
        bin_rim_height=args.bin_rim_height,
    )
    if args.model:
        cfg.model_path = args.model
    if args.vecnorm:
        cfg.vecnorm_path = args.vecnorm
    if args.contract:
        cfg.contract_name = args.contract

    controller = GraspController(cfg, real_servo=args.real,
                                 use_socket=args.socket and not args.release_only,
                                 load_policy=not args.release_only)
    try:
        if args.release_only:
            controller.run_release_only()
        else:
            controller.run(max_steps=args.max_steps)
    except KeyboardInterrupt:
        print("\n[Interrupted] Emergency stop.")
        controller.servo.emergency_stop()
    finally:
        controller.close()


if __name__ == "__main__":
    main()
