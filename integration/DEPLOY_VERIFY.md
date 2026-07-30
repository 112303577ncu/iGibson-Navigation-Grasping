# X3Plus Real-Robot Verification

This checklist keeps the current robot logic unchanged. Use it to verify the
single-process pipeline on the Jetson in safe stages.

## 1. Preflight and Self-Test

On the Jetson:

```bash
cd ~/Documents/x3plus_pipeline_deploy
source ~/grasp_venv/bin/activate
python3 integration/verify_x3plus_deploy.py
```

The verifier checks:

- required model, VecNormalize, and URDF files
- `Rosmaster_Lib` import availability
- `ultralytics` and `cv2` imports
- serial port presence
- port 7000 motor server is not listening
- `vision_grasp_pipeline.py --selftest`

Preflight warnings do not move the robot. Fix real-run blockers before using
`--run-real`.

## 2. Camera Dry-Run

Dry-run uses cameras and prints the `set_car_motion(...)` commands, but it does
not drive wheels or servos:

```bash
python3 integration/verify_x3plus_deploy.py --run-dry
```

Watch for:

- `REAR dist_front=... off=...` values that match the rear-camera scene
- `ARM dist=... off=...` values that match the arm-camera scene
- `HANDOFF` only when the object is centered and around the handoff distance
- `latched grasp target pos=[x, y, z] width=...` matching the real object

Stop with `Ctrl+C`.

## 3. Real Run

Before real motion, confirm:

- no port 7000 motor server is running
- no ROS base driver owns the Rosmaster serial port
- robot workspace is clear
- emergency stop is ready

Then run:

```bash
python3 integration/verify_x3plus_deploy.py --run-real --i-understand-real-motion
```

## Acceptance Checks

- Rear stage: centered object drives forward; left/right offsets turn correctly.
- Arm stage: robot stops at handoff distance and does not run a final blind push.
- Handoff log: latched `[x, y, z]` and `width` look physically plausible.
- PPO grasp: arm moves toward the object.
- Stage 1: S6 closes from 30 degrees toward 180 degrees.
- Stage 2: arm returns home while keeping the gripper closed.
- Failure retry: object still at the same spot means retreat and retry, at most
  three attempts by default.

## Tune Only If Symptoms Appear

- Left/right reversed: change `SIGN_Y`.
- Fixed left/right offset: tune `CAM_TO_BASE_Y`.
- Fixed forward/back offset: tune `CAM_TO_BASE_X` or `ARM_BLIND_START_DIST_M`.
- Fixed height error: tune `OBJ_Z_FIXED`.
- Distance scale error: recalibrate `THETA_ARM/H_ARM/FX_ARM/FY_ARM`.
- Turn direction reversed: swap the `v_z` signs for `turn_left` and
  `turn_right` in `action_to_vxyz()`.
