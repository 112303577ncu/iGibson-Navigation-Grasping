#!/usr/bin/env python3
"""Startup device inventory/check for the X3Plus Jetson.

Run this after boot, before calibration or real-robot tests:

    cd ~/Documents/deploy_jetson2
    source ~/grasp_venv/bin/activate
    python3 startup_device_check.py

It reports which USB serial devices and cameras are connected, verifies that
/dev/myserial points at the ch341 Rosmaster board, and optionally probes camera
frames with OpenCV.
"""
from __future__ import annotations

import argparse
import glob
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional


ROSMaster_DRIVERS = {"ch341"}
ROSMaster_VENDOR_IDS = {"1a86"}
LIDAR_DRIVERS = {"cp210x"}
LIDAR_VENDOR_IDS = {"10c4"}

# Cameras are identified by USB serial, NOT by /dev/videoN (indices reshuffle
# across replug/boot). Both are Sonix; only the rear one reports SN0001.
REAR_CAM_SN_HINTS = {"sn0001"}


def run(cmd: List[str], timeout: float = 3.0) -> str:
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            errors="replace",   # never crash on odd bytes / locale
            timeout=timeout,
            check=False,
        )
        # Combine both streams: fuser writes its listing to stderr while the
        # PIDs go to stdout, so a truthy-but-blank stdout must not mask stderr.
        return ((p.stdout or "") + (p.stderr or "")).strip()
    except FileNotFoundError:
        return ""
    except subprocess.TimeoutExpired:
        return "<timeout>"


def udev_props(dev: str) -> Dict[str, str]:
    out = run(["udevadm", "info", "-q", "property", "-n", dev])
    props: Dict[str, str] = {}
    for line in out.splitlines():
        if "=" in line:
            key, val = line.split("=", 1)
            props[key] = val
    return props


def short_props(props: Dict[str, str]) -> str:
    fields = [
        ("ID_USB_DRIVER", "driver"),
        ("ID_VENDOR_ID", "vid"),
        ("ID_MODEL_ID", "pid"),
        ("ID_SERIAL_SHORT", "sn"),
        ("ID_VENDOR_FROM_DATABASE", "vendor"),
        ("ID_MODEL_FROM_DATABASE", "model"),
        ("ID_MODEL", "name"),
        ("ID_PATH", "path"),
    ]
    parts = []
    for key, label in fields:
        val = props.get(key)
        if val:
            parts.append(f"{label}={val}")
    return ", ".join(parts) if parts else "(no udev details)"


def classify_serial(props: Dict[str, str]) -> str:
    driver = props.get("ID_USB_DRIVER", "")
    vid = props.get("ID_VENDOR_ID", "")
    model = " ".join([
        props.get("ID_MODEL", ""),
        props.get("ID_MODEL_FROM_DATABASE", ""),
    ]).lower()
    if driver in ROSMaster_DRIVERS or vid in ROSMaster_VENDOR_IDS:
        return "Rosmaster board"
    if driver in LIDAR_DRIVERS or vid in LIDAR_VENDOR_IDS or "cp210" in model:
        return "LiDAR / CP210x serial"
    return "unknown serial"


def classify_camera(props: Dict[str, str]) -> str:
    """Label a /dev/video* node as arm/rear by USB serial (indices are unstable).

    Rear = the Sonix reporting SN0001; arm = the other Sonix. A camera usually
    exposes two video nodes (capture + metadata) that share one serial, so both
    nodes get the same label.
    """
    sn = (props.get("ID_SERIAL_SHORT") or props.get("ID_SERIAL") or "").lower()
    model = " ".join([
        props.get("ID_MODEL", ""),
        props.get("ID_MODEL_FROM_DATABASE", ""),
    ]).lower()
    is_cam = "sonix" in model or "camera" in model or "video" in model
    if any(h in sn for h in REAR_CAM_SN_HINTS):
        return "REAR camera (SN0001)"
    if is_cam and sn:
        return "ARM camera (non-SN0001)"
    if is_cam:
        return "camera (no serial reported)"
    return "video node (unknown)"


def camera_identity(props: Dict[str, str]) -> Optional[str]:
    """Stable physical-camera key shared by capture/metadata video nodes.

    Never fall back to /dev/videoN: one USB camera commonly exposes two nodes,
    so counting node names can falsely report that two physical cameras exist.
    """
    serial = props.get("ID_SERIAL_SHORT") or props.get("ID_SERIAL")
    if serial:
        return f"serial:{serial.lower()}"
    path = props.get("ID_PATH") or props.get("ID_PATH_TAG")
    if path:
        # udev may append a per-video-node suffix to an otherwise shared USB path.
        return f"path:{path.split('-video-index', 1)[0].lower()}"
    return None


def real_target(path: str) -> Optional[str]:
    if not os.path.exists(path) and not os.path.islink(path):
        return None
    return os.path.realpath(path)


def list_symlinks(root: str) -> List[str]:
    if not os.path.isdir(root):
        return []
    out = []
    for p in sorted(Path(root).iterdir()):
        if p.is_symlink():
            out.append(f"{p} -> {os.path.realpath(str(p))}")
    return out


def fuser(dev: str) -> str:
    return run(["fuser", "-v", dev], timeout=2.0)


def print_serial_section(expected_rosmaster: str) -> bool:
    print("\n== USB serial ==")
    print("by-id (stable paths - prefer these, e.g. for --lidar-port):")
    ser_by_id = list_symlinks("/dev/serial/by-id")
    print(indent("\n".join(ser_by_id) if ser_by_id else "(none)", "  "))
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        print("FAIL: no /dev/ttyUSB* or /dev/ttyACM* devices found")
        return False

    ok = True
    rosmaster_ports = []
    for dev in ports:
        props = udev_props(dev)
        role = classify_serial(props)
        if role == "Rosmaster board":
            rosmaster_ports.append(dev)
        print(f"{dev}: {role}")
        print(f"  {short_props(props)}")
        users = fuser(dev)
        if users:
            print(f"  in use:\n{indent(users, '    ')}")

    target = real_target(expected_rosmaster)
    if target is None:
        print(f"FAIL: {expected_rosmaster} does not exist")
        ok = False
    else:
        target_props = udev_props(target)
        target_role = classify_serial(target_props)
        print(f"\n{expected_rosmaster} -> {target} ({target_role})")
        if target_role != "Rosmaster board":
            print(f"FAIL: {expected_rosmaster} is not pointing at the Rosmaster board")
            ok = False
        else:
            print("PASS: Rosmaster symlink is correct")

    if not rosmaster_ports:
        print("FAIL: no ch341/1a86 Rosmaster-like serial device found")
        ok = False
    return ok


def indent(text: str, prefix: str) -> str:
    return "\n".join(prefix + line for line in text.splitlines())


def probe_camera(dev: str) -> str:
    try:
        import cv2  # type: ignore
    except Exception as e:
        return f"skip: OpenCV import failed ({e})"

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return "FAIL: open failed"
    ok = False
    frame = None
    for _ in range(20):
        ok, frame = cap.read()
        time.sleep(0.03)
    cap.release()
    if not ok or frame is None:
        return "FAIL: read failed"
    return f"PASS: frame shape={tuple(frame.shape)}"


def print_camera_section(probe: bool) -> bool:
    print("\n== cameras ==")
    videos = sorted(glob.glob("/dev/video*"))
    if not videos:
        print("FAIL: no /dev/video* devices found")
        return False

    print("by-id:")
    by_id = list_symlinks("/dev/v4l/by-id")
    print(indent("\n".join(by_id) if by_id else "(none)", "  "))
    print("by-path:")
    by_path = list_symlinks("/dev/v4l/by-path")
    print(indent("\n".join(by_path) if by_path else "(none)", "  "))

    cameras: Dict[str, str] = {}   # stable serial/path -> role
    frame_ok = set()
    unidentified_nodes: List[str] = []
    for dev in videos:
        props = udev_props(dev)
        role = classify_camera(props)
        key = camera_identity(props)
        if key is None:
            unidentified_nodes.append(dev)
        else:
            cameras.setdefault(key, role)
        print(f"{dev}: {role}")
        print(f"  {short_props(props)}")
        users = fuser(dev)
        if users:
            print(f"  in use:\n{indent(users, '    ')}")
        if probe:
            result = probe_camera(dev)
            print(f"  probe: {result}")
            if result.startswith("PASS") and key is not None:
                frame_ok.add(key)

    print(f"\ndistinct cameras by stable identity: {len(cameras)}")
    for key, role in cameras.items():
        print(f"  {key}: {role}")
    if unidentified_nodes:
        print("WARN: these video nodes have no stable serial/path identity and "
              f"are not counted as physical cameras: {unidentified_nodes}")
    if len(cameras) < 2:
        print("WARN: expected two physical cameras (arm + rear)")

    if probe:
        # A physical camera can expose a capture node + a metadata node; the
        # metadata node never yields a frame. Judge by SERIAL, not by node, so
        # a metadata-node miss does not fail an otherwise-healthy camera.
        if len(frame_ok) >= 2:
            print(f"PASS: {len(frame_ok)} cameras produced frames")
            return True
        print(f"FAIL: only {len(frame_ok)} camera(s) produced a frame "
              "(check capture node / occupancy above)")
        return False
    return len(cameras) >= 2


def print_process_section() -> None:
    print("\n== likely related processes ==")
    out = run(["ps", "-ef"], timeout=3.0)
    keep = []
    needles = ("ros", "yahboom", "rostopic", "rosmaster", "car", "bringup", "jupyter")
    for line in out.splitlines():
        low = line.lower()
        if any(n in low for n in needles) and "startup_device_check.py" not in low:
            keep.append(line)
    print("\n".join(keep) if keep else "(none)")


def main() -> int:
    p = argparse.ArgumentParser(description="X3Plus startup device inventory/check")
    p.add_argument("--rosmaster", default="/dev/myserial",
                   help="expected Rosmaster serial symlink (default: /dev/myserial)")
    p.add_argument("--no-camera-probe", action="store_true",
                   help="list cameras without opening them")
    args = p.parse_args()

    print("X3Plus startup device check")
    print(f"time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"host: {socket.gethostname()}")
    print(f"python: {sys.executable}")

    serial_ok = print_serial_section(args.rosmaster)
    camera_ok = print_camera_section(probe=not args.no_camera_probe)
    print_process_section()

    print("\n== summary ==")
    if serial_ok and camera_ok:
        print("PASS: device mapping looks ready")
        return 0
    print("FAIL/WARN: fix the items above before running real-robot tests")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
