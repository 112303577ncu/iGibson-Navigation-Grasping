#!/usr/bin/env python3
"""One stopped ArUco/scan batch using V4L2 source timestamps; no movement."""
import argparse
from collections import deque
import json
from pathlib import Path
import subprocess
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from integration import v23_task_resume_guard as guard

CAMERA = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default=str(ROOT/'maps/demo_box_20261017/demo_box.yaml'))
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--registration-id', required=True)
    parser.add_argument('--camera-id', default='rear-sonix-640x480')
    parser.add_argument('--snapshot', default='/dev/shm/g2_corner_capture.json')
    args = parser.parse_args()
    if guard.available_mb() < 1024:
        raise RuntimeError('RAM below capture reserve')
    probe = subprocess.run(['fuser', CAMERA], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if probe.returncode == 0:
        raise RuntimeError('rear camera already owned; reuse its stream rather than open twice')
    import cv2
    folder = Path(args.output_dir); folder.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
    child = None
    records = deque(maxlen=10); seq = 0; previous_pts = -1
    old_capture = guard.read_json(args.snapshot).get('capture_id') if Path(args.snapshot).exists() else None
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    try:
        if not cap.isOpened():
            raise RuntimeError('camera open failed')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) or cap.get(cv2.CAP_PROP_BUFFERSIZE) != 1:
            raise RuntimeError('cannot bound camera buffering')
        deadline = time.monotonic()+15; snapshot = None
        while time.monotonic() < deadline:
            if guard.available_mb() < 768:
                raise RuntimeError('RAM pressure during capture')
            ok, image = cap.read(); now = time.time(); monotonic = time.monotonic()
            if not ok or image.shape != (480, 640, 3):
                raise RuntimeError('invalid rear camera frame')
            pts = cap.get(cv2.CAP_PROP_POS_MSEC)/1000.
            age = monotonic-pts
            if not 0 <= age <= .25 or pts <= previous_pts:
                if seq < 10:
                    seq += 1; continue
                raise RuntimeError('V4L2 source timestamp is not fresh/increasing on monotonic clock')
            previous_pts = pts; seq += 1
            if seq < 10:
                continue
            if child is None:
                command = ['/bin/bash', '-c', 'source /opt/ros/melodic/setup.bash; '
                           'export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1; '
                           'exec /usr/bin/python "$@"', 'corner-capture',
                           str(ROOT/'deploy/ros/g2_corner_snapshot.py'), '--map', args.map, '--output', args.snapshot]
                child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            corners, ids, _ = detector.detectMarkers(image)
            markers = []
            if ids is not None:
                for marker_id, corner in zip(ids.flatten(), corners):
                    u, v = corner.reshape(-1, 2).mean(axis=0)
                    markers.append(dict(id=int(marker_id), side='left' if u < 256 else ('right' if u > 384 else 'center'),
                                        center_px=[float(u), float(v)]))
            records.append((dict(seq=seq, stamp=now-age, source_monotonic=pts,
                                 source_age_s=age, markers=markers), image.copy()))
            if snapshot is None and Path(args.snapshot).exists():
                candidate = guard.read_json(args.snapshot)
                if candidate.get('capture_id') != old_capture:
                    snapshot = candidate
            if child.poll() is not None and child.returncode:
                stdout, stderr = child.communicate()
                raise RuntimeError('native capture failed: '+(stdout+stderr).decode('utf-8', 'replace'))
            if snapshot is None or now < snapshot['time']+.2:
                continue
            selected = [(record, image) for record, image in records
                        if abs(record['stamp']-snapshot['time']) <= .7 and record['markers']][-3:]
            if len(selected) != 3 or not set.intersection(*[{m['id'] for m in r['markers']} for r, _ in selected]):
                continue
            stdout, stderr = child.communicate(timeout=2)
            if child.returncode:
                raise RuntimeError((stdout+stderr).decode('utf-8', 'replace'))
            packet = {key: snapshot[key] for key in ('capture_id', 'boot_id', 'owner_pid', 'map_hash')}
            packet.update(schema=1, clock='jetson', registration_id=args.registration_id,
                          camera_id=args.camera_id, frames=[record for record, _ in selected])
            for index, (record, image) in enumerate(selected):
                name = 'frame_%02d.jpg' % index
                cv2.imwrite(str(folder/name), image); record['file'] = name
            # The PC matcher binds the exact native bytes. Reformatting JSON
            # here would invalidate its hash against the trusted Jetson file.
            shutil.copyfile(args.snapshot, str(folder/'capture.json'))
            guard.atomic(folder/'observations.json', packet)
            print(json.dumps(dict(ok=True, capture_id=snapshot['capture_id'], output_dir=str(folder),
                                  ids=[m['id'] for m in selected[-1][0]['markers']],
                                  max_source_age_s=max(r['source_age_s'] for r, _ in selected),
                                  arm_or_motion_executed=False)))
            return 0
        raise RuntimeError('no synchronized three-frame marker/scan batch within 15s')
    finally:
        cap.release()
        if child is not None and child.poll() is None:
            child.terminate()
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: child.kill(); child.wait()


if __name__ == '__main__':
    raise SystemExit(main())
