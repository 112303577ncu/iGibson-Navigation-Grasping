#!/usr/bin/env python3
"""Small MJPEG camera server for calibration and debugging.

Use one process to own the V4L2 camera, then let a browser and calibration
script read from HTTP endpoints at the same time:

    python3 stream_cam.py --device 0 --port 8081 --cols 9 --rows 6

    Browser preview:
        http://JETSON_IP:8081/preview

    Raw stream for tools:
        http://127.0.0.1:8081/stream
"""
from __future__ import annotations

import argparse
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

import cv2


CAMERA_PROBE_READS = 3
CAMERA_PROBE_DELAY_SEC = 0.05
CAMERA_READ_FAILURES_BEFORE_REOPEN = 10
CAMERA_REOPEN_ATTEMPTS = 3
CAMERA_REOPEN_DELAY_SEC = 0.5


class CameraState:
    def __init__(self, source: str, width: int, height: int, fps: float,
                 pattern: Optional[Tuple[int, int]]) -> None:
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self.pattern = pattern
        self.lock = threading.Lock()
        self.raw_jpeg: Optional[bytes] = None
        self.preview_jpeg: Optional[bytes] = None
        self.frame_sequence = 0
        self.last_frame_time = 0.0
        self.frame_shape = None
        self.running = True
        self.startup = threading.Event()
        self.capture_error: Optional[str] = None
        self.capture_backend: Optional[str] = None
        self._startup_frame = None
        self._capture_open_error: Optional[str] = None

    def clear_published_frames(self) -> None:
        """Stop HTTP clients from receiving a JPEG after capture is stale."""
        with self.lock:
            self.raw_jpeg = None
            self.preview_jpeg = None

    def _capture_candidates(self):
        """Return default-backend-first capture candidates.

        V4L2 by-id paths are resolved to ``/dev/videoN`` and then opened by
        numeric index.  This avoids OpenCV builds that can open index 2 but
        reject the equivalent path with "backend can't open by name".
        """
        raw = str(self.source).strip()
        if raw.isdigit():
            index = int(raw)
        else:
            resolved = raw if "://" in raw else os.path.realpath(raw)
            match = re.fullmatch(r"/dev/video(\d+)", resolved)
            index = int(match.group(1)) if match else None

        if index is not None:
            return [
                (f"default-index:{index}", (index,)),
                (f"v4l2-index:{index}", (index, cv2.CAP_V4L2)),
            ]

        resolved = raw if "://" in raw else os.path.realpath(raw)
        candidates = [(f"default-source:{resolved}", (resolved,))]
        if resolved.startswith("/dev/"):
            candidates.append(
                (f"v4l2-source:{resolved}", (resolved, cv2.CAP_V4L2))
            )
        return candidates

    def open_capture(self) -> Optional[cv2.VideoCapture]:
        """Open with backend fallback and return only a frame-proven handle."""
        self._startup_frame = None
        self.capture_backend = None
        failures = []

        for label, video_args in self._capture_candidates():
            cap = None
            success = False
            reason = "unknown failure"
            try:
                cap = cv2.VideoCapture(*video_args)
                if cap is None or not cap.isOpened():
                    reason = "backend did not open"
                    continue
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                for _ in range(CAMERA_PROBE_READS):
                    ok, frame = cap.read()
                    if ok and frame is not None and getattr(frame, "size", 0) > 0:
                        self._startup_frame = frame
                        self.capture_backend = label
                        self._capture_open_error = None
                        success = True
                        print(f"[stream_cam] camera verified via {label}: "
                              f"shape={tuple(frame.shape)}")
                        return cap
                    time.sleep(CAMERA_PROBE_DELAY_SEC)
                reason = f"opened but produced no frame in {CAMERA_PROBE_READS} reads"
            except Exception as exc:
                reason = f"{type(exc).__name__}: {exc}"
            finally:
                if cap is not None and not success:
                    try:
                        cap.release()
                    except Exception:
                        pass
                if not success:
                    failures.append(f"{label}: {reason}")

        self._capture_open_error = "; ".join(failures) or "no capture candidates"
        return None

    def reopen_capture(self):
        """Re-resolve the source and retry a frame-proven capture finitely."""
        last_error = "unknown camera error"
        for attempt in range(1, CAMERA_REOPEN_ATTEMPTS + 1):
            if not self.running:
                return None, None, "capture stopping"
            cap = self.open_capture()
            if cap is not None:
                frame = self._startup_frame
                print(f"[stream_cam] camera reopen verified "
                      f"({attempt}/{CAMERA_REOPEN_ATTEMPTS}) via "
                      f"{self.capture_backend}")
                return cap, frame, ""
            last_error = self._capture_open_error or last_error
            print(f"[stream_cam] camera reopen attempt "
                  f"{attempt}/{CAMERA_REOPEN_ATTEMPTS} failed: {last_error}")
            if attempt < CAMERA_REOPEN_ATTEMPTS:
                time.sleep(CAMERA_REOPEN_DELAY_SEC)
        return None, None, last_error

    def capture_loop(self) -> None:
        cap = None
        try:
            cap = self.open_capture()
            if cap is None or not cap.isOpened():
                detail = self._capture_open_error or "backend did not open"
                self.capture_error = (
                    f"cannot open camera source or read a frame: "
                    f"{self.source} ({detail})"
                )
                return

            first_frame = self._startup_frame
            if first_frame is None:
                # Preserves compatibility with injected/custom capture handles
                # while keeping startup tied to a real frame.
                ok, first_frame = cap.read()
                if (not ok or first_frame is None
                        or getattr(first_frame, "size", 0) <= 0):
                    self.capture_error = (
                        f"cannot read first frame from camera source: {self.source}"
                    )
                    return
            self.frame_shape = first_frame.shape

            delay = 1.0 / max(self.fps, 1.0)
            pending_frame = first_frame
            consecutive_read_failures = 0
            while self.running:
                if pending_frame is not None:
                    ok, frame = True, pending_frame
                    pending_frame = None
                else:
                    ok, frame = cap.read()
                if not ok or frame is None:
                    consecutive_read_failures += 1
                    if consecutive_read_failures == 1:
                        self.clear_published_frames()
                        print("[stream_cam] WARN: camera stopped producing frames; "
                              "stale MJPEG publication paused")
                    if consecutive_read_failures >= CAMERA_READ_FAILURES_BEFORE_REOPEN:
                        print(f"[stream_cam] reopening camera after "
                              f"{consecutive_read_failures} consecutive read failures")
                        cap.release()
                        cap = None
                        cap, pending_frame, reopen_error = self.reopen_capture()
                        if cap is None:
                            self.capture_error = (
                                "camera stream lost and reopen failed: "
                                f"{reopen_error}"
                            )
                            print(f"[stream_cam] ERROR: {self.capture_error}")
                            self.running = False
                            break
                        consecutive_read_failures = 0
                        continue
                    time.sleep(0.05)
                    continue
                consecutive_read_failures = 0

                self.frame_shape = frame.shape
                preview = frame.copy()
                found = False
                corners = None
                if self.pattern is not None:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    found, corners = cv2.findChessboardCorners(
                        gray,
                        self.pattern,
                        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
                    )
                    if found:
                        cv2.drawChessboardCorners(preview, self.pattern, corners, found)

                status = "corners: OK" if found else "corners: searching"
                next_sequence = self.frame_sequence + 1
                cv2.putText(preview, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0) if found else (0, 180, 255), 2)
                cv2.putText(preview, f"frame {next_sequence}",
                            (max(10, frame.shape[1] - 180), 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(preview, f"source {self.source}  {frame.shape[1]}x{frame.shape[0]}",
                            (10, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 255, 255), 2)

                ok_raw, raw_buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                ok_prev, prev_buf = cv2.imencode(".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if ok_raw and ok_prev:
                    with self.lock:
                        self.raw_jpeg = raw_buf.tobytes()
                        self.preview_jpeg = prev_buf.tobytes()
                        self.frame_sequence += 1
                        self.last_frame_time = time.monotonic()
                    if not self.startup.is_set():
                        self.startup.set()
                time.sleep(delay)
        except Exception as exc:
            self.capture_error = f"camera capture failed: {exc}"
            print(f"[stream_cam] ERROR: {self.capture_error}")
        finally:
            self.running = False
            self.clear_published_frames()
            self.startup.set()
            if cap is not None:
                cap.release()


def make_handler(state: CameraState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path in ("/", "/preview.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                body = (
                    "<html><head><title>X3Plus camera preview</title></head>"
                    "<body style='margin:0;background:#111;color:white;font-family:sans-serif'>"
                    "<div style='padding:8px;line-height:1.45'>"
                    "<b>X3Plus camera preview</b><br>"
                    "Click the object ground-contact point. Raw pixel: "
                    "<span id='coord' style='font-size:22px;color:#7CFF7C'>x=?, y=?</span><br>"
                    "Use <code>/preview</code> for overlay MJPEG, <code>/stream</code> for raw calibration input."
                    "</div>"
                    "<div style='position:relative;display:inline-block;margin:0 8px 8px 8px'>"
                    "<img id='cam' src='/preview' style='width:960px;max-width:calc(100vw - 16px);height:auto;display:block;cursor:crosshair'>"
                    "<div id='mark' style='display:none;position:absolute;width:15px;height:15px;"
                    "border:2px solid #ff4040;border-radius:50%;transform:translate(-50%,-50%);"
                    "pointer-events:none;box-sizing:border-box'></div>"
                    "</div>"
                    "<script>"
                    "const img=document.getElementById('cam');"
                    "const coord=document.getElementById('coord');"
                    "const mark=document.getElementById('mark');"
                    "img.addEventListener('click', e=>{"
                    " const r=img.getBoundingClientRect();"
                    " const sx=img.naturalWidth/r.width;"
                    " const sy=img.naturalHeight/r.height;"
                    " const x=Math.round((e.clientX-r.left)*sx);"
                    " const y=Math.round((e.clientY-r.top)*sy);"
                    " coord.textContent=`x=${x}, y=${y}`;"
                    " mark.style.display='block';"
                    " mark.style.left=(e.clientX-r.left)+'px';"
                    " mark.style.top=(e.clientY-r.top)+'px';"
                    "});"
                    "</script>"
                    "</body></html>"
                )
                self.wfile.write(body.encode("utf-8"))
                return

            if self.path.startswith("/stream"):
                self._serve_mjpeg(raw=True)
                return
            if self.path.startswith("/preview"):
                self._serve_mjpeg(raw=False)
                return

            self.send_error(404)

        def _serve_mjpeg(self, raw: bool) -> None:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_sequence = -1
            while True:
                with state.lock:
                    jpg = state.raw_jpeg if raw else state.preview_jpeg
                    sequence = state.frame_sequence
                    running = state.running
                if jpg is None or sequence == last_sequence:
                    if not running:
                        break
                    time.sleep(0.01)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    last_sequence = sequence
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break

    return Handler


def main() -> int:
    p = argparse.ArgumentParser(description="X3Plus MJPEG camera preview/stream server")
    p.add_argument("--device", default="0",
                   help="camera index, /dev/videoN, /dev/v4l/by-id symlink, or URL")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8081)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--cols", type=int, default=9, help="inner chessboard corners per row")
    p.add_argument("--rows", type=int, default=6, help="inner chessboard corners per column")
    p.add_argument("--no-corners", action="store_true", help="disable chessboard overlay")
    args = p.parse_args()

    pattern = None if args.no_corners else (args.cols, args.rows)
    state = CameraState(args.device, args.width, args.height, args.fps, pattern)
    thread = threading.Thread(target=state.capture_loop, daemon=True)
    thread.start()
    if not state.startup.wait(timeout=3.0):
        state.running = False
        print("[stream_cam] camera startup timed out")
        return 2
    if state.capture_error is not None:
        state.running = False
        thread.join(timeout=2.0)
        print(f"[stream_cam] {state.capture_error}")
        return 2
    if state.raw_jpeg is None:
        state.running = False
        thread.join(timeout=2.0)
        print("[stream_cam] camera startup signalled without a published frame")
        return 2

    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    server.timeout = 0.5
    print(f"[stream_cam] source={args.device} shape={state.frame_shape}")
    print(f"[stream_cam] preview: http://<jetson-ip>:{args.port}/preview")
    print(f"[stream_cam] raw:     http://127.0.0.1:{args.port}/stream")
    try:
        while state.running:
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        server.server_close()
        thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
