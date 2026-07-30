import io
import threading
import time
import unittest
from unittest import mock

import numpy as np

import stream_cam


class FakeCapture:
    def __init__(self, opened, reads=()):
        self.opened = opened
        self.reads = list(reads)
        self.released = False
        self.settings = []

    def isOpened(self):
        return self.opened and not self.released

    def set(self, prop, value):
        self.settings.append((prop, value))
        return True

    def read(self):
        if self.reads:
            return self.reads.pop(0)
        return False, None

    def release(self):
        self.released = True


class StoppingCapture(FakeCapture):
    def __init__(self, state):
        super().__init__(True)
        self.state = state

    def read(self):
        self.state.running = False
        return False, None


class StreamCameraCaptureTests(unittest.TestCase):
    def test_numeric_source_tries_default_before_v4l2_and_probes_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        default = FakeCapture(False)
        v4l2 = FakeCapture(True, [(True, frame)])
        state = stream_cam.CameraState("2", 640, 480, 15.0, None)

        with mock.patch.object(stream_cam.cv2, "VideoCapture",
                               side_effect=[default, v4l2]) as constructor:
            cap = state.open_capture()

        self.assertIs(cap, v4l2)
        self.assertIs(state._startup_frame, frame)
        self.assertEqual(state.capture_backend, "v4l2-index:2")
        self.assertTrue(default.released)
        self.assertFalse(v4l2.released)
        self.assertEqual(constructor.call_args_list, [
            mock.call(2),
            mock.call(2, stream_cam.cv2.CAP_V4L2),
        ])

    def test_opened_backend_without_frames_is_released_before_fallback(self):
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        no_frames = FakeCapture(True)
        fallback = FakeCapture(True, [(True, frame)])
        state = stream_cam.CameraState("2", 640, 480, 15.0, None)

        with mock.patch.object(stream_cam.cv2, "VideoCapture",
                               side_effect=[no_frames, fallback]), \
                mock.patch.object(stream_cam.time, "sleep"):
            cap = state.open_capture()

        self.assertIs(cap, fallback)
        self.assertTrue(no_frames.released)
        self.assertEqual(state.capture_backend, "v4l2-index:2")

    def test_by_id_path_resolves_to_video_index(self):
        state = stream_cam.CameraState(
            "/dev/v4l/by-id/usb-camera-video-index0", 640, 480, 15.0, None
        )
        with mock.patch.object(stream_cam.os.path, "realpath",
                               return_value="/dev/video2"):
            candidates = state._capture_candidates()
        self.assertEqual(candidates, [
            ("default-index:2", (2,)),
            ("v4l2-index:2", (2, stream_cam.cv2.CAP_V4L2)),
        ])

    def test_url_source_is_not_rewritten_as_a_local_path(self):
        url = "http://127.0.0.1:8080/stream"
        state = stream_cam.CameraState(url, 640, 480, 15.0, None)
        self.assertEqual(
            state._capture_candidates(),
            [(f"default-source:{url}", (url,))],
        )

    def test_all_failed_candidates_return_none_and_release_handles(self):
        default = FakeCapture(False)
        v4l2 = FakeCapture(False)
        state = stream_cam.CameraState("2", 640, 480, 15.0, None)
        with mock.patch.object(stream_cam.cv2, "VideoCapture",
                               side_effect=[default, v4l2]):
            cap = state.open_capture()
        self.assertIsNone(cap)
        self.assertTrue(default.released)
        self.assertTrue(v4l2.released)
        self.assertIn("default-index:2", state._capture_open_error)
        self.assertIn("v4l2-index:2", state._capture_open_error)

    def test_capture_loop_reopens_after_first_frame_then_read_failures(self):
        first_frame = np.zeros((24, 32, 3), dtype=np.uint8)
        second_frame = np.full((24, 32, 3), 127, dtype=np.uint8)
        state = stream_cam.CameraState("2", 32, 24, 30.0, None)
        initial = FakeCapture(True, [(False, None), (False, None)])
        recovered = StoppingCapture(state)
        calls = []

        def open_for_phase():
            calls.append(len(calls) + 1)
            if len(calls) == 1:
                state._startup_frame = first_frame
                state.capture_backend = "default-index:2"
                return initial
            state._startup_frame = second_frame
            state.capture_backend = "v4l2-index:2"
            return recovered

        with mock.patch.object(state, "open_capture", side_effect=open_for_phase), \
                mock.patch.object(stream_cam,
                                  "CAMERA_READ_FAILURES_BEFORE_REOPEN", 2), \
                mock.patch.object(stream_cam.time, "sleep"):
            state.capture_loop()

        self.assertEqual(calls, [1, 2])
        self.assertEqual(state.frame_sequence, 2)
        self.assertIsNone(state.capture_error)
        self.assertTrue(initial.released)
        self.assertTrue(recovered.released)

    def test_mjpeg_sends_only_when_frame_sequence_advances(self):
        state = stream_cam.CameraState("2", 32, 24, 30.0, None)
        with state.lock:
            state.raw_jpeg = b"first"
            state.preview_jpeg = b"preview-first"
            state.frame_sequence = 1

        handler_class = stream_cam.make_handler(state)
        handler = object.__new__(handler_class)
        handler.wfile = io.BytesIO()
        handler.send_response = lambda *_args, **_kwargs: None
        handler.send_header = lambda *_args, **_kwargs: None
        handler.end_headers = lambda: None
        worker = threading.Thread(target=handler._serve_mjpeg, args=(True,))
        worker.start()

        deadline = time.time() + 1.0
        while (handler.wfile.getvalue().count(b"--frame\r\n") < 1
               and time.time() < deadline):
            time.sleep(0.005)
        time.sleep(0.03)
        self.assertEqual(handler.wfile.getvalue().count(b"--frame\r\n"), 1)

        with state.lock:
            state.raw_jpeg = b"second"
            state.frame_sequence = 2
        deadline = time.time() + 1.0
        while (handler.wfile.getvalue().count(b"--frame\r\n") < 2
               and time.time() < deadline):
            time.sleep(0.005)
        state.running = False
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        payload = handler.wfile.getvalue()
        self.assertEqual(payload.count(b"--frame\r\n"), 2)
        self.assertIn(b"first", payload)
        self.assertIn(b"second", payload)


if __name__ == "__main__":
    unittest.main()
