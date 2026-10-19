import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('camera_capture', ROOT/'deploy/ros/g2_corner_camera_capture.py')
camera = importlib.util.module_from_spec(spec); spec.loader.exec_module(camera)


def runtime(monkeypatch, tmp_path, bad=None):
    clock = [1.]; released = []; source = tmp_path/'native.json'; out = tmp_path/'batch'
    def read():
        clock[0] += .05
        return True, np.zeros((480, 640, 3), dtype=np.uint8)
    cap = NS(isOpened=lambda: True, set=lambda *_: True, release=lambda: released.append(True), read=read,
             get=lambda key: 1 if key == 1 else (0 if bad == 'pts' else (clock[0]-.01)*1000))
    cv2 = NS(CAP_V4L2=0, CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4,
             CAP_PROP_BUFFERSIZE=1, CAP_PROP_POS_MSEC=2, VideoCapture=lambda *_: cap,
             imwrite=lambda path, _: Path(path).write_bytes(b'JPEG'),
             aruco=NS(DICT_4X4_50=0, getPredefinedDictionary=lambda _: None,
                      ArucoDetector=lambda _: NS(detectMarkers=lambda _: (
                          [np.asarray([[[80, 80], [100, 80], [100, 100], [80, 100]]])], np.asarray([[0]]), []))))
    monkeypatch.setitem(sys.modules, 'cv2', cv2)
    monkeypatch.setattr(camera.guard, 'available_mb', lambda: 1500)
    monkeypatch.setattr(camera.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(camera.time, 'time', lambda: 100+clock[0])
    monkeypatch.setattr(camera.subprocess, 'run', lambda *_a, **_k: NS(returncode=0 if bad == 'owner' else 1))
    def child(*_a, **_kw):
        source.write_text(json.dumps(dict(time=100+clock[0], capture_id='capture', boot_id='boot', owner_pid=7,
                                         map_hash='map'), separators=(',', ':')))
        return NS(returncode=0, poll=lambda: 0, communicate=lambda **_: (b'ok', b''))
    monkeypatch.setattr(camera.subprocess, 'Popen', child)
    monkeypatch.setattr(sys, 'argv', ['camera', '--output-dir', str(out), '--registration-id', 'registration',
                                     '--snapshot', str(source)])
    return source, out, released


def test_camera_export_keeps_exact_native_hash_and_source_timestamps(monkeypatch, tmp_path):
    source, out, released = runtime(monkeypatch, tmp_path)
    assert camera.main() == 0 and released
    assert (out/'capture.json').read_bytes() == source.read_bytes()
    obs = json.loads((out/'observations.json').read_text())
    assert len(obs['frames']) == 3 and all(f['source_age_s'] < .02 for f in obs['frames'])
    assert len({f['source_monotonic'] for f in obs['frames']}) == 3
    assert obs['clock'] == 'jetson' and obs['capture_id'] == 'capture'


def test_unknown_buffered_camera_time_is_rejected_and_released(monkeypatch, tmp_path):
    _, out, released = runtime(monkeypatch, tmp_path, 'pts')
    with pytest.raises(RuntimeError, match='source timestamp'):
        camera.main()
    assert released and not (out/'observations.json').exists()


def test_camera_owned_by_another_stream_is_not_opened_twice(monkeypatch, tmp_path):
    _, out, released = runtime(monkeypatch, tmp_path, 'owner')
    with pytest.raises(RuntimeError, match='already owned'):
        camera.main()
    assert not released and not out.exists()
