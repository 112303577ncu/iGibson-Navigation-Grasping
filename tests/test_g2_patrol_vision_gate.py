import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, MagicMock
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('patrol_vision_gate',ROOT/'deploy/ros/g2_map_patrol.py')
patrol=importlib.util.module_from_spec(spec);spec.loader.exec_module(patrol)

class VisionPermitTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.path=Path(self.folder.name)/'permit.json'
        self.runner=patrol.Runner.__new__(patrol.Runner)
        self.runner.pid=7;self.runner.vision_boot='boot';self.runner.vision_permit=str(self.path)
        self.runner.end=-1
    def check(self, record, error, message):
        if record is not None:self.path.write_text(json.dumps(record))
        with patch.object(patrol.time,'time',return_value=100),self.assertRaisesRegex(error,message):
            self.runner.state()
    def test_fresh_clear_permit_passes_to_normal_navigation_checks(self):
        self.check(dict(owner_pid=7,boot_id='boot',source_stamp=99.8,allow=True),RuntimeError,'patrol duration')
    def test_candidate_blocks_even_with_fresh_image(self):
        self.check(dict(owner_pid=7,boot_id='boot',source_stamp=99.8,allow=False),RuntimeError,'stopped confirmation')
    def test_missing_or_old_or_future_image_stops(self):
        self.check(None,patrol.Transient,'unavailable')
        for stamp in (98.7,100.1):
            self.check(dict(owner_pid=7,boot_id='boot',source_stamp=stamp,allow=True),patrol.Transient,'source stale')
    def test_new_owner_or_boot_cannot_reuse_permission(self):
        for owner,boot in [(8,'boot'),(7,'other')]:
            self.check(dict(owner_pid=owner,boot_id=boot,source_stamp=99.8,allow=True),RuntimeError,'context changed')
    def test_candidate_never_sends_nonzero_pulse(self):
        self.path.write_text(json.dumps(dict(owner_pid=7,boot_id='boot',source_stamp=99.8,allow=False)))
        r=self.runner;r.report={'pulses':0};r.sock=MagicMock();r.ready=lambda: {'odom':[0,0,0]}
        with patch.object(patrol.time,'time',return_value=100),self.assertRaisesRegex(RuntimeError,'stopped confirmation'):
            r.pulse(.15,0,.08)
        self.assertEqual(r.report['pulses'],1)
        self.assertTrue(all(json.loads(call.args[0])['action']=='stop' for call in r.sock.sendall.call_args_list))

class CovarianceSampleTests(unittest.TestCase):
    def test_negative_sample_waits_without_treating_it_as_valid(self):
        import threading
        sensor=patrol.Sensors.__new__(patrol.Sensors)
        sensor.lock=threading.Lock();sensor.scan=(100,100,'laser',[(0,1)])
        sensor.odom=(100,100,'odom','base_footprint',[0,0,0],0,0)
        cov=[0.]*36;cov[7]=-1e-15;sensor.amcl=(100,100,'map',cov)
        with patch.object(patrol.time,'time',return_value=100),self.assertRaisesRegex(patrol.Transient,'negative covariance sample'):
            sensor.state()


class StopFeedbackTests(unittest.TestCase):
    def test_zero_feedback_can_be_confirmed_without_authorizing_motion(self):
        import io
        for allow, stamp in ((True, 90.), (False, 100.)):
            clock=[100.]
            r=patrol.Runner.__new__(patrol.Runner)
            r.pid=7;r.vision_boot='boot';r.vision_permit='permit'
            r.end=200;r.last_health=100;r.report={};r.sock=MagicMock()
            r.policy=MagicMock();r.policy.allowed.return_value=True
            r.sensors=MagicMock()
            r.sensors.state.side_effect=lambda:dict(odom_stamp=clock[0],vx=0.,wz=0.,pose=[0,0,0])
            def opened(path,*args):
                if str(path)=='permit':
                    return io.StringIO(json.dumps(dict(owner_pid=7,boot_id='boot',source_stamp=stamp,allow=allow)))
                return io.StringIO(json.dumps(dict(owner_pid=7,ready=True,stamp=clock[0])))
            with patch.object(patrol,'open',side_effect=opened,create=True), \
                    patch.object(patrol.time,'time',side_effect=lambda:clock[0]), \
                    patch.object(patrol.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                    patch.object(patrol,'available_mb',return_value=1500):
                self.assertEqual(r.settle()['vx'],0.)
                with self.assertRaises(RuntimeError):r.pulse(.15,0,.08)
            self.assertTrue(all(json.loads(c.args[0])['action']=='stop' for c in r.sock.sendall.call_args_list))

    def test_stop_still_requires_actual_zero_velocity_feedback(self):
        clock=[100.];r=patrol.Runner.__new__(patrol.Runner);r.sock=MagicMock()
        r.state=lambda check_vision=True:dict(odom_stamp=clock[0],vx=.02,wz=0.)
        with patch.object(patrol.time,'time',side_effect=lambda:clock[0]), \
                patch.object(patrol.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            with self.assertRaisesRegex(patrol.Transient,'stop feedback timeout'):r.settle()
        self.assertTrue(all(json.loads(c.args[0])['action']=='stop' for c in r.sock.sendall.call_args_list))

    def test_stale_localization_cannot_become_a_stop_confirmation(self):
        clock=[100.];r=patrol.Runner.__new__(patrol.Runner);r.sock=MagicMock()
        r.state=MagicMock(side_effect=patrol.Transient('stale scan/odom'))
        with patch.object(patrol.time,'time',side_effect=lambda:clock[0]), \
                patch.object(patrol.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            with self.assertRaisesRegex(patrol.Transient,'stale scan/odom'):r.settle()

if __name__=='__main__':unittest.main()
