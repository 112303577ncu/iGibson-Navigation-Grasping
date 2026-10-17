import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"grasp/v23"))
import test_travel_pose as fake

class Controller(fake.Ctl):
    def __init__(self,at,holding=False,fail_close=False):
        super().__init__(at,holding)
        self.cfg.gripper_hw_closed=180.0;self.cfg.bin_rim_height=.06
        self.cfg.close_run_time_ms=120;self.cfg.close_settle_s=.14;self.cfg.close_max_iters=40
        self.preview_calls=0;self.fail_close=fail_close
    def move_guarded_and_verified(self,arm,grip,**kw):
        if kw['label']=='preview-close-empty':
            assert kw['max_iters']>=19  # full150deg close at8deg/command
            if self.fail_close:return {'reached':False,'reason':'blocked'}
            self.servo.deg=list(arm)+[180.0]
            return {'reached':True,'reason':'arrived'}
        result=super().move_guarded_and_verified(arm,grip,**kw)
        if grip==-1.5:self.servo.deg[5]=30.0
        return result
    def _scripted_release(self):
        self.preview_calls+=1;self.servo.deg=list(fake.E1)
        return 'released'

def test_empty_preview_runs_and_returns_travel_without_delivery_claim():
    c=Controller(fake.TRAVEL);r=fake.svc(c).cmd_release_preview()
    assert r['ok'] and r['preview'] and not r['object_delivery_verified']
    assert r['rim_cm']==6 and c.preview_calls==1 and c.released==0
    assert tuple(c.servo.deg[:5])==fake.TRAVEL[:5]
    assert c._grip_hold_rad is None

def test_preview_refuses_held_object_without_moving():
    c=Controller(fake.TRAVEL,holding=True);r=fake.svc(c).cmd_release_preview()
    assert not r['ok'] and c.moves==[] and c.preview_calls==0

def test_preview_refuses_closed_jaw_without_moving():
    c=Controller(list(fake.E1[:5])+[165]);r=fake.svc(c).cmd_release_preview()
    assert not r['ok'] and c.moves==[] and c.preview_calls==0

def test_preview_never_releases_if_empty_close_was_not_confirmed():
    c=Controller(fake.E1,fail_close=True);r=fake.svc(c).cmd_release_preview()
    assert not r['ok'] and c.preview_calls==0

def test_preview_refuses_chassis_without_arm_claim():
    c=Controller(fake.E1);s=fake.svc(c)
    class Chassis:
        def status(self):return {'maintenance':False,'moving':False,'motors':[0,0,0,0]}
    s.chassis=Chassis();r=s.cmd_release_preview()
    assert not r['ok'] and c.moves==[]

def test_preview_accepts_existing_arm_claim_without_maintenance_lock():
    c=Controller(fake.E1);s=fake.svc(c)
    class Chassis:
        def status(self):return {'maintenance':False,'arm_busy':True,'moving':False,'motors':[0,0,0,0]}
    s.chassis=Chassis();r=s.cmd_release_preview()
    assert r['ok'] and c.preview_calls==1
