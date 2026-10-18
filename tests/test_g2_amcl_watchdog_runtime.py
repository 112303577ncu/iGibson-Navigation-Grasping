"""Run the actual supervisor loop against virtual ROS/processes/time, no hardware."""
import builtins
import importlib.util
import io
import json
import signal
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
sys.path.insert(0,str(ROOT/'deploy/ros'))
spec=importlib.util.spec_from_file_location('amcl_supervisor_runtime',ROOT/'deploy/ros/g2_localization_service.py')
service=importlib.util.module_from_spec(spec);spec.loader.exec_module(service)
from g2_localization_policy import RecoveryWatchdog,RestartBudget


def run_monitor(monkeypatch,tmp_path,seconds=20,scan_gap=None,amcl_gap=None,
                seed_age=0,no_seed=False,new_reference_at=None,memory_gap=None,owner_change=None,
                jump_at=None,cached_amcl=False,cached_seed=None,persisted_budget=None):
    clock=SimpleNamespace(t=0.)
    reports=[];children=[];killed=[];notifications=[]
    seed_path=tmp_path/'seed.json'
    cache_path=tmp_path/'last_good.json';budget_path=tmp_path/'budget.json';health_path=tmp_path/'health.json'
    if cached_seed is not None:cache_path.write_text(json.dumps(cached_seed))
    if persisted_budget is not None:budget_path.write_text(json.dumps(persisted_budget))
    def supply():
        seed_path.write_text(json.dumps(dict(stamp=1000+clock.t-seed_age,boot_id='boot',
            owner_pid=7,map_hash='map',pose=[1,2,0],odom=[0,0,0])))
    if not no_seed:supply()
    def inside(interval):return interval is not None and interval[0]<=clock.t<interval[1]
    def sleep(dt):
        before=clock.t;clock.t=round(clock.t+dt,5)
        if new_reference_at is not None and before<new_reference_at<=clock.t:supply()
    class Sensors:
        def __init__(self):self.lock=threading.Lock();self.amcl=None
        @property
        def odom(self):return (1000+clock.t,1000+clock.t,'odom','base_footprint',[0,0,0],0,0)
        @property
        def scan(self):
            stamp=1000+clock.t-(2 if inside(scan_gap) else 0)
            return (stamp,stamp,'laser',[])
        def state(self):
            if inside(amcl_gap):raise RuntimeError('stale AMCL')
            self.amcl=(999 if cached_amcl else 1000+clock.t,)
            return dict(pose=[1.5 if jump_at is not None and clock.t>=jump_at else 1,2,0],
                        odom=[0,0,0],front=float('inf'),vx=0,wz=0)
    class Child:
        def __init__(self,command,**kwargs):
            self.command=command;self.pid=100+len(children);self.code=None;children.append(self)
        def poll(self):return self.code
        def wait(self):return self.code
    def kill(pid,sig):
        child=next(child for child in children if child.pid==pid)
        child.code=-sig;killed.append(pid)
    def rpc(command):
        assert command=='health','supervisor must never send robot commands'
        return dict(service_pid=8 if owner_change is not None and clock.t>=owner_change else 7,
                    chassis=dict(board_rx_age_s=0,motors=[0,0,0,0],moving=False,arm_busy=False,stopped_for_s=1),
                    odom=dict(valid=True))
    def read(path,*args,**kwargs):
        if str(path)=='/proc/sys/kernel/random/boot_id':return io.StringIO('boot')
        return builtins.open(path,*args,**kwargs)
    monkeypatch.setitem(sys.modules,'rospy',SimpleNamespace(init_node=lambda *a,**k:None,is_shutdown=lambda:clock.t>=seconds))
    monkeypatch.setattr(service,'Sensors',Sensors)
    monkeypatch.setattr(service,'rpc',rpc)
    monkeypatch.setattr(service,'time',SimpleNamespace(time=lambda:1000+clock.t,sleep=sleep))
    monkeypatch.setattr(service,'monotonic_seconds',lambda:500+clock.t)
    monkeypatch.setattr(service,'available_mb',lambda:700 if inside(memory_gap) else 1500)
    monkeypatch.setattr(service,'map_hash',lambda _: 'map')
    monkeypatch.setattr(service,'open',read,raising=False)
    def atomic(path,report):
        value=json.dumps(report,allow_nan=False)
        if str(path)==str(health_path):reports.append(json.loads(value))
        else:Path(path).write_text(value)
    monkeypatch.setattr(service,'atomic',atomic)
    monkeypatch.setattr(service,'systemd_notify',lambda message:notifications.append(message))
    monkeypatch.setattr(service,'signal',SimpleNamespace(signal=lambda *a:None,SIGTERM=signal.SIGTERM,SIGINT=signal.SIGINT,SIGKILL=9))
    monkeypatch.setattr(service.subprocess,'Popen',Child)
    monkeypatch.setattr(service.os,'killpg',kill,raising=False)
    monkeypatch.setattr(service.os,'setsid',lambda:None,raising=False)
    monkeypatch.setattr(sys,'argv',['monitor','--map','unused.yaml','--seed',str(seed_path),
        '--health',str(health_path),'--last-good',str(cache_path),'--restart-budget',str(budget_path)])
    service.main()
    return SimpleNamespace(reports=reports[:-1],children=children,killed=killed,notifications=notifications)


def test_healthy_amcl_keeps_refreshing_reference_beyond_120_seconds(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=180)
    assert len(run.children)==1 and run.reports[-1]['ready']
    assert run.reports[-1]['reference_age_s']==0
    assert run.reports[-1]['wait_elapsed_s']==0
    assert run.notifications[0].startswith('READY=1')


def test_short_scan_outage_pauses_without_restart(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=12,scan_gap=(4,5))
    faults=[r for r in run.reports if r.get('reason_code')=='SCAN_STALE']
    assert faults and all(not r['ready'] for r in faults)
    assert len(run.children)==1 and run.reports[-1]['ready']


def test_persistent_scan_outage_stops_only_owned_amcl_then_recovers(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=20,scan_gap=(3,14))
    assert len(run.children)==2 and run.reports[-1]['ready']
    assert set(run.killed)=={child.pid for child in run.children}


def test_stale_amcl_is_bounded_and_restarted(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=20,amcl_gap=(3,13))
    assert len(run.children)==2 and run.reports[-1]['ready']


def test_unrecoverable_amcl_reports_deadline_and_respects_restart_budget(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=45,amcl_gap=(0,1000))
    assert len(run.children)==3
    assert run.reports[-1]['reason_code']=='RESTART_BACKOFF'
    assert run.reports[-1]['status']=='degraded'
    assert run.reports[-1]['wait_timed_out'] and run.reports[-1]['retry_after_s']>500
    assert not any(r['ready'] for r in run.reports)


def test_missing_seed_never_guesses_but_new_reference_can_resume(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=30,no_seed=True,new_reference_at=25)
    earlier=[r for r in run.reports if r['stamp']<1025]
    assert all(not r['ready'] and r['status']=='needs_reference' for r in earlier)
    assert any(r['wait_timed_out'] for r in earlier)
    assert len(run.children)==1 and run.reports[-1]['ready']


def test_external_reference_60_seconds_old_starts_successfully(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=3,seed_age=60)
    assert len(run.children)==1 and run.reports[-1]['ready']


def test_old_or_wrong_boot_reference_does_not_start_amcl(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=22,seed_age=121)
    assert not run.children and run.reports[-1]['reason_code']=='REFERENCE_REQUIRED'
    assert run.reports[-1]['wait_timed_out']


def test_memory_pressure_stops_amcl_without_touching_resident(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=16,memory_gap=(3,5))
    faults=[r for r in run.reports if r['reason_code']=='RAM_PRESSURE']
    assert faults and faults[0]['amcl_pid'] is None
    assert len(run.children)==2 and run.reports[-1]['ready']


def test_owner_change_invalidates_reference_and_never_restarts_board(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=25,owner_change=5)
    assert len(run.children)==1
    assert run.reports[-1]['status']=='needs_reference' and not run.reports[-1]['ready']


def test_cached_amcl_cannot_satisfy_post_restart_readiness(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=25,cached_amcl=True)
    assert not any(r['ready'] for r in run.reports)


@pytest.mark.parametrize('age,starts',[(2,True),(16,False)])
def test_crashed_supervisor_only_restores_recent_verified_pose(monkeypatch,tmp_path,age,starts):
    cache=dict(stamp=1000-age,boot_id='boot',owner_pid=7,map_hash='map',pose=[1,2,0],
               odom=[0,0,0],consumed_external_stamp=900)
    run=run_monitor(monkeypatch,tmp_path,seconds=3,no_seed=True,cached_seed=cache)
    assert bool(run.children)==starts and run.reports[-1]['ready']==starts


def test_supervisor_restart_does_not_reset_exhausted_amcl_budget(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=25,
        persisted_budget=dict(boot_id='boot',attempts=[480,490,499]))
    assert not run.children and run.reports[-1]['reason_code']=='RESTART_BACKOFF'
    assert run.reports[-1]['retry_after_s']>500


def test_pose_jump_never_refreshes_last_good_reference(monkeypatch,tmp_path):
    run=run_monitor(monkeypatch,tmp_path,seconds=20,jump_at=5)
    ready=[r for r in run.reports if r['ready']]
    assert ready and all(r['seed']['pose']==[1,2,0] for r in ready)
    assert not run.reports[-1]['ready']


def test_changing_faults_cannot_reset_recovery_deadline():
    monitor=RecoveryWatchdog(20)
    for i,reason in enumerate(['stale AMCL','stale scan','stale map TF']):
        out=monitor.annotate(dict(ready=False,status='waiting',reason=reason),i*10)
    assert out['status']=='degraded' and out['wait_elapsed_s']==20
    assert monitor.annotate(dict(ready=True),21)['wait_elapsed_s']==0


def test_restart_retry_time_respects_both_cooldown_and_window():
    b=RestartBudget();assert b.take(0)
    assert b.retry_after(2)==8
    assert b.take(10) and b.take(20)
    assert b.retry_after(25)==575 and b.retry_after(601)==0


def test_systemd_watchdog_covers_stalled_supervisor():
    unit=(ROOT/'deploy/systemd/x3plus-g2-localization.service').read_text()
    assert 'Type=notify' in unit and 'WatchdogSec=15' in unit
    assert 'TimeoutStartSec=20' in unit and 'LimitCORE=0' in unit
