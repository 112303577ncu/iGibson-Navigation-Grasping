import json
import sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from integration import v23_delivery_navigation as flow


def fake_native(monkeypatch,ok=True):
    import tempfile
    monkeypatch.setitem(sys.modules,'fcntl',SimpleNamespace(LOCK_EX=1,LOCK_NB=2,flock=lambda *_:None))
    def fake_open(path,*args,**kwargs):
        if path=='/tmp/g2_delivery_flow.lock':return tempfile.TemporaryFile(mode='a+')
        return open(path,*args,**kwargs)
    monkeypatch.setattr(flow,'open',fake_open,raising=False)
    def spawn(command):
        output=Path(command[command.index('--output')+1])
        output.write_text(json.dumps({'ok':ok,'error':None if ok else 'localization unavailable'}))
        return SimpleNamespace(wait=lambda **_:0 if ok else 1)
    monkeypatch.setattr(flow.subprocess,'Popen',spawn)


def test_preparation_arrival_never_implies_release_ready(tmp_path,monkeypatch):
    fake_native(monkeypatch);out=tmp_path/'flow.json'
    code=flow.main(['--task','dropoff','--scene',str(tmp_path),'--output',str(out)])
    report=json.loads(out.read_text())
    assert code==2 and report['navigation_ok']
    assert report['next_stage']=='WAIT_VISUAL_TARGET'
    assert not report['mission_complete'] and not report['arm_or_release_executed']
    assert not report.get('release_ready',False)


def test_map_failure_never_starts_vision_or_arm(tmp_path,monkeypatch):
    fake_native(monkeypatch,False);out=tmp_path/'flow.json'
    monkeypatch.setattr(flow.subprocess,'call',lambda *_:(_ for _ in ()).throw(AssertionError('must not start vision')))
    code=flow.main(['--task','dropoff','--scene',str(tmp_path),'--output',str(out),'--visual-config','missing.json'])
    report=json.loads(out.read_text())
    assert code==1 and len(report['stages'])==1
    assert 'localization unavailable' in report['error']


def test_all_legacy_serial_constructors_fail_before_import_or_open():
    import ast
    for name,function in [('sugarbox_rl_motor_server','load_rosmaster'),('ai_motor_server_P0','initialize_rosmaster'),
                          ('calibrate_feedback_odom_linear','_load_rosmaster'),('calibrate_feedback_odom_yaw','_load_rosmaster')]:
        tree=ast.parse((ROOT/'integration'/(name+'.py')).read_text(encoding='utf-8'))
        node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==function)
        assert isinstance(node.body[0],ast.Raise)
