#!/usr/bin/env python3
import sys,subprocess,json,time,dataclasses,os
ROOT='/home/jetson/Documents/deploy_jetson2'
sys.path.insert(0,ROOT+'/integration');sys.path.insert(0,ROOT+'/deploy/ros')
from g2_mapping_route import NativeSensors
from g2_quarter_turn_calibration import swept_hazard,EnvelopeGate
from g2_map_patrol import Policy,rpc,available_mb
folder='/tmp/g2_wall_survey_final';os.makedirs(folder,exist_ok=True)
r={'ok':False,'steps':[],'start':time.time(),'reason':None};camera=None
try:
 h=rpc('health');assert h['chassis']['motors']==[0,0,0,0] and h['chassis']['client'] is None
 assert h['chassis']['arm_pose']=='travel' and not h['chassis']['arm_busy']
 assert available_mb()>=1024
 loc=json.load(open('/dev/shm/g2_localization_health.json'));assert loc['ready'] and time.time()-loc['stamp']<.5
 p=Policy(ROOT+'/maps/demo_box_20261018_patrol');pose=loc['state']['pose']
 import math
 assert all(p.allowed(pose[0]+.06*math.cos(i*math.pi/8),pose[1]+.06*math.sin(i*math.pi/8)) for i in range(16)),'turn reserve unavailable'
 sensors=NativeSensors('/dev/shm/g2_wall_native.json');sensors.wait_ready();scan,odom,_,error=sensors.snapshot();assert not error
 import importlib.util
 spec=importlib.util.spec_from_file_location('wall_session','/tmp/g2_wall_quarter_session.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 gate=mod.SessionGate(True,True)
 for _ in range(8):
  scan,odom,_,error=sensors.snapshot();assert not error
  assert not gate.check(scan),'persistent swept envelope blocked'
  time.sleep(.1)
 camera=subprocess.Popen(['/home/jetson/grasp_venv/bin/python3','/tmp/g2_rear_camera_audit.py','--output',folder+'/camera','--seconds','150'])
 time.sleep(2)
 for i in range(4):
  scan,o,_,e=sensors.snapshot();assert not e
  reference=folder+'/reference_%d.json'%i
  with open(reference,'w') as f:json.dump({'start_odom':dataclasses.asdict(o),'final_odom':dataclasses.asdict(o)},f)
  output=folder+'/turn_%d.json'%i
  command=['/home/jetson/grasp_venv/bin/python3','/tmp/g2_wall_quarter_session.py','--reference',reference,'--output',output,'--native-snapshot','/dev/shm/g2_wall_native.json','--confirmed-rear-structure','--confirmed-right-structure','--target-deg','30','--direction','left','--real']
  result=subprocess.run(command,timeout=75)
  with open(output) as f:t=json.load(f)
  r['steps'].append({'index':i,'ok':t['ok'],'degrees':t.get('final_accumulated_deg'),'error':t.get('error')})
  with open(folder+'/survey.json','w') as f:json.dump(r,f)
  if result.returncode or not t['ok']:raise RuntimeError('bounded turn failed: '+str(t.get('error')))
  if camera.poll() is not None:raise RuntimeError('camera stopped during survey')
  time.sleep(1)
 r['ok']=True
except BaseException as exc:r['reason']=str(exc)
finally:
 if camera is not None and camera.poll() is None:
  camera.terminate()
  try:camera.wait(timeout=5)
  except subprocess.TimeoutExpired:camera.kill();camera.wait()
 r['end']=time.time();r['final_health']=rpc('health')
 with open(folder+'/survey.json','w') as f:json.dump(r,f)
 print(json.dumps({k:r[k] for k in ['ok','steps','reason']}),flush=True)
