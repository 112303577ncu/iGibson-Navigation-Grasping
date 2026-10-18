import json, math, shutil, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'integration'))
from g2_patrol_policy import Policy,read_pgm
source=ROOT/'maps/demo_box_20261017';dest=ROOT/'maps/demo_box_20261018_patrol'
dest.mkdir(exist_ok=True)
for name in ['demo_box.pgm','demo_box.yaml','navigation_center_mask.yaml']:
 shutil.copy2(source/name,dest/name)
layout=json.loads((source/'scene_layout.json').read_text(encoding='utf-8'))
w,h,raw=read_pgm(str(source/'navigation_center_mask.pgm'));mask=np.array(raw,dtype=np.uint8).reshape(h,w)
a=layout['room_heading_rad'];c,s=math.cos(a),math.sin(a)
uv=np.array([[x*c+y*s,-x*s+y*c] for x,y in layout['room_polygon']]);u1,v1=uv.max(axis=0)
# Full operator-measured 20..25cm wall-gap envelope, plus 4cm registration allowance.
u0,u2=u1-.25-.135-.04,u1-.20+.04
v0,v2=v1-.25-.18-.04,v1-.20+.04
rr,cc=np.indices((h,w));x=-2+(cc+.5)*.02;y=-2+(h-1-rr+.5)*.02
u,v=x*c+y*s,-x*s+y*c
du=np.maximum(np.maximum(u0-u,0),u-u2);dv=np.maximum(np.maximum(v0-v,0),v-v2)
blocked=np.hypot(du,dv)<layout['bin']['center_keepout_buffer_m']
removed=int(np.count_nonzero((mask>250)&blocked));mask[blocked]=0
(dest/'navigation_center_mask.pgm').write_bytes(('P5\n%d %d\n255\n'%(w,h)).encode()+mask.tobytes())
match=json.loads((ROOT/'docs/calibration/nav_retest_2026-10-18/reposition_scan_match.json').read_text())['candidates'][0]
assert match['within_5cm_fraction']>.9 and match['consistent_with_operator_left_front']
layout['state']='supervised_retest_prepared_dynamic_validation_pending'
layout['sensor_geometry']['validation']={'verified':True,'lidar_forward_m':.10,'scope':'stationary scan-map and live TF compatibility; supervised dynamic test pending','evidence':'docs/calibration/nav_retest_2026-10-18/reposition_scan_match.json','fraction_within_5cm':match['within_5cm_fraction']}
layout['navigation_mask_valid_for_current_bin_placement']=True
layout['navigation_mask_validation']={'method':'original known-free 40cm mask intersected with conservative moved-box envelope exclusion','wall_gap_range_m':[.20,.25],'additional_registration_allowance_m':.04,'envelope_room_uv':[u0,u2,v0,v2],'removed_free_cells':removed,'exact_box_pose_registered':False,'source':'operator retained original corner, wall-gap measurements; 2026-10-18 reply after reposition'}
layout['bin']['dock_zone']['enabled']=False
layout['bin']['registration_verified']=False
layout['bin']['pose_map_valid_for_current_placement']=False
layout['patrol_inset_from_fitted_wall_m']=.70
layout['patrol_verification']='pending current run; older source results do not validate this scene'
(dest/'scene_layout.json').write_text(json.dumps(layout,indent=2),encoding='utf-8')
policy=Policy(str(dest));points=policy.patrol_points(.70)
assert policy.allowed(*match['pose'][:2])
goal=points[2]
goals=['home',{'name':'dropoff_preparation','point':list(goal)},'home']
(dest/'navigation_test_goals.json').write_text(json.dumps(goals,indent=2))
print(json.dumps({'scene':str(dest),'removed_cells':removed,'points':points,'preparation_goal':goal,'current_allowed':True,'exact_docking_enabled':False}))
