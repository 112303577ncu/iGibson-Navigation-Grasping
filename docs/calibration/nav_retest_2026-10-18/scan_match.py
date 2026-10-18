import json, math, sys
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import differential_evolution
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'integration'))
from g2_patrol_policy import read_pgm
scene=ROOT/'maps/demo_box_20261017'
layout=json.loads((scene/'scene_layout.json').read_text(encoding='utf-8'))
folder=ROOT/'docs/calibration/nav_retest_2026-10-18'
data=json.loads((folder/(sys.argv[1] if len(sys.argv)>1 else 'boot_sensors.json')).read_text())
w,h,raw=read_pgm(str(scene/'demo_box.pgm'))
grid=np.array(raw,dtype=np.uint8).reshape(h,w)
distance=distance_transform_edt(grid>65)*.02
scan=data['scan'];r=np.array([np.nan if v is None else v for v in scan['ranges']])
a=scan['angle_min']+np.arange(len(r))*scan['angle_increment']+math.pi
keep=np.isfinite(r)&(r>.45)&(r<2.8)
points=np.column_stack((.10+r[keep]*np.cos(a[keep]),r[keep]*np.sin(a[keep])))[::3]
def errors(p):
 c,s=math.cos(p[2]),math.sin(p[2]);xy=points.dot(np.array([[c,s],[-s,c]]))+p[:2]
 return map_coordinates(distance,np.array([h-1-(xy[:,1]+2)/.02-.5,(xy[:,0]+2)/.02-.5]),order=1,mode='constant',cval=1.)
def score(p):
 e=np.sort(errors(p));return float(np.mean(e[:int(.85*len(e))])+.1*np.mean(np.minimum(e,.3)))
home=layout['patrol_candidate_ccw'][0];box=layout['bin']['placement_update']['midpoint_pose_map_candidate']
results=[]
for i in range(4):
 fit=differential_evolution(score,[(home[0]-.35,home[0]+.35),(home[1]-.35,home[1]+.35),(-math.pi+i*math.pi/2,-math.pi+(i+1)*math.pi/2)],seed=18+i,popsize=15,maxiter=180,tol=1e-7,polish=True)
 p=fit.x;e=errors(p);bearing=(math.atan2(box['y']-p[1],box['x']-p[0])-p[2]+math.pi)%(2*math.pi)-math.pi
 results.append({'pose':p.tolist(),'score_m':fit.fun,'wall_median_m':float(np.median(e)),'wall_p90_m':float(np.percentile(e,90)),'within_5cm_fraction':float(np.mean(e<.05)),'candidate_box_bearing_left_deg':math.degrees(bearing),'consistent_with_operator_left_front':bool(0<bearing<math.pi/2)})
results.sort(key=lambda x:x['score_m'])
out={'source':'fresh stationary scan; operator near patrol home and bin left-front','lidar_forward_m':.10,'fitting_points':len(points),'candidates':results}
(folder/(sys.argv[2] if len(sys.argv)>2 else 'initial_scan_match.json')).write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
