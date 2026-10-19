#!/usr/bin/env python3
"""Offline bounded scan/map hypotheses. No ROS, actuator, or automatic reseed.

Symmetric rooms require an independent landmark/operator cue. A low endpoint
error alone never authorizes choosing one of the four matching corners.
"""
import argparse
import json
import math
from pathlib import Path
from g2_patrol_policy import read_pgm
from g2_aruco_corner_seed import file_hash,map_hash


def main():
    import numpy as np
    import yaml
    from scipy.ndimage import distance_transform_edt,map_coordinates
    from scipy.optimize import differential_evolution
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',required=True);p.add_argument('--scene',required=True);p.add_argument('--output',required=True)
    p.add_argument('--reference-map-yaml',help='Exact active Jetson YAML bytes; local scene supplies identical raster')
    args=p.parse_args();scene=Path(args.scene)
    data=json.loads(Path(args.snapshot).read_text());layout=json.loads((scene/'scene_layout.json').read_text())
    meta=yaml.safe_load((scene/'demo_box.yaml').read_text());image=scene/Path(meta['image']).name
    w,h,raw=read_pgm(str(image));res=float(meta['resolution']);ox,oy,oyaw=meta['origin']
    reference=Path(args.reference_map_yaml) if args.reference_map_yaml else scene/'demo_box.yaml'
    reference_meta=yaml.safe_load(reference.read_text())
    if any(reference_meta.get(k)!=meta.get(k) for k in ('resolution','origin','negate','occupied_thresh','free_thresh')):
        raise ValueError('reference map geometry/thresholds differ from local raster scene')
    if oyaw!=0:raise ValueError('rotated map unsupported')
    grid=np.array(raw).reshape(h,w);distance=distance_transform_edt(grid>65)*res
    scan=data['scan'];r=np.array([np.nan if v is None else v for v in scan['ranges']])
    a=scan['angle_min']+np.arange(len(r))*scan['angle_increment']+math.pi
    keep=np.isfinite(r)&(r>.45)&(r<2.8)
    points=np.column_stack((.10+r[keep]*np.cos(a[keep]),r[keep]*np.sin(a[keep])))[::3]
    if len(points)<100:raise ValueError('insufficient wall observations')
    heading=layout['room_heading_rad'];c,s=math.cos(heading),math.sin(heading)
    room=np.array(layout['room_polygon']);uv=room.dot(np.array([[c,-s],[s,c]]))
    lo=uv.min(axis=0)+.22;hi=uv.max(axis=0)-.22
    def pose(q):return [q[0]*c-q[1]*s,q[0]*s+q[1]*c,q[2]]
    def errors(pose):
        c1,s1=math.cos(pose[2]),math.sin(pose[2]);xy=points.dot(np.array([[c1,s1],[-s1,c1]]))+pose[:2]
        return map_coordinates(distance,np.array([h-1-(xy[:,1]-oy)/res-.5,(xy[:,0]-ox)/res-.5]),order=1,mode='constant',cval=1.)
    def score(q):
        e=np.sort(errors(pose(q)));return float(np.mean(e[:int(.85*len(e))])+.1*np.mean(np.minimum(e,.3)))
    hypotheses=[]
    for i in range(8):
        quadrant=i%4;angle=heading+quadrant*math.pi/2
        fit=differential_evolution(score,[(lo[0],hi[0]),(lo[1],hi[1]),(angle-math.pi/4,angle+math.pi/4)],seed=18+i,popsize=18,maxiter=220,tol=1e-7,polish=True)
        p1=pose(fit.x);p1[2]=(p1[2]+math.pi)%(2*math.pi)-math.pi;e=errors(p1)
        if any(math.hypot(p1[0]-x['pose'][0],p1[1]-x['pose'][1])<.10 and abs((p1[2]-x['pose'][2]+math.pi)%(2*math.pi)-math.pi)<.10 for x in hypotheses):continue
        box=layout['bin']['placement_update']['midpoint_pose_map_candidate']
        bearing=(math.atan2(box['y']-p1[1],box['x']-p1[0])-p1[2]+math.pi)%(2*math.pi)-math.pi
        hypotheses.append({'pose':p1,'score_m':float(fit.fun),'wall_p90_m':float(np.percentile(e,90)),
            'within_5cm_fraction':float(np.mean(e<.05)),'candidate_box_bearing_left_deg':math.degrees(bearing),
            'candidate_box_range_m':math.hypot(box['x']-p1[0],box['y']-p1[1])})
    hypotheses.sort(key=lambda x:x['score_m'])
    out={'accepted':False,'reason':'independent corner/heading reference required; paper-box map position is still a candidate',
         'points':len(points),'hypotheses':hypotheses,'source_stamp':data['time'],'lidar_forward_m':.10,
         'snapshot_sha256':file_hash(args.snapshot),'map_hash':map_hash(reference,image)}
    Path(args.output).write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))


if __name__=='__main__':main()
