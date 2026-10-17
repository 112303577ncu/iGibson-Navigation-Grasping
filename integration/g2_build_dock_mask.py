"""Build a reviewable local dock corridor; never enable or validate it."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from g2_patrol_policy import read_pgm


def build(folder):
    folder=Path(folder);layout=json.loads((folder/'scene_layout.json').read_text())
    meta=yaml.safe_load((folder/layout['map']).read_text())
    w,h,raw=read_pgm(str(folder/meta['image']));raw=np.array(raw,dtype=np.uint8).reshape(h,w)
    bw,bh,base=read_pgm(str(folder/'navigation_center_mask.pgm'))
    if (w,h)!=(bw,bh):raise ValueError('base/raw map dimensions differ')
    base=np.array(base,dtype=np.uint8).reshape(h,w)
    resolution=float(meta['resolution']);origin=meta['origin']
    if origin[2]!=0 or meta.get('negate',0)!=0:raise ValueError('unsupported raw map geometry')
    # Unknown and occupied cells are obstacles, including corner gaps.
    clearance=cv2.distanceTransform((raw>250).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
    room=np.array(layout['room_polygon'],np.float32);box=np.array(layout['bin']['footprint_map'],np.float32)
    dock=layout['bin']['dock_candidate'];yaw=dock['yaw'];c,s=math.cos(yaw),math.sin(yaw)
    candidate=np.zeros((h,w),np.uint8)
    # Centre corridor only: 28cm behind to 12cm beyond candidate, half-width12cm.
    # The old40cm mask remains active everywhere else. No expansion through box.
    for row,col in zip(*np.where(clearance>=.30)):
        x=origin[0]+(col+.5)*resolution;y=origin[1]+(h-1-row+.5)*resolution
        dx,dy=x-dock['x'],y-dock['y'];along=c*dx+s*dy;side=-s*dx+c*dy
        if not (-.28<=along<=.12 and abs(side)<=.12):continue
        if cv2.pointPolygonTest(room,(float(x),float(y)),True)<.30:continue
        if cv2.pointPolygonTest(box,(float(x),float(y)),True)>-layout['bin']['center_keepout_buffer_m']:continue
        candidate[row,col]=255
    image='navigation_dock_zone_candidate.pgm'
    with (folder/image).open('wb') as f:f.write(('P5\n%d %d\n255\n'%(w,h)).encode()+candidate.tobytes())
    cfg={'enabled':False,'status':'draft_only_requires_geometry_and_physical_registration','image':image,
         'resolution':resolution,'origin':origin,'center_wall_margin_m':.30,
         'corridor_along_m':[-.28,.12],'corridor_half_width_m':.12,
         'box_keepout_m':layout['bin']['center_keepout_buffer_m'],
         'added_cells':int(((candidate>250)&(base<=250)).sum()),
         'sha256':hashlib.sha256((folder/image).read_bytes()).hexdigest()}
    (folder/'dock_zone_candidate.json').write_text(json.dumps(cfg,indent=2))
    return cfg


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',required=True)
    print(json.dumps(build(p.parse_args().scene),indent=2))
