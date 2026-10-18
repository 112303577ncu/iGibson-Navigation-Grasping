"""Pure Python 2/3 localization restart gates; no actuator access."""
from __future__ import division
import math


def propagated_seed(seed, odom, now, boot_id, owner_pid, map_hash, max_age=15):
    if (seed.get('boot_id') != boot_id or seed.get('owner_pid') != owner_pid
            or seed.get('map_hash') != map_hash):
        raise ValueError('seed boot/owner/map changed')
    if not 0 <= now-seed['stamp'] <= max_age:
        raise ValueError('seed expired; fresh localization reference required')
    p, old = seed['pose'], seed['odom']
    values=list(p)+list(old)+list(odom)
    if len(p)!=3 or len(old)!=3 or len(odom)!=3 or any(math.isnan(v) or math.isinf(v) for v in values):
        raise ValueError('invalid seed coordinates')
    dx,dy=odom[0]-old[0],odom[1]-old[1]
    dyaw=(odom[2]-old[2]+math.pi)%(2*math.pi)-math.pi
    if math.hypot(dx,dy)>.10 or abs(dyaw)>math.radians(10):
        raise ValueError('seed motion exceeded restart allowance')
    angle=p[2]-old[2];c,s=math.cos(angle),math.sin(angle)
    return [p[0]+c*dx-s*dy,p[1]+s*dx+c*dy,(p[2]+dyaw+math.pi)%(2*math.pi)-math.pi]


class RestartBudget(object):
    def __init__(self):
        self.attempts=[]

    def take(self, now):
        self.attempts=[t for t in self.attempts if now-t<600]
        if len(self.attempts)>=3 or (self.attempts and now-self.attempts[-1]<10):
            return False
        self.attempts.append(now)
        return True


class FaultDelay(object):
    def __init__(self, seconds=8):
        self.seconds=seconds;self.since=None

    def update(self, failed, now):
        if not failed:
            self.since=None
            return False
        if self.since is None:self.since=now
        return now-self.since>=self.seconds


def turn_clear(points):
    """Filtered scan, raw laser angle convention; retain close dynamic returns."""
    hits=[]
    for a,r in points:
        x=.10+r*math.cos(a+math.pi);y=r*math.sin(a+math.pi)
        if -.15<=x<=.22 and abs(y)<=.14:continue
        if math.hypot(x,y)<.274:hits.append((a,x,y))
    hits.sort();count=0;last=None
    for hit in hits:
        count=(count+1 if last is not None and hit[0]-last[0]<math.radians(1)
               and math.hypot(hit[1]-last[1],hit[2]-last[2])<.04 else 1)
        if count>=3:return False
        last=hit
    return True
