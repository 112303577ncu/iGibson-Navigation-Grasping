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


def restart_pose(seed, odom, now, boot_id, owner_pid, map_hash, external_pending=False):
    """A new independent reference has 120s; internal last-good recovery has 15s.

    The supervisor clears external_pending after the first validated AMCL pose.
    Neither case permits resetting reference timestamps or relaxing motion bounds.
    """
    return propagated_seed(seed, odom, now, boot_id, owner_pid, map_hash,
                           120 if external_pending else 15)


class RestartBudget(object):
    def __init__(self):
        self.attempts=[]

    def take(self, now):
        self.attempts=[t for t in self.attempts if now-t<600]
        if len(self.attempts)>=3 or (self.attempts and now-self.attempts[-1]<10):
            return False
        self.attempts.append(now)
        return True

    def retry_after(self, now):
        recent=[t for t in self.attempts if now-t<600]
        cooldown=max(0.,10-(now-recent[-1])) if recent else 0.
        window=max(0.,600-(now-recent[0])) if len(recent)>=3 else 0.
        return max(cooldown,window)

    def restore(self, data, now, boot_id):
        if data.get('boot_id')!=boot_id:return
        values=data['attempts']
        if not isinstance(values,list) or len(values)>3 or any(
                not isinstance(v,(int,float)) or math.isnan(v) or math.isinf(v) or v>now for v in values):
            raise ValueError('invalid persisted AMCL restart budget')
        if values!=sorted(values):raise ValueError('unordered AMCL restart budget')
        self.attempts=[v for v in values if now-v<600]


class FaultDelay(object):
    def __init__(self, seconds=8):
        self.seconds=seconds;self.since=None

    def update(self, failed, now):
        if not failed:
            self.since=None
            return False
        if self.since is None:self.since=now
        return now-self.since>=self.seconds


def fault_category(reason):
    """Stable machine-readable causes for existing ROS/resident diagnostics."""
    reason=reason or ''
    if any(text in reason for text in ('seed','reference required','owner changed','localization jump')):
        return 'REFERENCE_REQUIRED','obtain_fresh_landmark_reference'
    if 'RAM' in reason:return 'RAM_PRESSURE','wait_for_memory'
    if 'budget/cooldown' in reason:return 'RESTART_BACKOFF','wait_for_restart_budget'
    if 'stopped resident' in reason:return 'RESIDENT_BUSY','wait_until_stopped'
    if 'board' in reason or 'feedback' in reason:return 'BOARD_STALE','wait_for_board_feedback'
    if 'scan' in reason:return 'SCAN_STALE','wait_for_fresh_scan'
    if 'odom' in reason:return 'ODOM_STALE','wait_for_fresh_odom'
    if 'TF' in reason:return 'TF_STALE','wait_for_fresh_tf'
    if 'uncertainty' in reason:return 'AMCL_UNCERTAIN','wait_for_localization'
    if 'AMCL' in reason:return 'AMCL_STALE','wait_or_restart_amcl'
    return 'SUPERVISOR_FAULT','inspect_localization_diagnostics'


class RecoveryWatchdog(object):
    """Bound an entire recovery episode, even if individual errors keep changing.

    Degraded is a visible non-ready state, not permission to drive on stale data.
    The daemon keeps watching and can recover when valid inputs/reference return.
    """
    def __init__(self, wait_seconds=20):
        if not 5<=wait_seconds<=60:raise ValueError('wait limit must be 5..60 seconds')
        self.limit=wait_seconds;self.failed_since=None;self.last_good=None

    def annotate(self, report, monotonic_now):
        if report.get('ready'):
            self.failed_since=None;self.last_good=monotonic_now
            report.update(reason_code='OK',next_action='none',wait_elapsed_s=0.,
                          wait_limit_s=self.limit,wait_timed_out=False,degraded=False,
                          last_good_age_s=0.)
        else:
            if self.failed_since is None:self.failed_since=monotonic_now
            elapsed=max(0.,monotonic_now-self.failed_since)
            if report.get('status')=='starting_localization':
                code,action='AMCL_STARTING','wait_for_new_amcl_pose'
            else:code,action=fault_category(report.get('reason'))
            timed_out=elapsed>=self.limit
            degraded=timed_out or code=='REFERENCE_REQUIRED'
            report.update(reason_code=code,next_action=action,wait_elapsed_s=round(elapsed,3),
                          wait_limit_s=self.limit,wait_timed_out=timed_out,degraded=degraded,
                          last_good_age_s=None if self.last_good is None else round(max(0.,monotonic_now-self.last_good),3))
            if degraded:
                report['waiting_status']=report.get('status')
                report['status']='needs_reference' if code=='REFERENCE_REQUIRED' else 'degraded'
        return report


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
