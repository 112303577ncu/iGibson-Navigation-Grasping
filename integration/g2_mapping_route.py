"""Supervised mapping steps via the resident owner; not autonomous patrol.

Straight segments retain g2_nav_client's motor-30 bounds and braking. Left
turns use settled pulses and the existing swept envelope. A session-only
operator override covers confirmed near-body returns, never forward walls.
"""
import argparse
import dataclasses
import json
import inspect
import math
from pathlib import Path
import time

try:
    from .g2_geometry import LIDAR_FORWARD_M
    from . import g2_nav_client as nav
    from . import g2_turn_probe as turn
    from . import g2_quarter_turn_calibration as quarter
    from .g2_filtered_turn_calibration import rpc, available_mb
except ImportError:
    from g2_geometry import LIDAR_FORWARD_M
    import g2_nav_client as nav
    import g2_turn_probe as turn
    import g2_quarter_turn_calibration as quarter
    from g2_filtered_turn_calibration import rpc, available_mb


def config():
    return nav.nr.NavRLConfig(lidar_yaw_offset_deg=180,
                             lidar_forward_offset_m=LIDAR_FORWARD_M,
                             safety_brake_dist=.50)


class NativeSensors:
    def __init__(self,path):
        self.path=Path(path)

    def snapshot(self):
        try:
            data=json.loads(self.path.read_text())
            now,wall=time.monotonic(),time.time()
            received=now-(wall-data['written_at'])
            a=data['scan']
            scan=nav.Scan(tuple(tuple(p) for p in a['points']),received,a['stamp'],a['frame'])
            odom=nav.parse_odom(data['odom'],now=received)
            return scan,odom,None,''
        except (OSError,ValueError,KeyError,TypeError) as exc:
            return None,None,None,'native snapshot: '+str(exc)

    def wait_ready(self):
        checked(self,time.monotonic()+5)

    def close(self):
        pass


def checked(sensors, deadline):
    """Wait stopped for transient telemetry loss, without renewing motion."""
    while True:
        scan, odom, _, error = sensors.snapshot()
        # Turning/idle readiness depends on fresh telemetry, not front clearance.
        reason, _ = nav.health(scan, odom, dataclasses.replace(config(), safety_brake_dist=0))
        if not error and not reason:
            if available_mb() < 768:
                raise RuntimeError('RAM below 768 MB')
            if deadline-time.monotonic()>.05 and (
                    time.monotonic()-scan.received_at>.16 or
                    time.time()-scan.stamp>.32):
                time.sleep(.01)
                continue
            return scan, odom
        if time.monotonic() >= deadline:
            raise RuntimeError('telemetry: ' + str(error or reason))
        time.sleep(.04)


def turn_progress(origin, current):
    return math.degrees(turn.yaw_delta(origin, current))


def bounded_pose(origin, odom):
    angle = turn_progress(origin.yaw, odom.yaw)
    drift = math.hypot(odom.x-origin.x, odom.y-origin.y)
    if not -1.5 <= angle <= 95 or drift > .05:
        raise RuntimeError('rotation yaw/translation limit')
    return angle


def park_report(report):
    """A stop can be sent before the board's settled feedback arrives."""
    deadline=time.monotonic()+3
    try:
        while True:
            state=rpc('health').get('chassis',{})
            if state.get('motors')==[0,0,0,0] and not state.get('moving'):
                try:
                    rpc('maintenance-lock')
                    report['lock_verified']=True
                    break
                except RuntimeError:
                    pass
            if time.monotonic()>=deadline:
                raise RuntimeError('parking feedback/maintenance lock timeout')
            time.sleep(.1)
    finally:
        # Preserve the diagnostic even when maintenance could not be locked.
        report['final_status']=rpc('status')


def left(sensors, report, confirmed_self_returns):
    scan, start = checked(sensors, time.monotonic()+3)
    if report.get('resume_origin'):
        expected=report.pop('resume_expected')
        if math.hypot(start.x-expected[0],start.y-expected[1])>.02 or abs(turn.yaw_delta(expected[2],start.yaw))>math.radians(2):
            raise RuntimeError('robot moved or odom changed after interrupted turn')
        start=nav.Odom(**report['resume_origin'])
    report['origin'] = dataclasses.asdict(start)
    gate = quarter.EnvelopeGate()
    begun = time.monotonic()
    report['pulses'] = []
    stalled = 0
    for _ in range(30):
        scan, odom = checked(sensors, time.monotonic()+3)
        angle = bounded_pose(start, odom)
        if angle >= 89.2:
            if angle > 91.5:
                raise RuntimeError('settled yaw outside 89.2..91.5 degrees')
            report['yaw_deg'] = angle
            return
        if time.monotonic()-begun > 75:
            raise RuntimeError('rotation timeout')
        before = angle
        duration = quarter.pulse_duration(90-angle)
        client = turn.TurnClient(1)
        pulse_start = time.monotonic()
        try:
            while time.monotonic()-pulse_start < duration:
                # Never wait for telemetry with a live velocity command.
                scan, odom = checked(sensors, time.monotonic())
                bounded_pose(start, odom)
                if gate.check(scan):
                    report['swept_observations'] = report.get('swept_observations', 0)+1
                    if not confirmed_self_returns:
                        raise RuntimeError('near-body swept return')
                client.velocity()
                time.sleep(.03)
        except RuntimeError as exc:
            if not str(exc).startswith('telemetry:'):
                raise
            # Stop immediately; recovery below happens with no live client.
            report['telemetry_pauses']=report.get('telemetry_pauses',0)+1
            if report['telemetry_pauses']>8:
                raise RuntimeError('rotation telemetry recovery exhausted')
        finally:
            client.close()
        stable, last = 0, None
        settle_end = time.monotonic()+3
        while True:
            scan, odom = checked(sensors, settle_end)
            angle = bounded_pose(start, odom)
            if odom.stamp != last:
                last = odom.stamp
                stable = stable+1 if abs(odom.vx)<.01 and abs(odom.wz)<.02 else 0
            if stable >= 3 and time.monotonic()-pulse_start >= .8:
                break
            if time.monotonic() >= settle_end:
                raise RuntimeError('rotation did not settle')
            time.sleep(.04)
        report['pulses'].append({'duration_s':duration, 'yaw_deg':angle})
        if not -.5 <= angle-before <= 15:
            raise RuntimeError('pulse progress/overshoot limit')
        stalled = stalled+1 if angle-before < .05 else 0
        if stalled >= 3:
            raise RuntimeError('three pulses without progress')
    raise RuntimeError('rotation pulse count limit')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--step', choices=('probe','straight','left'), default='probe')
    p.add_argument('--output', required=True)
    p.add_argument('--real', action='store_true')
    p.add_argument('--confirmed-self-returns', action='store_true')
    p.add_argument('--native-snapshot')
    p.add_argument('--resume-report')
    args = p.parse_args()
    report = {'step':args.step,'real':args.real,'ok':False,
              'confirmed_self_returns':args.confirmed_self_returns}
    sensors = None
    try:
        report['initial_status'] = nav.check_service_status()
        if args.resume_report:
            previous=json.loads(Path(args.resume_report).read_text())
            if args.step!='left' or previous.get('step')!='left' or previous.get('ok'):
                raise RuntimeError('resume requires an interrupted left turn')
            final=previous['final_status']
            if report['initial_status']['uptime_s']<final['uptime_s']:
                raise RuntimeError('owner restarted since interrupted turn')
            report['resume_origin']=previous.get('resume_origin',previous['origin'])
            report['resume_expected']=final['odom']['pose']
        nav.check_one_serial_owner()
        if available_mb() < 1024:
            raise RuntimeError('RAM below 1 GB before step')
        sensors = NativeSensors(args.native_snapshot) if args.native_snapshot else nav.RosNavSensors()
        sensors.wait_ready()
        scan, odom = checked(sensors, time.monotonic()+3)
        report['origin'] = dataclasses.asdict(odom)
        report['front_m'] = nav.nr.front_min_brake(scan.points, config())
        if abs(odom.vx)>.01 or abs(odom.wz)>.02:
            raise RuntimeError('not stationary')
        if args.real and args.step != 'probe':
            rpc('maintenance-unlock')
            if args.step == 'straight':
                if report['front_m'] < .50:
                    report['reason'] = 'lidar_brake'
                    report['distance_m'] = 0
                else:
                    options = {'recover_sensor_s':3} if 'recover_sensor_s' in inspect.signature(nav.run_straight).parameters else {}
                    reason, distance = nav.run_straight(sensors, config(), .25, **options)
                    report.update(reason=reason, distance_m=distance)
                    if reason not in ('distance_target','lidar_brake'):
                        raise RuntimeError('straight: '+reason)
            else:
                left(sensors, report, args.confirmed_self_returns)
        scan, odom = checked(sensors, time.monotonic()+3)
        report['final_odom'] = dataclasses.asdict(odom)
        report['final_front_m'] = nav.nr.front_min_brake(scan.points, config())
        report['ok'] = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if args.real:
            try:
                park_report(report)
            except Exception as exc:
                report.update(ok=False, cleanup_error=str(exc))
        if sensors is not None:
            sensors.close()
        Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k in (
        'step','ok','error','reason','distance_m','front_m','final_front_m','yaw_deg')}),flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
