#!/usr/bin/env python
"""Fixed self-return profile: /scan_raw -> /scan, compatible with ROS Melodic."""
import copy
import json
import math


def finite(value):
    return not math.isnan(value) and not math.isinf(value)


class FixedProfile(dict):
    """Legacy angle bins plus explicitly surveyed base-frame body rectangles."""
    def __init__(self):
        dict.__init__(self)
        self.body_rectangles = []


def load_profile(path):
    with open(path) as stream:
        data = json.load(stream)
    if data.get('schema') != 1 or data.get('frame_id') != 'laser':
        raise ValueError('unsupported self-return profile')
    tolerance = float(data['tolerance_m'])
    if not finite(tolerance) or not 0 < tolerance <= .025:
        raise ValueError('invalid tolerance')
    profile = FixedProfile()
    for key, values in data['ranges_by_angle_deg'].items():
        angle = int(key)
        if not -180 <= angle <= 180 or not values:
            raise ValueError('invalid angle/ranges')
        if any(not finite(v) or not .1 <= v <= .75 for v in values):
            raise ValueError('invalid learned range')
        profile[angle] = values
    rectangles = data.get('body_rectangles_base_m', [])
    if rectangles:
        geometry = data.get('laser_geometry', {})
        if geometry != {'forward_m': .10, 'yaw_rad': math.pi}:
            raise ValueError('body mask requires verified 10cm/180deg laser geometry')
        if len(rectangles) > 8:
            raise ValueError('too many body rectangles')
        for rectangle in rectangles:
            bounds = rectangle['bounds_xyxy']
            if len(bounds) != 4 or not all(finite(v) for v in bounds):
                raise ValueError('invalid body rectangle')
            x0,y0,x1,y1 = bounds
            if not x0 < x1 or not y0 < y1:
                raise ValueError('invalid body rectangle order')
            if max(math.hypot(x,y) for x in (x0,x1) for y in (y0,y1)) > .32:
                raise ValueError('body mask must stay inside 32cm centre radius')
            if not rectangle.get('source'):
                raise ValueError('body mask needs physical source evidence')
            profile.body_rectangles.append(tuple(bounds))
    if not profile and not profile.body_rectangles:
        raise ValueError('empty self-return profile')
    return profile, tolerance


def filter_ranges(ranges, angle_min, increment, profile, tolerance):
    if not finite(angle_min) or not finite(increment) or not 0 < increment < .1 or not 1 <= len(ranges) <= 4096:
        raise ValueError('unsupported scan geometry')
    output, masked = list(ranges), 0
    for i, distance in enumerate(ranges):
        if not finite(distance):
            continue
        # Python 2 round differs at .5; explicitly use Python 3 ties-to-even,
        # matching the calibration profile builder.
        angle = math.degrees(angle_min + i * increment)
        floor = math.floor(angle)
        key = int(floor + (1 if angle-floor > .5 or
                          (angle-floor == .5 and int(floor) % 2) else 0))
        values = profile.get(key, ())
        theta = angle_min + i * increment + math.pi
        x,y = .10 + distance * math.cos(theta), distance * math.sin(theta)
        body = any(x0 <= x <= x1 and y0 <= y <= y1
                   for x0,y0,x1,y1 in getattr(profile, 'body_rectangles', ()))
        if body or any(abs(distance - value) <= tolerance for value in values):
            output[i] = float('nan')  # unknown ray, not free-space clearing
            masked += 1
    return output, masked


def main():
    import rospy
    from sensor_msgs.msg import LaserScan
    rospy.init_node('g2_self_return_filter')
    profile, tolerance = load_profile(rospy.get_param('~profile'))
    rospy.loginfo('fixed self-return profile loaded: %d angle bins, %d body rectangles',
                  len(profile), len(profile.body_rectangles))
    publisher = rospy.Publisher('/scan', LaserScan, queue_size=1)

    def callback(scan):
        try:
            with open('/proc/meminfo') as stream:
                available = next(float(line.split()[1])/1024 for line in stream
                                 if line.startswith('MemAvailable:'))
            if available < 768:
                raise ValueError('RAM below 768 MB')
            age = (rospy.Time.now()-scan.header.stamp).to_sec()
            if scan.header.frame_id != 'laser' or not 0 <= age <= .35:
                rospy.logwarn_throttle(2, 'raw scan stale or incorrect frame; withheld')
                return
            output = copy.copy(scan)
            output.ranges, masked = filter_ranges(scan.ranges, scan.angle_min,
                                                  scan.angle_increment, profile, tolerance)
            publisher.publish(output)
            rospy.loginfo_throttle(30, 'self-return filter: %d/%d rays masked', masked, len(scan.ranges))
        except (ValueError, IOError, OSError, StopIteration) as exc:
            rospy.logerr('self-return filter failed: %s', exc)
            rospy.signal_shutdown(str(exc))

    rospy.Subscriber('/scan_raw', LaserScan, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
