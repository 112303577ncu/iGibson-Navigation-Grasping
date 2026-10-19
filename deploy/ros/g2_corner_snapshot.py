#!/usr/bin/env python
"""One stopped, boot-bound LiDAR/odom capture without AMCL or motion."""
from __future__ import division
import argparse
import json
import math
import os
import subprocess
import threading
import time
import uuid

from g2_map_patrol import rpc, available_mb
from g2_localization_service import map_hash


def main():
    import rospy
    import tf
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--output', default='/dev/shm/g2_corner_capture.json')
    args = parser.parse_args()
    if os.path.realpath(args.output) == os.path.realpath(args.map):
        parser.error('output must not overwrite map')
    rospy.init_node('g2_corner_snapshot', disable_signals=True)
    transforms = tf.TransformListener()
    mutex = threading.Lock(); state = {}

    def scan(message):
        if message.header.frame_id != 'laser' or not 100 <= len(message.ranges) <= 4096:
            return
        ranges = [float(r) if not math.isnan(r) and not math.isinf(r)
                  and message.range_min <= r <= message.range_max else None for r in message.ranges]
        with mutex:
            state['scan'] = dict(stamp=message.header.stamp.to_sec(), received=time.time(), frame='laser',
                                 angle_min=message.angle_min, angle_increment=message.angle_increment,
                                 range_min=message.range_min, range_max=message.range_max, ranges=ranges)

    def odom(message):
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_footprint':
            return
        pos, quat = message.pose.pose.position, message.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        with mutex:
            state['odom'] = dict(stamp=message.header.stamp.to_sec(), received=time.time(),
                                 pose=[pos.x, pos.y, yaw], frame='odom', child='base_footprint',
                                 twist=[message.twist.twist.linear.x, message.twist.twist.linear.y,
                                        message.twist.twist.angular.z])

    def idle():
        health = rpc('health'); ch = health['chassis']; od = health['odom']
        # fuser puts the device label on stderr; only stdout holds owner PIDs.
        process = subprocess.Popen(['fuser', '/dev/myserial'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, _ = process.communicate(); owners = stdout.decode().split()
        if (process.returncode or owners != [str(health['service_pid'])] or available_mb() < 1024
                or health.get('ok') is not True or health.get('chassis_tcp_ready') is not True
                or ch.get('motors') != [0, 0, 0, 0] or ch.get('moving') is not False
                or ch.get('arm_busy') is not False or ch.get('arm_pose') != 'travel'
                or 'client' not in ch or ch['client'] is not None or ch.get('stopped_for_s', 0) < .5
                or not 0 <= ch.get('board_rx_age_s', 999) <= .2 or od.get('valid') is not True
                or od.get('failures') != 0 or any(abs(v) > .01 for v in od['twist'])):
            raise ValueError('need sole, stopped travel resident, fresh feedback and 1024MB RAM')
        return health

    initial = idle()
    rospy.Subscriber('/scan', LaserScan, scan, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('/odom_setmotor', Odometry, odom, queue_size=1, tcp_nodelay=True)
    # ROS Python2 lacks monotonic(); /proc uptime is independent of wall-clock adjustments.
    def clock():
        with open('/proc/uptime') as stream:
            return float(stream.read().split()[0])
    end = clock()+5
    while clock() < end and not rospy.is_shutdown():
        with mutex:
            data = dict(state)
        now = time.time()
        if all(k in data and 0 <= now-data[k]['stamp'] <= .4
               and 0 <= now-data[k]['received'] <= .4 for k in ('scan', 'odom')):
            try:
                offset, rotation = transforms.lookupTransform('base_footprint', 'laser', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                time.sleep(.05)
                continue
            laser_yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            yaw_error = (laser_yaw-math.pi+math.pi) % (2*math.pi)-math.pi
            if abs(offset[0]-.10) > .005 or abs(offset[1]) > .005 or abs(yaw_error) > math.radians(1):
                raise ValueError('LiDAR TF must be x=0.10m, y=0, yaw=pi')
            health = idle()
            if health['service_pid'] != initial['service_pid']:
                raise ValueError('resident owner changed during capture')
            measured = data['odom']['pose']; resident = health['odom']['pose']
            angle = (measured[2]-resident[2]+math.pi) % (2*math.pi)-math.pi
            if (math.hypot(measured[0]-resident[0], measured[1]-resident[1]) > .01
                    or abs(angle) > math.radians(1) or any(abs(v) > .01 for v in data['odom']['twist'])):
                raise ValueError('snapshot/board stationary odom mismatch')
            with open('/proc/sys/kernel/random/boot_id') as stream:
                boot = stream.read().strip()
            data.update(schema=1, time=time.time(), capture_id=uuid.uuid4().hex, boot_id=boot,
                        owner_pid=health['service_pid'], map_hash=map_hash(os.path.abspath(args.map)),
                        lidar_forward_m=.10, lidar_yaw_rad=math.pi)
            temporary = args.output+'.tmp'
            with open(temporary, 'w') as stream:
                json.dump(data, stream, allow_nan=False)
            os.rename(temporary, args.output)
            print(json.dumps(dict(ok=True, capture_id=data['capture_id'], time=data['time'],
                                  boot_id=boot, owner_pid=data['owner_pid'], map_hash=data['map_hash'],
                                  output=args.output, arm_or_motion_executed=False)))
            return 0
        time.sleep(.05)
    raise RuntimeError('fresh scan/odom capture timed out after 5s')


if __name__ == '__main__':
    raise SystemExit(main())
