#!/usr/bin/env python
"""Bounded native ROS latest-only sensor snapshot, no motion or Serial access."""
import argparse
import json
import math
import os
import threading
import time


def main():
    import rospy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',default='/dev/shm/g2_native_nav.json')
    p.add_argument('--seconds',type=int,default=600)
    args=p.parse_args()
    if not 10<=args.seconds<=600:
        p.error('seconds must be 10..600')
    rospy.init_node('g2_native_nav_snapshot')
    lock=threading.Lock()
    state={}
    def odom(m):
        with lock:
            state['odom']={'header':{'stamp':{'secs':m.header.stamp.secs,'nsecs':m.header.stamp.nsecs},'frame_id':m.header.frame_id},'child_frame_id':m.child_frame_id,
                'pose':{'pose':{'position':{'x':m.pose.pose.position.x,'y':m.pose.pose.position.y},'orientation':{'x':m.pose.pose.orientation.x,'y':m.pose.pose.orientation.y,'z':m.pose.pose.orientation.z,'w':m.pose.pose.orientation.w}}},
                'twist':{'twist':{'linear':{'x':m.twist.twist.linear.x},'angular':{'z':m.twist.twist.angular.z}}}}
    def scan(m):
        with open('/proc/meminfo') as f:
            ram=next(float(line.split()[1])/1024 for line in f if line.startswith('MemAvailable:'))
        if ram<768:
            rospy.signal_shutdown('RAM_pressure');return
        if not 0< len(m.ranges)<=4096:
            rospy.signal_shutdown('invalid scan size');return
        with lock:
            out=dict(state)
        out['written_at']=time.time()
        out['scan']={'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,
            'points':[(math.degrees(m.angle_min+i*m.angle_increment),float(r)) for i,r in enumerate(m.ranges) if not math.isnan(r) and not math.isinf(r) and m.range_min<=r<=m.range_max]}
        temp=args.output+'.tmp'
        with open(temp,'w') as f:
            f.write(json.dumps(out,separators=(',',':')))
        os.rename(temp,args.output)
    rospy.Subscriber('/odom_setmotor',Odometry,odom,queue_size=1,tcp_nodelay=True)
    rospy.Subscriber('/scan',LaserScan,scan,queue_size=1,tcp_nodelay=True)
    end=time.time()+args.seconds
    try:
        while not rospy.is_shutdown() and time.time()<end:
            time.sleep(.1)
    finally:
        for path in (args.output,args.output+'.tmp'):
            if os.path.exists(path):os.unlink(path)


if __name__=='__main__':
    main()
