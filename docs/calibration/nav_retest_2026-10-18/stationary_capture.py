#!/usr/bin/python
import json,math,socket,sys,threading,time
import rospy,tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
rospy.init_node('g2_stationary_evidence',anonymous=True)
listener=tf.TransformListener();data={};counts={};lock=threading.Lock()
def capture(key,m):
 d={'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id,'received':time.time()}
 if key=='odom':
  p=m.pose.pose.position;q=m.pose.pose.orientation
  d.update(pose=[p.x,p.y,tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2]],child=m.child_frame_id,twist=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])
 else:d.update(angle_min=m.angle_min,angle_increment=m.angle_increment,range_min=m.range_min,range_max=m.range_max,ranges=[float(x) if not math.isinf(x) and not math.isnan(x) else None for x in m.ranges])
 with lock:data[key]=d;counts[key]=counts.get(key,0)+1
subs=[rospy.Subscriber('/scan',LaserScan,lambda m:capture('scan',m),queue_size=1),rospy.Subscriber('/scan_raw',LaserScan,lambda m:capture('raw',m),queue_size=1),rospy.Subscriber('/odom_setmotor',Odometry,lambda m:capture('odom',m),queue_size=1)]
time.sleep(3)
with lock:out=dict(data);out['counts']=dict(counts)
for pair in [('base_footprint','laser'),('odom','base_footprint')]:
 try:out['tf_'+'_'.join(pair)]=listener.lookupTransform(pair[0],pair[1],rospy.Time(0))
 except Exception as e:out['tf_'+'_'.join(pair)]=str(e)
s=socket.socket(socket.AF_UNIX);s.settimeout(3);s.connect('/tmp/grasp_health.sock');s.sendall(b'health\n');out['health']=json.loads(s.makefile('r').readline());s.close()
out['time']=time.time()
with open(sys.argv[1],'w') as f:json.dump(out,f)
print(json.dumps({'time':out['time'],'counts':out['counts'],'odom':out.get('odom'),'motors':out['health']['chassis']['motors']}))
