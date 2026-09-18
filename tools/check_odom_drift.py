#!/usr/bin/env python3
"""跑一段轨迹, 全程对比 /odom 和 Gazebo 真值"""
import math, threading, rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

odom = {'x':0.0,'y':0.0,'th':0.0}
truth = {'x':0.0,'y':0.0,'th':0.0}

def cb_o(m):
    odom['x']=m.pose.pose.position.x; odom['y']=m.pose.pose.position.y
    q=m.pose.pose.orientation
    odom['th']=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def cb_t(m):
    try: i=m.name.index('competition_robot')
    except ValueError: return
    p=m.pose[i]; q=p.orientation
    truth['x']=p.position.x; truth['y']=p.position.y
    truth['th']=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

rospy.init_node('check_odom_drift', anonymous=True)
rospy.Subscriber('/odom', Odometry, cb_o)
rospy.Subscriber('/gazebo/model_states', ModelStates, cb_t)
import geometry_msgs.msg as gm
pub = rospy.Publisher('/cmd_vel', gm.Twist, queue_size=10)
rospy.sleep(2.0)

def drive(vx, wz, secs):
    t0 = rospy.Time.now()
    r = rospy.Rate(20)
    while (rospy.Time.now()-t0).to_sec() < secs and not rospy.is_shutdown():
        t = gm.Twist(); t.linear.x = vx; t.angular.z = wz
        pub.publish(t); r.sleep()

print('  %-22s %-24s %-24s %s' % ('阶段', '真值(x,y,yaw)', '里程计(x,y,yaw)', '位置差'))
for name, vx, wz, secs in [('前进 4s', 0.30, 0.0, 4), ('左转 4s', 0.0, 0.8, 4),
                           ('前进 4s', 0.30, 0.0, 4), ('左转 4s', 0.0, 0.8, 4),
                           ('前进 4s', 0.30, 0.0, 4), ('左转 4s', 0.0, 0.8, 4),
                           ('前进 4s', 0.30, 0.0, 4)]:
    drive(vx, wz, secs)
    rospy.sleep(0.3)
    d = math.hypot(odom['x']-truth['x'], odom['y']-truth['y'])
    print('  %-22s (%+.3f,%+.3f,%+.3f) (%+.3f,%+.3f,%+.3f) %.3f m'
          % (name, truth['x'], truth['y'], truth['th'],
             odom['x'], odom['y'], odom['th'], d))
