#!/usr/bin/env python3
"""Capture one top-down frame of the arena from a running Gazebo to verify texture alignment."""
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

out = sys.argv[1]
rospy.init_node('capture_topdown', anonymous=True)
bridge = CvBridge()
done = {'v': False}


def cb(msg):
    if done['v']:
        return
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    import cv2
    cv2.imwrite(out, img)
    done['v'] = True
    print('saved %s  %dx%d' % (out, img.shape[1], img.shape[0]))
    rospy.signal_shutdown('done')


rospy.Subscriber('/arena_topdown/image_raw', Image, cb)
t0 = rospy.Time.now()
while not rospy.is_shutdown() and not done['v'] and (rospy.Time.now() - t0).to_sec() < 60:
    rospy.sleep(0.2)
sys.exit(0 if done['v'] else 1)
