#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from pyquaternion import Quaternion

qx_90 = Quaternion(axis=[1, 0, 0], angle=np.pi / 2)


def callback(msg, list):
    br, rate = list
    # msg=PoseStamped()
    # print(msg)
    br.sendTransform([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                     [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
                     rospy.Time.now(),
                     "base_link",  # child
                     "global_tank")  # parent transform from parent to child
    br.sendTransform([0, 0, 0],
                     [qx_90.x, qx_90.y, qx_90.z, qx_90.w],
                     rospy.Time.now(),
                     "camera_link",  # child
                     "base_link")
    rate.sleep()


def tranfsormation():
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()

    rospy.Subscriber("/mavros/local_position/pose_NED", PoseStamped, callback, [br, rate], queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    tranfsormation()
