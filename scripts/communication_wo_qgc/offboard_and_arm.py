#!/usr/bin/env python

import sys
import rospy
from mavros_msgs.srv import CommandBool,SetMode


def arm_px4():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        status_arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        status_offboard = rospy.ServiceProxy('mavros/set_mode', SetMode)
        return status_arming(True),status_offboard(0,"OFFBOARD")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print("arming",arm_px4())
