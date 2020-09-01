#!/usr/bin/env python

import rospy

from path_planning import PathPlanning
from net_detection_and_control.msg import ekf_data
from mavros_msgs.msg import AttitudeTarget

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from pyquaternion import Quaternion



def main():
    global orientation, xyz
    planning_class = PathPlanning()
    rospy.init_node('path_planning', anonymous=True)
    rate = rospy.Rate(30)
    publisher_marker = rospy.Publisher('path_planning', MarkerArray, queue_size=1)
    publisher_waypoint = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    rospy.Subscriber("uuv00/mavros/imu/data", Imu, planning_class.callback_imu, queue_size=1)
    rospy.Subscriber("uuv00/pose_px4", PoseStamped, planning_class.callback_barometer, queue_size=1)


    rospy.Subscriber("plane_to_drive_by", ekf_data, planning_class.callback_net_distance, queue_size=1)
    while not rospy.is_shutdown():
        planning_class.pathplanning(publisher_waypoint, publisher_marker)
        rate.sleep()


if __name__ == '__main__':
    main()
