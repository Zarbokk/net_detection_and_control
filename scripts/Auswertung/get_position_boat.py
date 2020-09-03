# !/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from geometry_msgs.msg import PoseStamped
from gantry_control_ros.msg import gantry
import rospy
import numpy as np
import ros_numpy
import struct
from net_detection_and_control.msg import ekf_data
list_planes = list()
list_pos = list()
k = 0
mod_value = 5
current_gantry_pos_x = 0
current_gantry_pos_y = 0
current_gantry_pos_z = 0

def callback_pos(data):
    global k, list_pos, current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z, mod_value


    list_pos.append([data.pose.position.x, data.pose.position.y, data.pose.position.z,k,rospy.get_time()])
    k = k + 1
    print("k", k)



def shutdown_hook():
    global list_pos_gantry,list_planes
    print("saving")
    np.savetxt("pose_px4_boat.csv", np.asarray(list_pos, dtype=np.float64), delimiter=",", fmt="%10.15f")
    print("saved")


def listener():
    rospy.init_node('publisher', anonymous=True)
    rospy.Subscriber("/uuv00/pose_px4", PoseStamped, callback_pos)
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    listener()
