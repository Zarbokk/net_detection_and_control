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
list_pos_gantry = list()
k = 0
mod_value = 5
current_gantry_pos_x = 0
current_gantry_pos_y = 0
current_gantry_pos_z = 0

def callback(data):
    global k, list_pos_gantry, current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z,mod_value

    # data = ekf_data()

    pt = [data.n1_x, data.n1_y, data.n2_x, data.n2_y,k]
    list_planes.append(pt)
    list_pos_gantry.append([current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z, k])
    k = k + 1
    print("k", k)


def callback_gantry(data):
    global current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z
    current_gantry_pos_x = data.pos_gantry.x
    current_gantry_pos_y = data.pos_gantry.y
    current_gantry_pos_z = data.pos_gantry.z


def shutdown_hook():
    global list_pos_gantry,list_planes
    print("saving")
    np.savetxt("planes_all.csv", np.asarray(list_planes, dtype=np.float32), delimiter=",")
    np.savetxt("gantry_pos_planes.csv", np.asarray(list_pos_gantry, dtype=np.float32), delimiter=",")
    print("saved")


def listener():
    rospy.init_node('publisher', anonymous=True)
    rospy.Subscriber("/plane_to_drive_by", ekf_data, callback, queue_size=1)
    rospy.Subscriber("/gantry/current_position", gantry, callback_gantry)
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    listener()
