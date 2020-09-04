# !/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from gantry_control_ros.msg import gantry
import rospy
import numpy as np
import ros_numpy
import struct
from net_detection_and_control.msg import ekf_data
list_planes = list()
list_pos = list()
list_pc = list()
k = 0
mod_value = 5
current_px4_pos_x = 0
current_px4_pos_y = 0
current_px4_pos_z = 0
yaw_current =0



def callback_pos(data):
    global k, list_pos, current_px4_pos_x, current_px4_pos_y, current_px4_pos_z, mod_value,yaw_current

    rotation_body_frame = Quaternion(w=data.pose.orientation.w,
                                     x=data.pose.orientation.x,
                                     y=data.pose.orientation.y,
                                     z=data.pose.orientation.z)

    yaw_current, pitch_current, roll_current = rotation_body_frame.yaw_pitch_roll
    roll_current=-roll_current
    current_px4_pos_x=data.pose.position.x
    current_px4_pos_y=data.pose.position.y
    current_px4_pos_z=data.pose.position.z


def callback_pointcloud(data):
    global k, list_pos, current_px4_pos_x, current_px4_pos_y, current_px4_pos_z, mod_value,yaw_current
    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape[0], 4))
    # remap from camera coordinate system to base_link
    points[:, 0] = pc['x'].flatten()
    points[:, 1] = pc['y'].flatten()
    points[:, 2] = pc['z'].flatten()
    points = np.float32(points)
    current_time = rospy.get_time()
    print(rospy.get_time())
    for i in range(points.shape[0]):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        # print("r", r, g, b, a)
        pt = [x, y, z, k,current_time]
        list_pc.append(pt)




    list_pos.append([current_px4_pos_x,current_px4_pos_y ,current_px4_pos_z,yaw_current,k,current_time])
    k = k + 1
    print("k", k)


def shutdown_hook():
    global list_pos_gantry,list_planes
    print("saving")
    np.savetxt("pose_px4_boat.csv", np.asarray(list_pos, dtype=np.float64), delimiter=",", fmt="%10.15f")
    np.savetxt("point_cloud.csv", np.asarray(list_pc, dtype=np.float64), delimiter=",", fmt="%10.15f")
    print("saved")


def listener():
    rospy.init_node('publisher', anonymous=True)
    rospy.Subscriber("/uuv00/pose_px4", PoseStamped, callback_pos)
    rospy.Subscriber("/point_cloud2", PointCloud2, callback_pointcloud, queue_size=1)

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    listener()
