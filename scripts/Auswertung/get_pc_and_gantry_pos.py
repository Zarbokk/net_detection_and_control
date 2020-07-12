# !/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from geometry_msgs.msg import PoseStamped
from gantry_control_ros.msg import gantry
import rospy
import numpy as np
import ros_numpy
import struct

list_point_cloud = list()
list_pos_gantry = list()
k = 0
mod_value = 5
current_gantry_pos_x = 0
current_gantry_pos_y = 0
current_gantry_pos_z = 0

def callback(data):
    global list_point_cloud, k, list_pos_gantry, current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z,mod_value
    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape[0], 4))
    # remap from camera coordinate system to base_link
    points[:, 0] = pc['x'].flatten()
    points[:, 1] = -pc['z'].flatten()
    points[:, 2] = pc['y'].flatten()
    points[:, 3] = pc['rgb'].flatten()
    # print(points.shape)
    points = points[::3, :]
    # print(points.shape)
    points = np.float32(points)
    # points = points[points[:, 1] < 5]
    # points = points[points[:, 1] > -5]
    median_center = np.zeros((3))
    # from koordinate system (z raus) zu x aus der kamera raus

    for i in range(points.shape[0]):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        r, g, b, a = struct.unpack('BBBB', points[i, 3])
        # print("r", r, g, b, a)
        pt = [x, y, z, r, g, b, a, k]
        if k % mod_value == 0:
            list_point_cloud.append(pt)
    print(len(list_point_cloud))
    if k % mod_value == 0:
        list_pos_gantry.append([current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z, k])
    k = k + 1
    print("k", k)


def callback_gantry(data):
    global current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z
    current_gantry_pos_x = data.pos_gantry.x
    current_gantry_pos_y = data.pos_gantry.y
    current_gantry_pos_z = data.pos_gantry.z


def shutdown_hook():
    global list_point_cloud
    print("saving")
    print("np.asarray(list_point_cloud, dtype=np.float32)", np.asarray(list_point_cloud, dtype=np.float32).shape)
    np.savetxt("point_cloud.csv", np.asarray(list_point_cloud, dtype=np.float32), delimiter=",")
    np.savetxt("gantry_pos.csv", np.asarray(list_pos_gantry, dtype=np.float32), delimiter=",")
    print("saved")


def listener():
    rospy.init_node('publisher', anonymous=True)
    rospy.Subscriber("/d435i/depth/color/points", PointCloud2, callback, queue_size=1)
    rospy.Subscriber("/gantry/current_position", gantry, callback_gantry)
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    listener()
