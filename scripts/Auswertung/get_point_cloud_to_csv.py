# !/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import rospy
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np
import pyrealsense2 as rs
import ros_numpy
from sklearn.cluster import DBSCAN
import struct
from std_msgs.msg import Header
from net_detection_and_control.msg import ekf_data

tmp = list()
k = 0


def callback(data):
    global tmp, k
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
        tmp.append(pt)
    print(len(tmp))
    k = k + 1
    print("k", k)

def shutdown_hook():
    global tmp
    print("saving")
    print("np.asarray(tmp, dtype=np.float32)",np.asarray(tmp, dtype=np.float32).shape)
    np.savetxt("point_cloud.csv",np.asarray(tmp, dtype=np.float32), delimiter=",")

def listener():
    rospy.init_node('publisher', anonymous=True)
    rospy.Subscriber("/d435i/depth/color/points", PointCloud2, callback, queue_size=1)
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    listener()
