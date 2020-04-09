# !/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import rospy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np
import pyrealsense2 as rs
import ros_numpy
from sklearn.cluster import DBSCAN
import struct
from std_msgs.msg import Header
import ekf_class


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm != 0:
        v = v / norm
    return v


def printing_to_rviz(left_segment, right_segment, current_state_ekf_r, current_state_ekf_l, pub, publisher_marker):
    #right
    s_v = np.asarray([[current_state_ekf_r[3] / current_state_ekf_r[0], 0, 0]], dtype="float32")
    r_v_1 = np.asarray([[s_v[0, 0] + 0.5,
                         (current_state_ekf_r[3] - (s_v[0, 0] + 0.5) * current_state_ekf_r[0]) / current_state_ekf_r[1],
                         0]],
                       dtype="float32")

    r_v_2 = np.asarray([[0, 0, 1]], dtype="float32")
    r_v_1 = r_v_1 - s_v
    r_v_2 = normalize_vector(r_v_2)
    r_v_1 = normalize_vector(r_v_1)

    markerArray = MarkerArray()
    i = 1
    for mu in np.linspace(-1, 1, 10):
        for theta in np.linspace(-1, 1, 10):
            current_point = np.transpose(s_v) + mu * np.transpose(r_v_1) + theta * np.transpose(r_v_2)
            r = 0.02
            marker = Marker()
            marker.header.frame_id = "d435i_link"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = r * 2  # r*2
            marker.scale.y = r * 2
            marker.scale.z = r * 2
            marker.color.r = 1
            marker.color.g = 1
            marker.color.a = 1  # transparency
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = current_point[0]  # x
            marker.pose.position.y = current_point[1]  # y
            marker.pose.position.z = current_point[2]  # z
            markerArray.markers.append(marker)
            i = i + 1

    #left
    s_v = np.asarray([[current_state_ekf_l[3] / current_state_ekf_l[0], 0, 0]], dtype="float32")
    r_v_1 = np.asarray([[s_v[0, 0] + 0.5,
                         (current_state_ekf_l[3] - (s_v[0, 0] + 0.5) * current_state_ekf_l[0]) / current_state_ekf_l[1],
                         0]],
                       dtype="float32")

    r_v_2 = np.asarray([[0, 0, 1]], dtype="float32")
    r_v_1 = r_v_1 - s_v
    r_v_2 = normalize_vector(r_v_2)
    r_v_1 = normalize_vector(r_v_1)
    for mu in np.linspace(-1, 1, 10):
        for theta in np.linspace(-1, 1, 10):
            current_point = np.transpose(s_v) + mu * np.transpose(r_v_1) + theta * np.transpose(r_v_2)
            r = 0.02
            marker = Marker()
            marker.header.frame_id = "d435i_link"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = r * 2  # r*2
            marker.scale.y = r * 2
            marker.scale.z = r * 2
            marker.color.r = 0.5
            marker.color.g = 0
            marker.color.b = 1
            marker.color.a = 1  # transparency
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = current_point[0]  # x
            marker.pose.position.y = current_point[1]  # y
            marker.pose.position.z = current_point[2]  # z
            markerArray.markers.append(marker)
            i = i + 1
    publisher_marker.publish(markerArray)
    # PUBLISH CURRENT POINT CLOUD
    points = []
    for i in range(left_segment.shape[0]):
        x = left_segment[i, 0]
        y = left_segment[i, 1]
        z = left_segment[i, 2]
        r = 255
        g = 0
        b = 0
        a = 150
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        points.append(pt)

    for i in range(right_segment.shape[0]):
        x = right_segment[i, 0]
        y = right_segment[i, 1]
        z = right_segment[i, 2]
        r = 0
        g = 255
        b = 0
        a = 150
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        points.append(pt)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              # PointField('rgb', 12, PointField.UINT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]

    # print points

    header = Header()
    header.frame_id = "d435i_link"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)


def callback(data, list):
    pub, ekf_l, ekf_r, publisher_marker = list

    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape[0], 3))
    points[:, 0] = pc['x']
    points[:, 1] = pc['y']
    points[:, 2] = pc['z']
    # print(points.shape)
    points = points[::10, :]
    # print(points.shape)
    points = np.float32(points)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 8
    ret, label, center = cv2.kmeans(points, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    best_match = np.argmin(abs(center[:, 2] - 1))
    A = points[label.ravel() == best_match]
    A = A[::10, :]

    # CALCULATE THE DIFFERENT SIDES      LEFT AND RIGHT

    print("center", center[best_match, :])
    median_center = np.zeros((3))
    # from koordinate system (z raus) zu x aus der kamera raus
    median_center[0] = center[best_match, 2]
    median_center[1] = -center[best_match, 0]
    median_center[2] = -center[best_match, 1]
    print("median_center", median_center)
    current_mean_angle = np.arctan2(median_center[1], median_center[0])
    print("current_mean_angle", current_mean_angle)
    left_segment = []
    right_segment = []
    for i in range(A.shape[0]):
        x = A[i, 2]
        y = -A[i, 0]
        z = -A[i, 1]
        if np.arctan2(y, x) > current_mean_angle:
            left_segment.append([x, y, z])
        else:
            right_segment.append([x, y, z])
    left_segment = np.asarray(left_segment)
    print("left_segment", left_segment.shape)
    right_segment = np.asarray(right_segment)
    print("right_segment", right_segment.shape)

    # update EKF
    ekf_l.prediction()

    ekf_l.update(left_segment)
    current_state_ekf_l = ekf_l.get_x_est()

    ekf_r.prediction()

    print("EKF Update:")
    ekf_r.update(right_segment)
    current_state_ekf_r = ekf_r.get_x_est()

    rviz = True
    if rviz:
        printing_to_rviz(left_segment, right_segment, current_state_ekf_r, current_state_ekf_l, pub, publisher_marker)


def listener():
    rospy.init_node('publisher', anonymous=True)
    ekf_l = ekf_class.ExtendedKalmanFilter()
    ekf_r = ekf_class.ExtendedKalmanFilter()
    pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
    publisher_marker = rospy.Publisher('detection_net_plane', MarkerArray, queue_size=1)
    rospy.Subscriber("/d435i/depth/color/points", PointCloud2, callback, [pub, ekf_l, ekf_r, publisher_marker])
    rospy.spin()
    # video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
