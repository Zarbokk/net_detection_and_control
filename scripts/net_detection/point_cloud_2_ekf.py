#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import rospy
from sensor_msgs.msg import Imu
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np
#import pyrealsense2 as rs
import ros_numpy
#from sklearn.cluster import DBSCAN
import struct
from std_msgs.msg import Header
import ekf_class
from net_detection_and_control.msg import ekf_data
import time

def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm != 0:
        v = v / norm
    return v


def printing_to_rviz(left_segment, right_segment, current_state_ekf_r, current_state_ekf_l, pub,
                     publisher_marker):  # mx+b=y
    # right
    s_v = np.asarray([[0, 0, current_state_ekf_r[1]]], dtype="float32")
    r_v_1 = np.asarray([[1,
                         0,
                         current_state_ekf_r[0]+current_state_ekf_r[1]]],
                       dtype="float32")

    r_v_2 = np.asarray([[0, 1, 0]], dtype="float32")
    r_v_1 = r_v_1 - s_v
    r_v_2 = normalize_vector(r_v_2)
    r_v_1 = normalize_vector(r_v_1)

    markerArray = MarkerArray()
    i = 1
    for mu in np.linspace(0, 1, 10):
        for theta in np.linspace(-1, 1, 10):
            current_point = np.transpose(s_v) + mu * np.transpose(r_v_1) + theta * np.transpose(r_v_2)
            r = 0.02
            marker = Marker()
            marker.header.frame_id = "d435i_depth_optical_frame"
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

    # left
    s_v = np.asarray([[0, 0, current_state_ekf_l[1]]], dtype="float32")
    r_v_1 = np.asarray([[1,
                         0,
                         current_state_ekf_l[0]+current_state_ekf_l[1]]],
                       dtype="float32")
    r_v_1 = r_v_1 - s_v
    r_v_2 = normalize_vector(r_v_2)
    r_v_1 = normalize_vector(r_v_1)
    for mu in np.linspace(-1, 0, 10):
        for theta in np.linspace(-1, 1, 10):
            current_point = np.transpose(s_v) + mu * np.transpose(r_v_1) + theta * np.transpose(r_v_2)
            r = 0.02
            marker = Marker()
            marker.header.frame_id = "d435i_depth_optical_frame"
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
    header.frame_id = "d435i_depth_optical_frame"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)

def callback_imu(msg, tmp_list):
    [ekf_l, ekf_r] = tmp_list

    x_rot_vel = msg.angular_velocity.x
    y_rot_vel = msg.angular_velocity.y
    z_rot_vel = msg.angular_velocity.z
    ekf_l.prediction(z_rot_vel)
    ekf_r.prediction(z_rot_vel)



def callback(data, list):
    pub, ekf_l, ekf_r, publisher_marker, rate, publisher_plane = list
    start_time = time.time()
    pc = ros_numpy.numpify(data)

    points = np.zeros((pc.shape[0], 3))
    # remap from camera coordinate system to base_link
    points[:, 0] = pc['x'].flatten()
    points[:, 1] = pc['y'].flatten()
    points[:, 2] = pc['z'].flatten()
    # print(points.shape)
    points = points[::3, :]
    # print("points.shape",points.shape)
    points = np.float32(points)
    #points = points[points[:, 1] < 5]
    #points = points[points[:, 1] > -5]

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 1
    ret, label, center = cv2.kmeans(points, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    best_match = np.argmin(abs(center[:, 1] - 1))
    A = points#[label.ravel() == best_match]

    A = A[::2, :]

    # CALCULATE THE DIFFERENT SIDES      LEFT AND RIGHT

    # print("center", center[best_match, :])
    median_center = np.zeros((3))
    # from koordinate system (z raus) zu x aus der kamera raus
    median_center[0] = center[best_match, 0]
    median_center[1] = center[best_match, 1]
    median_center[2] = center[best_match, 2]
    # print("median_center", median_center)
    current_mean_angle = np.arctan2(median_center[2], median_center[0])
    # print("current_mean_angle", current_mean_angle)
    left_segment = []
    right_segment = []
    for i in range(A.shape[0]):
        x = A[i, 0]
        y = A[i, 1]
        z = A[i, 2]
        if np.linalg.norm([x-median_center[0],y-median_center[1],z-median_center[2]])<1:
            if np.arctan2(z, x) > current_mean_angle:
                left_segment.append([x, y, z])
            else:
                right_segment.append([x, y, z])
    left_segment = np.asarray(left_segment)
    #print("left_segment", left_segment.shape)
    right_segment = np.asarray(right_segment)
    #print("right_segment", right_segment.shape)

    A = np.ones((left_segment.shape[0], 2))
    A[:, 0] = left_segment[:, 0]
    B = left_segment[:, 2]

    # print("A", A.shape)
    # print("B", B.shape)
    # print("inverse:", np.linalg.inv(np.matmul(np.transpose(A), A)))
    m_b = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(A), A)), np.transpose(A)), B)
    # update EKF
    #ekf_l.prediction()
    # print("m_b", m_b)
    ekf_l.update(m_b)
    current_state_ekf_l = ekf_l.get_x_est()

    A = np.ones((right_segment.shape[0], 2))
    A[:, 0] = right_segment[:, 0]
    B = right_segment[:, 2]

    m_b = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(A), A)), np.transpose(A)), B)
    #ekf_r.prediction()

    # print("EKF Update:")
    ekf_r.update(m_b)
    current_state_ekf_r = ekf_r.get_x_est()

    rviz = True
    if rviz:
        printing_to_rviz(left_segment, right_segment, current_state_ekf_r, current_state_ekf_l, pub, publisher_marker)
        #print("current_state_ekf_r", current_state_ekf_r)
        #print("current_state_ekf_l", current_state_ekf_l)

    msg = ekf_data()
    msg.header.frame_id = "d435i_depth_optical_frame"
    msg.header.stamp = rospy.Time.now()
    # msg.d1 = current_state_ekf_r[3]
    msg.n1_x = current_state_ekf_r[0]
    msg.n1_y = current_state_ekf_r[1]
    # msg.n1_z = current_state_ekf_r[2]
    # msg.d2 = current_state_ekf_l[3]
    msg.n2_x = current_state_ekf_l[0]
    msg.n2_y = current_state_ekf_l[1]
    # msg.n2_z = current_state_ekf_l[2]
    publisher_plane.publish(msg)
    elapsed_time = time.time() - start_time
    #print("time duration",elapsed_time)

    rate.sleep()


def listener():
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(20)
    ekf_l = ekf_class.ExtendedKalmanFilter()
    ekf_r = ekf_class.ExtendedKalmanFilter()
    pub_cloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)
    publisher_marker = rospy.Publisher('detection_net_plane', MarkerArray, queue_size=1)
    publisher_plane = rospy.Publisher('plane_to_drive_by', ekf_data, queue_size=1)
    rospy.Subscriber("output", PointCloud2, callback,
                     [pub_cloud, ekf_l, ekf_r, publisher_marker, rate, publisher_plane], queue_size=1,buff_size=65536*2)
    rospy.Subscriber("/d435i/gyro/sample", Imu, callback_imu,
                     [ekf_l, ekf_r], queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()
