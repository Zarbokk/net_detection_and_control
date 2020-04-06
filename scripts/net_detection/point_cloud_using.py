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


def callback(data, list):
    pub, ekf, publisher_marker = list
    # print("Start of current picture")
    # print(ekf.get_x_est())
    # print(ekf.get_p_mat())
    # print(image.encoding)
    # brige = CvBridge()
    # try:
    #     frame = brige.imgmsg_to_cv2(image, "passthrough")
    #     # frame = brige.compressed_imgmsg_to_cv2(image, "passthrough")
    # except CvBridgeError as e:
    #     print(e)
    #
    # print(frame.shape)
    # point_data_3d = np.zeros((frame.shape[0] * frame.shape[1], 3))
    # print(point_data_3d.shape)
    # for i in range(point_data_3d.shape[0]):
    #     # print(i)
    #     # print(np.mod(i, 480))
    #     # print(i/640)
    #     point_data_3d[i, 0] = np.mod(i, 480)/300.0
    #     point_data_3d[i, 1] = i/640/300.0
    #     point_data_3d[i, 2] = frame[np.mod(i, 480), i/640]
    # point_data_3d = np.float32(point_data_3d)
    # #print(point_data_3d[1000,:])

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
    # print(center)

    best_match = np.argmin(abs(center[:, 2] - 1))
    # print(best_match)
    A = points[label.ravel() == best_match]
    # B = points[label.ravel() == 1]
    # C = points[label.ravel() == 2]
    # print(A.shape)
    A = A[::10, :]
    # print(A.shape)
    points = []
    lim = 8
    # publish point cloud
    for i in range(A.shape[0]):
        x = A[i, 2]
        y = -A[i, 0]
        z = -A[i, 1]
        r = 255
        g = 0
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

    ekf.prediction()

    print("EKF Update:")
    ekf.update(A)
    current_state_ekf = ekf.get_x_est()
    print("x_est", current_state_ekf)
    # calculat 3 vectors for ebene representation such that p=s_v+mu*r_v_1+theta*r_v_1
    s_v = np.asarray([[0, 0, current_state_ekf[3] / current_state_ekf[2]]], dtype="float32")
    r_v_1 = np.asarray([[0.5, 0, (current_state_ekf[3] - 0.5 * current_state_ekf[0]) / current_state_ekf[2]]],
                       dtype="float32")
    r_v_2 = np.cross(r_v_1, np.transpose(current_state_ekf[0:3]))
    r_v_2 = np.asarray(r_v_2, dtype="float32")

    r_v_2 = normalize_vector(r_v_2)
    r_v_1 = normalize_vector(r_v_1)
    print("s_v",s_v)
    print("r_v_1", r_v_1)
    print("r_v_2", r_v_2)
    # print(ekf.get_z_est(0,-0.5))
    # print(ekf.get_z_est(0,0))
    # print(ekf.get_z_est(0,0.5))
    # print(ekf.get_x_est())
    # a = ekf.get_x_est()[0]
    # b = ekf.get_x_est()[1]
    # c = ekf.get_x_est()[2]
    # distance_to_net = PoseStamped()
    # distance_to_net.header.stamp = rospy.Time.now()
    # distance_to_net.header.frame_id = "boat_to_net"  # ned
    # distance_to_net.pose.position.x = a
    # distance_to_net.pose.position.y = b
    # distance_to_net.pose.position.z = c
    #
    # publisher_distance_net.publish(distance_to_net)
    #
    rviz = True
    if rviz:
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
                marker.pose.position.x = current_point[2]  # x
                marker.pose.position.y = -current_point[0]  # y
                marker.pose.position.z = -current_point[1]  # z
                markerArray.markers.append(marker)
                i = i + 1
            publisher_marker.publish(markerArray)

    # cv2.drawContours(frame, contour, -1, (0, 255, 255), 3)
    # for i in range(points.shape[0]):
    #     cv2.circle(frame, tuple(points[i, :]), 2, (0, 0, 255), -1)


def listener():
    rospy.init_node('publisher', anonymous=True)
    ekf = ekf_class.ExtendedKalmanFilter()
    pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
    publisher_marker = rospy.Publisher('detection_net_plane', MarkerArray, queue_size=1)
    rospy.Subscriber("/d435i/depth/color/points", PointCloud2, callback, [pub, ekf, publisher_marker])
    rospy.spin()
    # video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
