# !/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from sensor_msgs import point_cloud2
import pcl
from visualization_msgs.msg import Marker, MarkerArray
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import ros_numpy
import struct
from std_msgs.msg import Header
from net_detection_and_control.msg import ekf_data
from sklearn.cluster import DBSCAN
rospy.init_node('publisher', anonymous=True)
rate = rospy.Rate(20)


def callback(data, pub):
    pc = ros_numpy.numpify(data)

    points = np.zeros((pc.shape[0], 4))
    # remap from camera coordinate system to base_link
    points[:, 0] = pc['z'].flatten()
    points[:, 1] = pc['x'].flatten()
    points[:, 2] = pc['y'].flatten()
    points[:, 3] = pc['rgb'].flatten()
    # print(points.shape)
    # points = points[::3, :]
    # print(points.shape)
    # current_input = np.float32(points)
    points = np.float32(points)
    # print(max(points[:, 0]))
    # points = points[points[:, 1] < 5]
    # points = points[points[:, 1] > -5]
    index_of_removal = np.where(points[:, 0] > 2)
    points = np.delete(points, index_of_removal, 0)



    # Get a tuple of unique values & their frequency in numpy array
    uniqueValues, occurCount = np.unique(points[:, 0], return_counts=True)
    print("Unique Values : ", uniqueValues)
    print("Occurrence Count : ", occurCount)
    # print("before", points.shape)

    which_to_remove = np.where(occurCount > 10000)[0]
    # print("which_to_remove", which_to_remove.shape)
    number_to_remove = uniqueValues[which_to_remove]
    # print("number_to_remove", number_to_remove)
    atol = 1e-3  # Absolute tolerance
    for i in number_to_remove:
        index_of_removal = np.where(abs(i - points[:, 0]) < atol)
        points = np.delete(points, index_of_removal, 0)
    print("after", points.shape)
    list_points = []
    for i in range(points.shape[0]):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        r, g, b, a = struct.unpack('BBBB', points[i, 3])
        #r = 0.0
        #g = 80.0*k
        #b = 0.0
        #a = 150.0
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        list_points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1),
              # PointField('rgba', 12, PointField.UINT32, 1),
              ]

    # print points

    header = Header()
    header.frame_id = data.header.frame_id
    pc2 = point_cloud2.create_cloud(header, fields, list_points)
    pub.publish(pc2)

    rate.sleep()


def listener():
    pub_cloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)
    rospy.Subscriber("/d435i/depth/color/points", PointCloud2, callback, pub_cloud, buff_size=65536 * 2)
    rospy.spin()


if __name__ == '__main__':
    listener()
