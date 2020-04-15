#!/usr/bin/env python

from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
from sensor_msgs import point_cloud2
import cv2
import rospy
import numpy as np
import ros_numpy
import struct
from std_msgs.msg import Header


def callback(data, list):
    pub, rate = list

    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape[0] * pc.shape[1], 4))
    # print("pc.shape",struct.unpack('BBBB',pc['rgb'][3,3]))
    points[:, 0] = pc['x'].flatten()
    points[:, 1] = pc['y'].flatten()
    points[:, 2] = pc['z'].flatten()
    points[:, 3] = pc['rgb'].flatten()
    # print(points.shape)
    # points = points[::10, :]
    # print(points.shape)
    points = np.float32(points)

    # right

    # PUBLISH CURRENT POINT CLOUD

    points_cloud = []
    for i in range(points.shape[0]):
        x = points[i, 0]
        y = -points[i, 2]
        z = points[i, 1]
        b, g, r, a = struct.unpack("BBBB", points[i, 3])
        # print("a",b,g,r,a)
        # r = 0
        # g = 255
        # b = 0
        a = 150
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        points_cloud.append(pt)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              # PointField('rgb', 12, PointField.UINT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]

    # print points

    header = Header()
    header.frame_id = "base_link"
    pc2 = point_cloud2.create_cloud(header, fields, points_cloud)
    pub.publish(pc2)
    rate.sleep()


def listener():
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(30)
    pub = rospy.Publisher("/d435i/depth/color/points", PointCloud2, queue_size=2)
    rospy.Subscriber("/camera/depth/points", PointCloud2, callback, [pub, rate])
    rospy.spin()
    # video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
