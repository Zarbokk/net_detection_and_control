import roslib
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField  # CompressedImage  # Image
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import pcl
from std_msgs.msg import Header
import ros_numpy

rospy.init_node('publisher', anonymous=True)
rate = rospy.Rate(20)
import pcl.pcl_visualization

#visual = pcl.pcl_visualization.CloudViewing()


def on_new_point_cloud(data, pub):
    pc = ros_numpy.numpify(data)

    points = np.zeros((pc.shape[0], 4))
    # remap from camera coordinate system to base_link
    points[:, 0] = pc['z'].flatten()
    points[:, 1] = pc['x'].flatten()
    points[:, 2] = pc['y'].flatten()
    points[:, 3] = pc['rgb'].flatten()
    points = np.float32(points)

    index_of_removal = np.where(points[:, 0] > 2)
    points = np.delete(points, index_of_removal, 0)

    p = pcl.PointCloud()
    p.from_list(points[:, 0:3])

    voxel_filter = p.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(0.020, 0.020, 0.020)
    p = voxel_filter.filter()

    stat_outlier_filter = p.make_statistical_outlier_filter()
    stat_outlier_filter.set_mean_k(50)
    stat_outlier_filter.set_std_dev_mul_thresh(2.0)
    p = stat_outlier_filter.filter()

    tree = p.make_kdtree()
    clustering = p.make_EuclideanClusterExtraction()
    clustering.set_ClusterTolerance(0.1)
    clustering.set_MinClusterSize(100)
    clustering.set_MaxClusterSize(250000)
    clustering.set_SearchMethod(tree)
    cluster_indices = clustering.Extract()
    print("cluster_indices", len(cluster_indices))
    number_of_cluster = len(cluster_indices)
    cloud_cluster = pcl.PointCloud()
    cluster = list()
    average_distance = list()
    for j, indices in enumerate(cluster_indices):
        points = np.zeros((len(indices), 3), dtype=np.float32)
        # points = np.zeros((cloudsize, 3), dtype=np.float32)

        # for indice in range(len(indices)):
        for i, indice in enumerate(indices):
            points[i][0] = p[indice][0]
            points[i][1] = p[indice][1]
            points[i][2] = p[indice][2]
        # print("points",points.shape)
        ss = "cloud_cluster_" + str(j) + ".pcd"
        cluster.append(points)
        x_average = np.mean(points[:, 0])
        average_distance.append(x_average)
        # pcl.save(cloud_cluster, ss)
    average_distance = np.asarray(average_distance)
    index_pointcloud = np.argmin(abs(average_distance-0.5))



    first_cluster = pcl.PointCloud()
    first_cluster.from_list(cluster[index_pointcloud])
    np_cloud = np.asarray(first_cluster)
    print("np_cloud",np_cloud.shape)
    for i in range(np_cloud.shape[0]):
        change = -0.4727*np_cloud[i,1]**2 + 0.0769 * np_cloud[i,2]**2
        np_cloud[i, 0] = np_cloud[i, 0] - change


    points = np.asarray(np_cloud)
    print("points", points.shape)
    points = np.float32(points)

    list_points = []

    for i in range(points.shape[0]):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        # r, g, b, a = struct.unpack('BBBB', a[i, 3])
        r = 0.0
        g = 180.0
        b = 0.0
        a = 150.0
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
    publish_pc = pc2.create_cloud(header, fields, list_points)
    pub.publish(publish_pc)

    rate.sleep()


# reference : http://robonchu.hatenablog.com/entry/2017/09/20/234640
def main():
    # rospy.Subscriber("/kinect2/sd/points", PointCloud2, on_new_point_cloud)
    pub_cloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)
    rospy.Subscriber("/d435i/depth/color/points", PointCloud2, on_new_point_cloud, pub_cloud, buff_size=65536 * 2)
    rospy.spin()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
