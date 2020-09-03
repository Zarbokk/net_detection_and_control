import rospy

from pyquaternion import Quaternion
import numpy as np
from mavros_msgs.msg import AttitudeTarget

from visualization_msgs.msg import Marker, MarkerArray
from net_detection_and_control.msg import ekf_data


class PathPlanning(object):
    def __init__(self):
        self.depth_des = 0.8
        self.distance_to_net = 1.1
        # distance_to_net_des = 1.5
        self.depth = 0
        # distance_to_net = 0
        self.roll_current = 0
        self.pitch_current = 0
        self.net_plane_parameter = np.asarray([0.0, 0.0, 0.0, 0.0])

    def normalize_vector(self, v):
        norm = np.linalg.norm(v)
        if norm != 0:
            v = v / norm
        return v

    def callback_imu(self, msg):
        # msg=Imu()
        #global roll_current, pitch_current

        tmpquat = Quaternion(w=msg.orientation.w,
                             x=msg.orientation.x,
                             y=msg.orientation.y,
                             z=msg.orientation.z)

        [yaw, self.pitch_current, self.roll_current] = tmpquat.inverse.yaw_pitch_roll
        # print(pitch_current * 180 / np.pi, roll_current * 180 / np.pi)

    def callback_barometer(self, msg):
        # msg=Imu()
        #global depth
        self.depth = msg.pose.position.z

    def callback_net_distance(self, msg):
        # msg=Imu()
        #global net_plane_parameter
        self.net_plane_parameter[0] = msg.n1_x
        self.net_plane_parameter[1] = msg.n1_y
        self.net_plane_parameter[2] = msg.n1_z
        self.net_plane_parameter[3] = msg.d1

    def pathplanning(self,publisher_waypoint,publisher_marker):
        #global depth, net_plane_parameter, depth_des, distance_to_net_des, publisher_marker, publisher_waypoint
        # transform into real_world_coordinates

        s_v = np.asarray([[0, 0, self.net_plane_parameter[1]]], dtype="float32")
        r_v_1 = np.asarray([[1,
                             0,
                             self.net_plane_parameter[0] + self.net_plane_parameter[1]]],
                           dtype="float32")

        r_v_2 = np.asarray([[0, 1, 0]], dtype="float32")
        r_v_1 = r_v_1 - s_v
        r_v_2 = self.normalize_vector(r_v_2)
        r_v_1 = self.normalize_vector(r_v_1)
        #print("r_v_1", r_v_1)
        point_one = s_v + 2 * r_v_1 + self.distance_to_net * self.normalize_vector(
            np.cross(r_v_2, r_v_1))  # in 1 meter distance and 1.5 meter distance to net
        #print("point_one", point_one)
        z_max = point_one[0, 2]
        x_max = point_one[0, 0]
        a = np.arctan2(r_v_1[0, 2], r_v_1[0, 0])
        #print(a)
        d_cubic = 0
        c_cubic = a
        b_cubic = (z_max - a * x_max) / (-2.0 / 3.0 * x_max ** 2 + x_max ** 2)
        a_cubic = -2.0 * b_cubic / x_max / 3.0

        stammfunktion = a_cubic * x_max ** 3 + b_cubic * x_max ** 2 + c_cubic * x_max + d_cubic
        pitch_gain = 1
        pitch = - pitch_gain * np.arctan((self.depth_des-self.depth) / np.sqrt(stammfunktion ** 2 + x_max ** 2))

        ableitung = a_cubic * (x_max/2.0) ** 3 + b_cubic * (x_max/2.0) ** 2 + c_cubic * x_max/2.0 + d_cubic
        #print("first",- 0.5 * np.arctan(ableitung))

        #ableitung = 3.0 * a_cubic * (x_max / 2.0) ** 2 + 2.0 * b_cubic * (x_max / 2.0) + c_cubic
        #print("second",- 0.5 * np.arctan(ableitung))
        yaw = - 0.5 * np.arctan(ableitung)
        # pitch = np.pi / 4
        # print("pitch:", pitch * 180 / np.pi)

        # yaw=-np.pi/2
        # print("yaw:", yaw * 180 / np.pi)
        roll = -5.0/180.0*np.pi
        # yaw = 0
        qz_90n = Quaternion(
            axis=[0, 0, 1], angle=-(yaw - np.pi / 2)) * Quaternion(axis=[0, 1, 0], angle=-pitch) * Quaternion(
            axis=[1, 0, 0], angle=roll)

        thrust = 0.1
        send_waypoint = AttitudeTarget()
        send_waypoint.type_mask = 0
        send_waypoint.orientation.x = qz_90n.x
        send_waypoint.orientation.y = qz_90n.y
        send_waypoint.orientation.z = qz_90n.z
        send_waypoint.orientation.w = qz_90n.w
        # print(qz_90n.x,qz_90n.y,qz_90n.z,qz_90n.w)
        # 0.2 works
        send_waypoint.thrust = thrust
        publisher_waypoint.publish(send_waypoint)

        # print("pitch:")
        # print(-np.arctan((wanted_depth - depth) / 1)*180/np.pi)
        # print("yaw:")
        # print(1*(wanted_distance_to_net-distance_to_net))

        # Visualisierung Rviz:
        rviz = True

        if rviz:

            markerArray = MarkerArray()
            i = 1
            for x in np.linspace(0, x_max, 20):
                r = 0.1
                marker = Marker()
                marker.header.frame_id = "d435i_depth_optical_frame"
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = r  # r*2 of distance to camera from tag_14
                marker.scale.y = r
                marker.scale.z = r
                marker.color.r = 1
                marker.color.a = 1  # transparency
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x  # x
                marker.pose.position.y = x / x_max * ( self.depth_des-self.depth)  # z
                marker.pose.position.z = a_cubic * x ** 3 + b_cubic * x ** 2 + c_cubic * x + d_cubic  # y
                markerArray.markers.append(marker)
                i = i + 1
            publisher_marker.publish(markerArray)
