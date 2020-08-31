import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from mavros_msgs.msg import AttitudeTarget

depth_des = 1.0
distance_to_net_des = 1.5
depth = 0
distance_to_net = 0
roll_current = 0
pitch_current = 0
net_plane_parameter = np.asarray([0.0, 0.0, 0.0])
publisher_marker = rospy.Publisher('path_planning', MarkerArray, queue_size=1)
publisher_waypoint = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)


def callback_imu(msg):
    # msg=Imu()
    global roll_current, pitch_current

    tmpquat = Quaternion(w=msg.orientation.w,
                         x=msg.orientation.x,
                         y=msg.orientation.y,
                         z=msg.orientation.z)

    [yaw, pitch_current, roll_current] = tmpquat.inverse.yaw_pitch_roll
    print(pitch_current * 180 / np.pi, roll_current * 180 / np.pi)


def callback_barometer(msg):
    # msg=Imu()
    global depth
    depth = msg.pose.position.z


def callback_net_distance(msg):
    # msg=Imu()
    global net_plane_parameter
    net_plane_parameter[0] = msg.pose.position.x
    net_plane_parameter[1] = msg.pose.position.y
    net_plane_parameter[2] = msg.pose.position.z


def pathplanning():
    global depth, net_plane_parameter, depth_des, distance_to_net_des, publisher_marker, publisher_waypoint
    # transform into real_world_coordinates
    f = 476.0
    c = -net_plane_parameter[2]
    a = net_plane_parameter[1]

    x_real = 400.0 / f * np.abs(c)

    a = a / abs(x_real) * 0.5
    d = np.sqrt((x_real * a) ** 2 + x_real ** 2)
    e = np.sqrt(d ** 2 + distance_to_net_des ** 2)
    alpha = np.pi / 2 + np.arctan(a)  # np.pi - np.pi / 2 - np.arctan(abs(a))
    # if a > 0:
    #    alpha = alpha + np.pi / 2
    gamma = np.arcsin(distance_to_net_des * np.sin(np.pi / 2) / e)

    betta = np.pi - gamma - alpha
    y_max = c + e * np.cos(betta)
    x_max = np.sin(betta) * e

    d_cubic = 0
    c_cubic = a
    b_cubic = (y_max - a * x_max) / (-2.0 / 3.0 * x_max ** 2 + x_max ** 2)
    a_cubic = -2.0 * b_cubic / x_max / 3.0

    stammfunktion = a_cubic * x_max ** 3 + b_cubic * x_max ** 2 + c_cubic * x_max + d_cubic
    pitch_gain = 3
    pitch = pitch_gain * np.arctan((depth - depth_des) / np.sqrt(stammfunktion ** 2 + x_max ** 2))

    ableitung = 3.0 * a_cubic * (x_max / 2.0) ** 2 + 2.0 * b_cubic * (x_max / 2.0) + c_cubic
    yaw = np.arctan(ableitung)
    # pitch = np.pi / 4
    print("pitch:", pitch * 180 / np.pi)

    # yaw=-np.pi/2
    print("yaw:", yaw * 180 / np.pi)
    roll = 0
    qz_90n = Quaternion(
        axis=[0, 0, 1], angle=-(yaw - np.pi / 2)) * Quaternion(axis=[0, 1, 0], angle=-pitch) * Quaternion(
        axis=[1, 0, 0], angle=roll)

    thrust = 0.0
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
            marker.header.frame_id = "local_boat"
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
            marker.pose.position.y = -(a_cubic * x ** 3 + b_cubic * x ** 2 + c_cubic * x + d_cubic)  # y
            marker.pose.position.z = x / x_max * (depth - depth_des)  # z
            markerArray.markers.append(marker)
            i = i + 1
        publisher_marker.publish(markerArray)


def main():
    rospy.init_node('path_planning', anonymous=True)
    rate = rospy.Rate(30)
    rospy.Subscriber("/uuv00/mavros/imu/data", Imu, callback_imu, queue_size=1)
    rospy.Subscriber("/barometer", PoseStamped, callback_barometer, queue_size=1)
    rospy.Subscriber("/estimated_distance_to_net", PoseStamped, callback_net_distance, queue_size=1)
    while not rospy.is_shutdown():
        # pathplanning()
        rate.sleep()
        pass


if __name__ == '__main__':
    main()
