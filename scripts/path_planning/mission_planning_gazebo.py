import rospy

from path_planning import PathPlanning
from net_detection_and_control.msg import ekf_data
from mavros_msgs.msg import AttitudeTarget

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from pyquaternion import Quaternion

orientation = Quaternion(0, 0, 0, 1)
xyz = np.asarray([0, 0, 0])


def callback(msg):
    global orientation, xyz
    # msg = PoseStamped()
    xyz = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    orientation = Quaternion(x=msg.pose.orientation.x, y=msg.pose.orientation.y, z=msg.pose.orientation.z,
                             w=msg.pose.orientation.w)

    # print("xyz", xyz)


def drive_to_point(pose_des, publisher):
    global orientation, xyz
    pitch_gain = 1

    roll = 0
    pitch = pitch_gain * (xyz[2] - pose_des[2])
    yaw = np.arctan2(pose_des[1] - xyz[1], pose_des[0] - xyz[0]) - orientation.yaw_pitch_roll[0]
    # print("yaw",yaw)
    # print("orientation.yaw_pitch_roll[0]", orientation.yaw_pitch_roll[0])
    # print("np.arctan2(pose_des[1] - xyz[1], pose_des[0] - xyz[0])",np.arctan2(pose_des[1] - xyz[1], pose_des[0] - xyz[0]))
    qz_90n = Quaternion(
        axis=[0, 0, 1], angle=-(yaw - np.pi / 2)) * Quaternion(axis=[0, 1, 0], angle=-pitch) * Quaternion(
        axis=[1, 0, 0], angle=roll)

    thrust = 0.5
    send_waypoint = AttitudeTarget()
    send_waypoint.type_mask = 0
    send_waypoint.orientation.x = qz_90n.x
    send_waypoint.orientation.y = qz_90n.y
    send_waypoint.orientation.z = qz_90n.z
    send_waypoint.orientation.w = qz_90n.w
    # print(qz_90n.x,qz_90n.y,qz_90n.z,qz_90n.w)
    # 0.2 works
    send_waypoint.thrust = thrust * np.linalg.norm(xyz - pose_des) + 0.4
    publisher.publish(send_waypoint)


def move_change_rotation_to(ypr_des, publisher):
    global orientation, xyz

    roll = ypr_des[2]
    pitch = ypr_des[1]
    yaw = ypr_des[0] - orientation.yaw_pitch_roll[0]
    # print("yaw",yaw)
    # print("orientation.yaw_pitch_roll[0]", orientation.yaw_pitch_roll[0])
    # print("np.arctan2(pose_des[1] - xyz[1], pose_des[0] - xyz[0])",np.arctan2(pose_des[1] - xyz[1], pose_des[0] - xyz[0]))
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
    publisher.publish(send_waypoint)


def main():
    global orientation, xyz
    planning_class = PathPlanning()
    rospy.init_node('path_planning', anonymous=True)
    rate = rospy.Rate(30)
    publisher_marker = rospy.Publisher('path_planning', MarkerArray, queue_size=1)
    publisher_waypoint = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    rospy.Subscriber("/mavros/local_position/pose_NED", PoseStamped, callback, queue_size=1)
    rospy.Subscriber("/mavros/imu/data", Imu, planning_class.callback_imu, queue_size=1)
    rospy.Subscriber("/gazebo/barometer", PoseStamped, planning_class.callback_barometer, queue_size=1)

    move_away_position = np.asarray([3, 4, 0])
    move_away_orientation = np.asarray([np.pi, 0, 0])  # y p r
    move_away = True
    while move_away:

        if np.linalg.norm(xyz - move_away_position) > 0.1:
            drive_to_point(move_away_position, publisher_waypoint)
        else:
            move_change_rotation_to(move_away_orientation, publisher_waypoint)
            print("move_to_pos_done")
            if np.linalg.norm(orientation.yaw_pitch_roll - move_away_orientation) < 0.01:
                move_away = False

        rate.sleep()

    move_to_starting_point = True
    print("driving at net")
    move_to_starting_position = np.asarray([0, 0, 0])
    move_to_starting_orientation = np.asarray([0, 0, 0])  # y p r
    while move_to_starting_point:

        if np.linalg.norm(xyz - move_to_starting_position) > 0.1:
            drive_to_point(move_to_starting_position, publisher_waypoint)
        else:
            move_change_rotation_to(move_to_starting_orientation, publisher_waypoint)
            print("move_to_start_done")
            if np.linalg.norm(orientation.yaw_pitch_roll - move_to_starting_orientation) < 0.01:
                move_to_starting_point = False

        rate.sleep()
    rospy.Subscriber("/plane_to_drive_by", ekf_data, planning_class.callback_net_distance, queue_size=1)
    while not rospy.is_shutdown():
        planning_class.pathplanning(publisher_waypoint, publisher_marker)
        rate.sleep()


if __name__ == '__main__':
    main()
