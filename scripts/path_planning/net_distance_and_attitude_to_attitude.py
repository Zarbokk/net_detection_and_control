import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import numpy as np

wanted_depth = 1.5
wanted_distance_to_net = 40
depth = 0
distance_to_net = 0
roll_current = 0
pitch_current = 0


def callback_imu(msg):
    # msg=Imu()
    global roll_current, pitch_current

    tmpquat = Quaternion(w=msg.orientation.w,
                         x=msg.orientation.x,
                         y=msg.orientation.y,
                         z=msg.orientation.z)

    [yaw, pitch_current, roll_current] = tmpquat.inverse.yaw_pitch_roll


def callback_barometer(msg):
    # msg=Imu()
    global depth
    depth = msg.pose.position.z


def callback_net_distance(msg):
    # msg=Imu()
    global distance_to_net
    distance_to_net = msg.pose.position.z


def pathplanning():
    global depth,distance_to_net

    #print("pitch:")
    #print(-np.arctan((wanted_depth - depth) / 1)*180/np.pi)
    print("yaw:")
    print(1*(wanted_distance_to_net-distance_to_net))


def main():
    rospy.init_node('path_planning', anonymous=True)
    rate = rospy.Rate(30)
    rospy.Subscriber("/mavros/imu/data", Imu, callback_imu, queue_size=1)
    rospy.Subscriber("/barometer", PoseStamped, callback_barometer, queue_size=1)
    rospy.Subscriber("/estimated_distance_to_net", PoseStamped, callback_net_distance, queue_size=1)
    while not rospy.is_shutdown():
        pathplanning()
        rate.sleep()
        pass


if __name__ == '__main__':
    main()
