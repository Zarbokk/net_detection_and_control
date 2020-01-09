import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import numpy as np

global depth, distance_to_net, roll, pitch


def callback_imu(msg):
    # msg=Imu()


    tmpquat = Quaternion(w=msg.orientation.w,
                         x=msg.orientation.x,
                         y=msg.orientation.y,
                         z=msg.orientation.z)

    print(tmpquat.inverse.yaw_pitch_roll)





def callback_barometer(msg):
    # msg=Imu()
    # print(msg.pose.position.x)
    return


def callback_net_distance(msg):
    # msg=Imu()
    # print(msg.pose.position.x)
    return


def main():
    rospy.init_node('path_planning')

    rospy.Subscriber("/mavros/imu/data", Imu, callback_imu, queue_size=1)
    rospy.Subscriber("/barometer", PoseStamped, callback_barometer, queue_size=1)
    rospy.Subscriber("/estimated_distance_to_net", PoseStamped, callback_net_distance, queue_size=1)
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
