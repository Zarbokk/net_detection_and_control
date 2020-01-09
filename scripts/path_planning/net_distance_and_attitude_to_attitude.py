import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

def callback_imu(msg):
    #msg=Imu()
    print(msg.orientation.x)

def callback_barometer(msg):
    #msg=Imu()
    print(msg.pose.position.x)

def callback_net_distance(msg):
    #msg=Imu()
    print(msg.pose.position.x)

def main():
    rospy.init_node('path_planning')

    rospy.Subscriber("/mavros/imu/data", Imu, callback_imu, queue_size=1)
    rospy.Subscriber("/barometer", PoseStamped, callback_barometer, queue_size=1)
    rospy.Subscriber("/estimated_distance_to_net", PoseStamped, callback_net_distance, queue_size=1)
    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()





