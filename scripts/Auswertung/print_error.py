from pyquaternion import Quaternion
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from gazebo_msgs.msg import ModelStates

error_pos = list()
x_px4_pos = list()
y_px4_pos = list()
z_px4_pos = list()
timestamp_px4_pos = list()

current_time = 0


def callback(msg, publisher):  # sqrt((x1-h)+(y1-k))-r=d
    global x_px4_pos, y_px4_pos, z_px4_pos, error_pos, timestamp_px4_pos
    d = np.sqrt((msg.pose[0].position.x - 0) ** 2 + (msg.pose[0].position.y - 0) ** 2) - 2
    error = abs(d) - 1.5

    x_px4_pos.append(msg.pose[0].position.x)
    y_px4_pos.append(msg.pose[0].position.y)
    z_px4_pos.append(msg.pose[0].position.z)
    error_pos.append(error)
    timestamp_px4_pos.append((rospy.Time.now()).to_sec() - current_time)

    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.point.x = error
    publisher.publish(msg)


def myhook():
    global x_px4_pos, y_px4_pos, z_px4_pos, error_pos, timestamp_px4_pos
    print("saving")
    np.savetxt("px4_pos.csv",
               np.transpose([np.asarray(x_px4_pos, dtype=np.float32),
                             np.asarray(y_px4_pos, dtype=np.float32),
                             np.asarray(z_px4_pos, dtype=np.float32),
                             np.asarray(error_pos, dtype=np.float32),
                             np.asarray(timestamp_px4_pos, dtype=np.float32)]), delimiter=",")


def main():
    rospy.init_node('path_planning', anonymous=True)
    publisher = rospy.Publisher('/error_calculation', PointStamped, queue_size=1)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, publisher, queue_size=1)
    rospy.on_shutdown(myhook)
    rospy.spin()


if __name__ == '__main__':
    main()
