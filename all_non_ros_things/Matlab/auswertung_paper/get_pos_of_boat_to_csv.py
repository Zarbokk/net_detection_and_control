from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np



def callback(data):
    global current_gantry_pos_x, current_gantry_pos_y, current_gantry_pos_z
    current_gantry_pos_x = data.pos_gantry.x
    current_gantry_pos_y = data.pos_gantry.y
    current_gantry_pos_z = data.pos_gantry.z

def shutdown_hook():
    global list_point_cloud
    print("saving")
    print("np.asarray(list_point_cloud, dtype=np.float32)", np.asarray(list_point_cloud, dtype=np.float32).shape)
    np.savetxt("point_cloud.csv", np.asarray(list_point_cloud, dtype=np.float32), delimiter=",")
    np.savetxt("gantry_pos.csv", np.asarray(list_pos_gantry, dtype=np.float32), delimiter=",")
    print("saved")

def listener():
    rospy.init_node('publisher', anonymous=True)
    rospy.Subscriber("/mavros/local_position/pose_NED", PoseStamped, callback)
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    listener()