# !/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image  # CompressedImage  # Image
import cv2
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
import ekf_class
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import ModelStates

from pyquaternion import Quaternion

ekf = None
publisher_distance_net = None
rviz = True
publisher_marker = rospy.Publisher('detection_net_plane', MarkerArray, queue_size=1)
publisher_net_marker = rospy.Publisher('real_net_gt', Marker, queue_size=1)


def maxValue(regulate, max_value):
    if regulate > max_value:
        regulate = max_value
    if regulate < -max_value:
        regulate = -max_value
    return regulate


def cubic_function(x, a, b, c, d):
    return a * x ** 3 + b * x ** 2 + c * x + d


def tiefpass(x, x_old, rate=0.5):
    return x * (1 - rate) + x_old * rate


def controller_verfolgen(x, y, alpha):
    # print("distance x:", x, "distance y:", y, alpha * 180 / np.pi, distance)
    x = x - 350

    a = -y / x ** 3 + np.tan(alpha) / x ** 2
    b = y / x ** 2 - a * x
    y_pos = cubic_function(x / 2, a, b, 0, 0)

    theta_wanted = np.arctan2(y_pos, x / 2)
    # print(x / 2, y_pos, theta_wanted * 180 / np.pi)
    # print(theta_wanted)
    # print(theta_car)
    gamma = 1 * (theta_wanted)
    gamma = maxValue(gamma * 180 / 3.14159, 29)
    # print(gamma)
    steering = gamma

    # saved_steering = gamma
    v_wanted = 1 * (x / 2 - 50)
    # v_wanted = maxValue(v_wanted, 4095)
    # accell_in = Kacell * (v_wanted - speed_car)
    accell_in = maxValue(9 * v_wanted, 4000)
    return accell_in, steering


def callback(image):
    global ekf, publisher_distance_net, publisher_marker, rviz
    # print("Start of current picture")
    # print(ekf.get_x_est())
    # print(ekf.get_p_mat())
    ekf.prediction()
    # print(image.encoding)
    brige = CvBridge()
    try:
        frame = brige.imgmsg_to_cv2(image, "passthrough")
        # frame = brige.compressed_imgmsg_to_cv2(image, "passthrough")
    except CvBridgeError as e:
        print(e)

    frame = cv2.GaussianBlur(frame, (5, 5), 10)
    # cv2.imshow('input_image', frame)
    # cv2.waitKey(1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # ret, gray = cv2.threshold(gray, 20, 255, 0)
    # edges = cv2.Canny(gray, 50, 100, apertureSize=3)
    # cv2.imshow('edge', edges)
    # cv2.waitKey(1)
    #
    # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100,maxLineGap=10)
    #
    # if lines is not None:
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    ret, gray = cv2.threshold(gray, 20, 255, 0)
    # cv2.imshow('corner_detection', gray)
    # cv2.waitKey(1)

    contours, hierarchy = cv2.findContours(gray, 1, 2)
    distances_all_squares = list()
    for contour in contours:
        area = cv2.contourArea(contour)

        if (area > 100 and area < 37675):
            # print(contour)
            if False:
                points = np.asarray(contour)[:, 0, :]
                # print(points)
                difference_array = np.zeros([points.shape[0], points.shape[0]])
                for i in range(points.shape[0]):
                    current_candidate = points[i, :]
                    rest_candidates = points  # points[np.arange(len(points))!=i]
                    # print(current_candidate)
                    # print("points")
                    # print(points.shape)
                    # print("rest")
                    # print(rest_candidates.shape)
                    difference_array[i, :] = np.sum(np.abs(current_candidate - rest_candidates) ** 2, axis=-1) ** (
                            1. / 2)
                for_min = difference_array + np.eye(difference_array.shape[0]) * 1000
                while points.shape[0] > 4:
                    # print(difference_array)
                    # print(for_min)
                    current_points_to_remove = np.where(for_min == np.min(for_min))[0]
                    if np.max(difference_array[current_points_to_remove[0], :]) > np.max(
                            difference_array[current_points_to_remove[1], :]):
                        # remove current_points_to_remove[1]
                        # print(current_points_to_remove[1])
                        points = np.delete(points, current_points_to_remove[1], 0)
                        # print(difference_array.shape)
                        mask = np.ones_like(difference_array, dtype=bool)
                        mask[current_points_to_remove[1], :] = False
                        mask[:, current_points_to_remove[1]] = False
                        difference_array = np.reshape(difference_array[mask], (points.shape[0], points.shape[0]))
                        for_min = np.reshape(for_min[mask], (points.shape[0], points.shape[0]))
                    else:
                        # remove current_points_to_remove[0]
                        # print(current_points_to_remove[0])
                        points = np.delete(points, current_points_to_remove[0], 0)
                        mask = np.ones_like(difference_array, dtype=bool)
                        mask[current_points_to_remove[0], :] = False
                        mask[:, current_points_to_remove[0]] = False
                        difference_array = np.reshape(difference_array[mask], (points.shape[0], points.shape[0]))
                        for_min = np.reshape(for_min[mask], (points.shape[0], points.shape[0]))

                    # print(current_points_to_remove)
                    # print(current_candidate-points[:,0,:])
                    #     array=np.sum(np.abs(current_candidate-points[0:,0,:]) ** 2, axis=-1) ** (1. / 2))
                    # min_v=np.min(array)
                    # print(array)
                    # print(np.where(array == np.min(array) and np.min(array)!= 0))
                    # print(points)
                # print(points)
                # exit()
                # print(points.shape)
                # print(points[0, :])
                # cv2.circle(frame, (447,63), 2, (0, 0, 255), -1)

                #
                # # print(area)
                # leftmost = tuple(contour[contour[:, :, 0].argmin()][0])
                # rightmost = tuple(contour[contour[:, :, 0].argmax()][0])
                # topmost = tuple(contour[contour[:, :, 1].argmin()][0])
                # bottommost = tuple(contour[contour[:, :, 1].argmax()][0])

                # cv2.circle(frame, leftmost, 2, (0, 0, 255), -1)
                # cv2.circle(frame, rightmost, 2, (0, 0, 255), -1)
                # cv2.circle(frame, topmost, 2, (0, 0, 255), -1)
                # cv2.circle(frame, bottommost, 2, (0, 0, 255), -1)

                # rect = cv2.minAreaRect(contour)
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                # ellipse = cv2.fitEllipse(contour)
                # frame = cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
                # frame = cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                # print(points.shape)
                # print(points)
                # print('tmp1')

                points = points[np.argsort(points[:, 0]), :]
                linke_seite_rechteck = points[0:2, :]
                links_oben = points[np.where(linke_seite_rechteck[:, 1] == np.min(linke_seite_rechteck[:, 1]))[0], :][0]
                links_unten = points[np.where(linke_seite_rechteck[:, 1] == np.max(linke_seite_rechteck[:, 1]))[0], :][
                    0]

                rechte_seite_rechteck = points[2:4, :]
                rechts_oben = \
                    points[np.where(rechte_seite_rechteck[:, 1] == np.min(rechte_seite_rechteck[:, 1]))[0] + 2, :][0]
                rechts_unten = \
                    points[np.where(rechte_seite_rechteck[:, 1] == np.max(rechte_seite_rechteck[:, 1]))[0] + 2, :][0]

                # print(links_oben)
                # print(links_unten)
                # print(rechts_oben)
                # print(rechts_unten)

                cv2.circle(frame, tuple(links_oben), 2, (0, 0, 255), -1)
                cv2.circle(frame, tuple(links_unten), 3, (0, 0, 255), -1)
                cv2.circle(frame, tuple(rechts_oben), 4, (0, 0, 255), -1)
                cv2.circle(frame, tuple(rechts_unten), 5, (0, 0, 255), -1)

                # print(points)
                # print('tmp2')
                # pts_src = np.array([links_unten, links_oben, rechts_oben, rechts_unten])
                # pts_dst = np.array([[0, 0], [0, 1], [1, 1], [1, 0]])
                # h, status = cv2.findHomography(pts_src, pts_dst)
                # print(h.shape)
                # h = np.asarray(h)
                # print(h)
                # print(np.matmul(h, np.transpose([0, 0, 1])))
                # K = np.asarray([[476.7030836014194, 0.0, 400.5], [0.0, 476.7030836014194, 400.5], [0.0, 0.0, 1.0]])
                # print(K)
                # r = cv2.decomposeHomographyMat(h, K)
                # print(np.asarray(r).shape)
                # print("rotation:")
                # for i in range(4):
                #     print(r[2][i])
                #     #print(r[3][i])
                #
                #     rotation = R.from_dcm(r[1][i])
                #     print(rotation.as_quat())
                # print(r[2][i])
                # print(r[3][i])

                # rotation = R.from_dcm(r[1][3])
                # print(rotation.as_euler('xyz', degrees=True))
            x_pos, y_pos, w, h = cv2.boundingRect(contour)

            distances_all_squares.append(
                [476 * np.sqrt(0.165 * 0.145 / area), (float(x_pos) - 400) / 800, (float(y_pos) - 400) / 800, ])
    cv2.imshow('Hough_detection', frame)
    cv2.waitKey(1)
    # break

    print("EKF Update:")
    distances_all_squares = np.asarray(distances_all_squares)
    np.random.shuffle(distances_all_squares)
    ekf.update(distances_all_squares)
    # print(ekf.get_z_est(0,-0.5))
    # print(ekf.get_z_est(0,0))
    # print(ekf.get_z_est(0,0.5))
    print(ekf.get_x_est())
    a = ekf.get_x_est()[0]
    b = ekf.get_x_est()[1]
    c = ekf.get_x_est()[2]
    distance_to_net = PoseStamped()
    distance_to_net.header.stamp = rospy.Time.now()
    distance_to_net.header.frame_id = "boat_to_net"  # ned
    distance_to_net.pose.position.x = a
    distance_to_net.pose.position.y = b
    distance_to_net.pose.position.z = c

    publisher_distance_net.publish(distance_to_net)

    if rviz:
        x_real = 400.0 / 476.0 * abs(c)
        markerArray = MarkerArray()
        i = 1
        for y in np.linspace(-x_real, x_real, 10):
            for x in np.linspace(-x_real, x_real, 10):
                r = 0.1
                marker = Marker()
                marker.header.frame_id = "local_boat"
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = r * 2  # r*2
                marker.scale.y = r * 2
                marker.scale.z = r * 2
                marker.color.r = 1
                marker.color.g = 1
                marker.color.a = 1  # transparency
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x  # x
                marker.pose.position.y = -(-c + b / abs(x_real) * 0.5 * x)  # y
                marker.pose.position.z = y  # z
                markerArray.markers.append(marker)
                i = i + 1
            publisher_marker.publish(markerArray)

    # cv2.drawContours(frame, contour, -1, (0, 255, 255), 3)
    # for i in range(points.shape[0]):
    #     cv2.circle(frame, tuple(points[i, :]), 2, (0, 0, 255), -1)


def draw_net_rviz(msg):
    global publisher_net_marker
    orientation_boat = Quaternion(w=msg.pose[0].orientation.w,
                                  x=msg.pose[0].orientation.x,
                                  y=msg.pose[0].orientation.y,
                                  z=msg.pose[0].orientation.z)

    pose_boat = np.array([msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z])
    pose_boat = orientation_boat.inverse.rotate(-pose_boat)
    marker = Marker()
    marker.header.frame_id = "local_boat"
    marker.id = 0
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.scale.x = 4
    marker.scale.y = 4
    marker.scale.z = 2
    marker.color.r = 0
    marker.color.g = 1
    marker.color.a = 0.2  # transparency
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = pose_boat[0]
    marker.pose.position.y = pose_boat[1]
    marker.pose.position.z = pose_boat[2]-1
    publisher_net_marker.publish(marker)


def listener():
    geschwindigkeit = 0.5
    # tracker = tracking_red_dots(308,410)
    # tracker = tracking_red_dots(960, 1280,350,900,400,960)
    global ekf, publisher_distance_net, rviz

    ekf = ekf_class.ExtendedKalmanFilter()

    rospy.init_node('publisher', anonymous=True)
    publisher_distance_net = rospy.Publisher('/estimated_distance_to_net', PoseStamped, queue_size=1)
    rospy.Subscriber("/multisense_sl/camera/left/image_raw", Image, callback)
    if rviz:
        rospy.Subscriber("/gazebo/model_states", ModelStates, draw_net_rviz)
    rospy.spin()
    # video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()
