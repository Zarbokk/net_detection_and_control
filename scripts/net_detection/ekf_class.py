import numpy as np
import rospy


class ExtendedKalmanFilter(object):
    def __init__(self, x0=[0, 1]):  # m b  bei der gleichung point*n=d wobei n=[x y]'
        number_of_states = 2
        """ initialize EKF """
        self.__x_est_0 = np.array([[x0[0]], [x0[1]]]).reshape((number_of_states, 1))
        self.__x_est = self.__x_est_0
        # standard deviations
        self.__sig_x1 = 0.0100
        self.__sig_x2 = 0.0100

        self.__p_mat_0 = np.array(np.diag([self.__sig_x1 ** 2,
                                           self.__sig_x2 ** 2]))
        self.__p_mat = self.__p_mat_0
        # process noise
        self.__sig_w1 = 0.08
        self.__sig_w2 = 0.08
        self.__q_mat = np.array(np.diag([self.__sig_w1 ** 2,
                                         self.__sig_w2 ** 2])) * 0.5

        # measurement noise
        # --> see measurement_covariance_model
        self.__sig_r = 0.10
        self.__r_mat = np.array(np.diag([self.__sig_r ** 2,
                                         self.__sig_r ** 2]))

        # initial values and system dynamic (=eye)
        self.__i_mat = np.eye(2)

        self.__last_time_stamp_update = rospy.get_time()
        self.__last_time_stamp_prediction = rospy.get_time()
        # self.__z_meas = np.zeros(self.__max_tag)
        # self.__y_est = np.zeros(self.__max_tag)
        # self.__r_dist = np.zeros(self.__max_tag)

    def set_x_0(self, x0):
        self.__x_est = x0
        return True

    def set_p_mat_0(self, p0):
        self.__p_mat = p0
        return True

    def reset_ekf(self):
        self.__x_est = self.__x_est_0
        self.__p_mat = self.__p_mat_0

    def get_z_est(self, pos_x, pos_y):
        return self.h(self.__x_est, pos_x, pos_y)

    def get_x_est(self):
        return self.__x_est.round(decimals=3)

    def get_p_mat(self):
        return self.__p_mat

    def normalize_state(self):
        norm = np.linalg.norm(self.__x_est[0:2])
        if norm != 0:
            self.__x_est[0:2] = self.__x_est[0:2] / norm

    # measurement function
    def h(self, x, measurements, number_of_measurements):  # distance between 0 and 1

        return x

    # Jacobian of the measurement function
    def h_jacobian(self, x, measurements, number_of_measurements):  # dim : number_of_measurements X 3 X 4

        return np.eye(2)  # dim : [number_of_measurements X 3 X 4]

    def f_jacobian(self, alpha):
        F = np.eye(2)
        F[0, 0] = (np.sin(alpha) ** 2 + np.cos(alpha) ** 2) / ((np.cos(alpha) - self.__x_est[0] * np.sin(alpha)) ** 2)
        F[0, 1] = 0
        F[1, 0] = self.__x_est[0] * np.sin(alpha) / ((np.cos(alpha) - self.__x_est[1] * np.sin(alpha)) ** 2)
        F[1, 1] = 1 / (np.cos(alpha) - self.__x_est[0] * np.sin(alpha))
        return F

    def prediction(self, roation_z_vel):
        """ prediction """
        delta_t = rospy.get_time() - self.__last_time_stamp_prediction
        self.__last_time_stamp_prediction = rospy.get_time()
        if delta_t == 0:
            delta_t = 0.02

        alpha = roation_z_vel * delta_t
        self.__x_est[0] = (np.sin(alpha) + np.cos(alpha) * self.__x_est[0]) / (
                np.cos(alpha) - np.sin(alpha) * self.__x_est[0])  # m update
        self.__x_est[1] = self.__x_est[1] / (np.cos(alpha) - np.sin(alpha) * self.__x_est[0])  # b update
        self.__p_mat = np.matmul(self.f_jacobian(alpha), np.matmul(self.__p_mat, np.transpose(self.f_jacobian(alpha)))) + self.__q_mat
        return True

    def update(self, measurements):  # z_meas, x_pos, y_pos):
        """ innovation """
        number_of_measurements = measurements.shape[0]
        # iterate through all tx-rss-values
        # for itx in range(self.__tx_num):
        # estimate measurement from x_est
        # print("measurements",measurements)
        z_est = self.h(self.__x_est, measurements, number_of_measurements)
        # print("z_est", z_est.shape)
        # print("measurements",measurements.shape)
        y_tild = measurements.reshape((2, 1)) - z_est
        # print("y_tild",y_tild.shape)
        h_jac_mat = self.h_jacobian(self.__x_est, measurements, number_of_measurements)

        p_matrix_current = self.__p_mat

        s_mat = np.matmul(h_jac_mat,
                          np.matmul(p_matrix_current,
                                    h_jac_mat.transpose())) + self.__r_mat  # = H * P * H^t + R

        k_mat = np.matmul(np.matmul(p_matrix_current, h_jac_mat.transpose()), np.linalg.inv(s_mat))
        # print("k_mat", k_mat.shape)
        self.__x_est = self.__x_est + np.matmul(k_mat, y_tild).reshape(2, 1)  # = x_est + k * y_tild

        self.__p_mat = np.matmul((self.__i_mat - np.matmul(k_mat, h_jac_mat)), self.__p_mat)  # (i-KH)*p mat

        # print("self.__p_mat = " + str(self.__p_mat))
        # print("done")
        # print("x_up= " + str(self.__x_est.transpose()))
        return True
