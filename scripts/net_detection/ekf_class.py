import numpy as np


class ExtendedKalmanFilter(object):
    def __init__(self, x0=[0, 0, 0]):  # a*x+b*y+c
        number_of_states = 3
        """ initialize EKF """
        self.__x_est_0 = np.array([[x0[0]], [x0[1]], [x0[2]]]).reshape((number_of_states, 1))
        self.__x_est = self.__x_est_0
        # standard deviations
        self.__sig_x1 = 0.500
        self.__sig_x2 = 0.500
        self.__sig_x3 = 0.0500

        self.__p_mat_0 = np.array(np.diag([self.__sig_x1 ** 2,
                                           self.__sig_x2 ** 2,
                                           self.__sig_x3 ** 2]))
        self.__p_mat = self.__p_mat_0
        # process noise
        self.__sig_w1 = 0.500
        self.__sig_w2 = 0.500
        self.__sig_w3 = 0.0500
        self.__q_mat = np.array(np.diag([self.__sig_w1 ** 2,
                                         self.__sig_w2 ** 2,
                                         self.__sig_w3 ** 2]))

        # measurement noise
        # --> see measurement_covariance_model
        self.__sig_r = 1
        self.__r_mat = self.__sig_r ** 2

        # initial values and system dynamic (=eye)
        self.__i_mat = np.eye(3)

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

    # measurement function
    def h(self, x, x_pos, y_pos):  # distance between 0 and 1
        z = x[0] * x_pos + x[1] * y_pos + x[2]

        return z

    # Jacobian of the measurement function
    def h_jacobian(self, x, x_pos, y_pos):
        h_jac = np.array([[x_pos], [y_pos], [1]]).reshape((1, 3))  # ableitung nach x von h

        return h_jac  # dim [num_tag X 3]

    def prediction(self):
        """ prediction """
        self.__x_est = self.__x_est  # + np.random.randn(3, 1) * 1  # = I * x_est
        self.__p_mat = self.__i_mat.dot(self.__p_mat.dot(self.__i_mat)) + self.__q_mat
        return True

    def update(self, z_meas, x_pos, y_pos):
        """ innovation """

        # iterate through all tx-rss-values
        # for itx in range(self.__tx_num):
        # estimate measurement from x_est
        z_est = self.h(self.__x_est, x_pos, y_pos)
        y_tild = z_meas - z_est
        # print("zmeas = " + str(z_meas.transpose()))
        # print("yest = " + str(y_est.transpose()))
        # print("ytild = " + str(y_tild.transpose()))
        # calc K-gain
        h_jac_mat = self.h_jacobian(self.__x_est, x_pos, y_pos)

        # print(i_tag)
        # print(y_tild_red)
        s_mat = np.dot(h_jac_mat, np.dot(self.__p_mat, h_jac_mat.transpose())) + self.__r_mat  # = H * P * H^t + R
        # print("smat = " + str(s_mat))

        k_mat = np.dot(self.__p_mat, h_jac_mat.transpose()) / s_mat  # 1/s scalar since s_mat is dim = 1x1
        # print(i_tag)
        # print("k_mat = " + str(k_mat.transpose()))
        # print("y_tild = " + str(y_tild_red))
        self.__x_est = self.__x_est + k_mat * y_tild  # = x_est + k * y_tild
        # self.__x_est[2, 0] = np.abs(self.__x_est[2, 0])
        self.__p_mat = (self.__i_mat - np.dot(k_mat, h_jac_mat)) * self.__p_mat  # = (I-KH)*P

        # print("x_up= " + str(self.__x_est.transpose()))
        return True
