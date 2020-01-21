import numpy as np


class ExtendedKalmanFilter(object):
    def __init__(self, x0=[0, 0, 0]):  # a*x+b*y+c
        number_of_states = 3
        """ initialize EKF """
        self.__x_est_0 = np.array([[x0[0]], [x0[1]], [x0[2]]]).reshape((number_of_states, 1))
        self.__x_est = self.__x_est_0
        # standard deviations
        self.__sig_x1 = 0.100
        self.__sig_x2 = 0.100
        self.__sig_x3 = 0.050

        self.__p_mat_0 = np.array(np.diag([self.__sig_x1 ** 2,
                                           self.__sig_x2 ** 2,
                                           self.__sig_x3 ** 2]))
        self.__p_mat = self.__p_mat_0
        # process noise
        self.__sig_w1 = 0.100
        self.__sig_w2 = 0.100
        self.__sig_w3 = 0.050
        self.__q_mat = np.array(np.diag([self.__sig_w1 ** 2,
                                         self.__sig_w2 ** 2,
                                         self.__sig_w3 ** 2]))*0.5

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
    def h(self, x, measurements, number_of_measurements):  # distance between 0 and 1
        z = x[0] * measurements[:, 1] + x[1] * measurements[:, 2] + x[2] * np.ones((1, number_of_measurements))

        return z

    # Jacobian of the measurement function
    def h_jacobian(self, x, measurements):
        h_jac = np.ones((measurements.shape[0], 3))
        h_jac[:, 0] = measurements[:, 1]
        h_jac[:, 1] = measurements[:, 2]
        return h_jac  # dim [num_tag X 3]

    def prediction(self):
        """ prediction """
        self.__x_est = self.__x_est  # + np.random.randn(3, 1) * 1  # = I * x_est
        self.__p_mat = self.__i_mat.dot(self.__p_mat.dot(self.__i_mat)) + self.__q_mat
        return True

    def update(self, measurements):  # z_meas, x_pos, y_pos):
        """ innovation """
        number_of_measurements = measurements.shape[0]
        # iterate through all tx-rss-values
        # for itx in range(self.__tx_num):
        # estimate measurement from x_est
        z_est = self.h(self.__x_est, measurements, number_of_measurements)

        y_tild = measurements[:, 0].reshape(1, number_of_measurements) - z_est

        # print("zmeas = " + str(z_meas.transpose()))
        # print("yest = " + str(y_est.transpose()))
        # print("ytild = " + str(y_tild.transpose()))
        # calc K-gain
        h_jac_mat = self.h_jacobian(self.__x_est, measurements)
        # print("h_jac_mat")
        # print(h_jac_mat.shape)
        # print(i_tag)
        # print(y_tild_red)
        p_matrix_current = self.__p_mat
        for i in range(number_of_measurements):
            s_mat = np.dot(h_jac_mat[i, :],
                           np.dot(p_matrix_current, h_jac_mat[i, :].transpose())) + self.__r_mat  # = H * P * H^t + R
            # print("smat = " + str(s_mat))

            k_mat = np.dot(p_matrix_current, h_jac_mat[i, :].transpose()) / s_mat  # 1/s scalar since s_mat is dim = 1x1
            # print(i_tag)
            # print("k_mat = " + str(k_mat.transpose()))
            # print("y_tild = " + str(y_tild_red))
            # print("hier")
            # print(y_tild[0, i])
            # print(k_mat.transpose())
            # print(measurements)
            # print(h_jac_mat)
            self.__x_est = self.__x_est + (k_mat.transpose() * y_tild[0, i]).reshape(3, 1)  # = x_est + k * y_tild
            # self.__x_est[2, 0] = np.abs(self.__x_est[2, 0])
            self.__p_mat = (self.__i_mat - np.dot(k_mat, h_jac_mat[i, :])) * self.__p_mat  # = (I-KH)*P

        # print("x_up= " + str(self.__x_est.transpose()))
        return True
