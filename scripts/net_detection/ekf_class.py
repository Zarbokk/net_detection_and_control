import numpy as np


class ExtendedKalmanFilter(object):
    def __init__(self, x0=[0, 1, 1, 1, 1, -1]):  # s1 s2 r1 r2 l1 l2
        number_of_states = 4
        """ initialize EKF """
        self.__x_est_0 = np.array([[x0[0]], [x0[1]], [x0[2]], [x0[3]]]).reshape((number_of_states, 1))
        self.__x_est = self.__x_est_0
        # standard deviations
        self.__sig_x1 = 0.0100
        self.__sig_x2 = 0.0100
        self.__sig_x3 = 0.0050
        self.__sig_x4 = 0.0050
        self.__sig_x5 = 0.0050
        self.__sig_x6 = 0.0050

        self.__p_mat_0 = np.array(np.diag([self.__sig_x1 ** 2,
                                           self.__sig_x2 ** 2,
                                           self.__sig_x3 ** 2,
                                           self.__sig_x4 ** 2,
                                           self.__sig_x5 ** 2,
                                           self.__sig_x6 ** 2]))
        self.__p_mat = self.__p_mat_0
        # process noise
        self.__sig_w1 = 0.0100
        self.__sig_w2 = 0.0100
        self.__sig_w3 = 0.0050
        self.__sig_w4 = 0.0050
        self.__sig_w5 = 0.0050
        self.__sig_w6 = 0.0050
        self.__q_mat = np.array(np.diag([self.__sig_w1 ** 2,
                                         self.__sig_w2 ** 2,
                                         self.__sig_w3 ** 2,
                                         self.__sig_w4 ** 2,
                                         self.__sig_w5 ** 2,
                                         self.__sig_w6 ** 2])) * 0.5

        # measurement noise
        # --> see measurement_covariance_model
        self.__sig_s = 2
        self.__sig_r1 = 2
        self.__sig_r2 = 2
        self.__sig_l1 = 2
        self.__sig_l2 = 2
        self.__r_mat = np.array(np.diag([self.__sig_s ** 2,
                                         self.__sig_r1 ** 2,
                                         self.__sig_r2 ** 2,
                                         self.__sig_l1 ** 2,
                                         self.__sig_l2 ** 2]))

        # initial values and system dynamic (=eye)
        self.__i_mat = np.eye(4)

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
        norm = np.linalg.norm(self.__x_est[0:3])
        if norm != 0:
            self.__x_est[0:3] = self.__x_est[0:3] / norm

    # measurement function
    def h_r(self, measurements, number_of_measurements):
        # get position of measurement  P=s+mu*r+etha*z
        s1 = self.__x_est[0]
        s2 = self.__x_est[1]
        r1 = self.__x_est[2]
        r2 = self.__x_est[3]
        points_in_plane = np.zeros((number_of_measurements, 2))
        for i in range(number_of_measurements):
            p1 = measurements[i, 0]
            p2 = measurements[i, 1]

            mu = (p1 - s1) / r1 + ((p2 - s2) * r1 * r2 - (p1 - s1) * r2 ** 2) / ((r2 ** 2 + r1 ** 2) * r1)

            points_in_plane[i, 0] = s1 + mu * r1
            points_in_plane[i, 1] = s2 + mu * r2

        return points_in_plane

    def h_s(self, measurements, number_of_measurements):
        # get position of measurement  F= e^2+norm(s)*p  || e = (p-s)
        s1 = self.__x_est[0]
        s2 = self.__x_est[1]
        distance = np.zeros((number_of_measurements, 1))
        for i in range(number_of_measurements):
            p1 = measurements[i, 0]
            p2 = measurements[i, 1]

            e1 = p1 - s1
            e2 = p2 - s2
            error = e1 ** 2 + e2 ** 2
            distance[i, 0] = error
        return distance

    def h_jacobian_s(self, measurements, number_of_measurements):
        # get position of measurement  F= e^2+norm(s)*p  || e = (p-s)
        s1 = self.__x_est[0]
        s2 = self.__x_est[1]
        h_jacobian = np.zeros((number_of_measurements, 2))
        for i in range(number_of_measurements):
            p1 = measurements[i, 0]
            p2 = measurements[i, 1]

            h_jacobian[i, 0] = 2 * s1 - p1
            h_jacobian[i, 1] = 2 * s2 - p2

        return h_jacobian

    def h_jacobian_r(self, measurements, number_of_measurements):  # distance between 0 and 1
        # first just one plane P=s+mu*r+etha*z
        # mu = (p1-s1)/r1+((p2-s2)*r1*r2-(p1-s1)*r2**2)/((r2**2+r1**2)*r1)

        # update for each point coming in.
        s1 = self.__x_est[0]
        s2 = self.__x_est[1]
        r1 = self.__x_est[2]
        r2 = self.__x_est[3]
        l1 = self.__x_est[4]
        l2 = self.__x_est[5]

        h_jacobian = np.zeros((number_of_measurements, 2, 2))
        for i in range(number_of_measurements):
            p1 = measurements[i, 0]
            p2 = measurements[i, 1]

            u = ((p2 - s2) * r1 * r2 - (p1 - s1) * r2 ** 2)
            v = ((r2 ** 2 + r1 ** 2))
            u_d = (p2 - s2) * r2
            v_d = 2 * r1
            h_1_r1 = (u_d * v - u * v_d) / (v ** 2)

            u = ((p2 - s2) * r1 * r2 - (p1 - s1) * r2 ** 2)
            v = ((r2 ** 2 + r1 ** 2))
            u_d = (p2 - s2) * r1 - 2 * (p1 - s1) * r2
            v_d = 2 * r2
            h_1_r2 = (u_d * v - u * v_d) / (v ** 2)

            u = ((p2 - s2) * r1 * r2 ** 2 - (p1 - s1) * r2 ** 3)
            v = ((r2 ** 2 + r1 ** 2) * r1)
            u_d = (p2 - s2) * r2 ** 2
            v_d = (r2 ** 2 + 3 * r1 ** 2)
            h_2_r1 = -(p1 - s1) * r2 / (r1 ** 2) + (u_d * v - u * v_d) / (v ** 2)

            u = ((p2 - s2) * r1 * r2 ** 2 - (p1 - s1) * r2 ** 3)
            v = ((r2 ** 2 + r1 ** 2) * r1)
            u_d = 2 * (p2 - s2) * r1 * r2 - 3 * (p1 - s1) * r2 ** 2
            v_d = 2 * r1 * r2

            h_2_r2 = (p1 - s1) / r1 + (u_d * v - u * v_d) / (v ** 2)

            h_jacobian[i, 0, 0] = h_1_r1
            h_jacobian[i, 0, 1] = h_1_r2

            h_jacobian[i, 1, 0] = h_2_r1
            h_jacobian[i, 1, 1] = h_2_r2

        return h_jacobian

    # Jacobian of the measurement function
    def h_jacobian(self, x, measurements, number_of_measurements):  # dim : number_of_measurements X 3 X 4
        h_jacobian = np.zeros((number_of_measurements, 4))
        for i in range(number_of_measurements):
            first_term = x[0] * measurements[i, 0] + x[1] * measurements[i, 1] + x[2] * measurements[i, 2]
            h_1_n0 = 2 * measurements[i, 0] * first_term - 2 * x[3] * measurements[i, 0]
            h_1_n1 = 2 * measurements[i, 1] * first_term - 2 * x[3] * measurements[i, 1]
            h_1_n2 = 2 * measurements[i, 2] * first_term - 2 * x[3] * measurements[i, 2]
            h_1_d = 2 * x[3] - 2 * first_term
            # print("h_jacobian[i,:,:]",h_jacobian[i,:,:])
            h_jacobian[i, 0] = h_1_n0
            h_jacobian[i, 1] = h_1_n1
            h_jacobian[i, 2] = h_1_n2
            h_jacobian[i, 3] = h_1_d
        return h_jacobian  # dim : [number_of_measurements X 3 X 4]

    def prediction(self):
        """ prediction """
        self.__x_est = self.__x_est  # + np.random.randn(3, 1) * 1  # = I * x_est
        self.__p_mat = np.matmul(self.__i_mat, np.matmul(self.__p_mat, self.__i_mat)) + self.__q_mat
        return True

    def update(self, measurements):  # z_meas, x_pos, y_pos):
        """ innovation """
        number_of_measurements = measurements.shape[0]

        estimated_point_on_plane_r = self.h_r(measurements, number_of_measurements)
        error_to_s = self.h_s(measurements, number_of_measurements)

        y_r = measurements - estimated_point_on_plane_r  # meas - estimated
        y_s = 0 - error_to_s  # theoretisch meas - estimated
        print("y_r", y_r)
        print("y_s", y_s)

        h_jac_mat_r = self.h_jacobian_r(measurements, number_of_measurements)

        h_jac_mat_s = self.h_jacobian_s(measurements, number_of_measurements)

        print("h_jac_mat_s", h_jac_mat_s)
        print("h_jac_mat_r", h_jac_mat_r)

        p_matrix_r = self.__p_mat[2:3, 2:3]
        print("p_matrix_r", p_matrix_r)
        # calculate update r
        KH_SUM = np.zeros((2, 2))
        for i in range(number_of_measurements):
            s_mat = np.matmul(h_jac_mat_r[i, :],
                              np.matmul(p_matrix_r,
                                        h_jac_mat_r[i, :].transpose())) + self.__r_mat[2:3, 2:3]  # = H * P * H^t + R
            print("s_mat", s_mat)

            k_mat = np.matmul(p_matrix_r, h_jac_mat_r[i, :].transpose()) * np.linalg.inv(s_mat)

            print("k_mat = " + str(k_mat))

            self.__x_est[2:3] = self.__x_est[2:3] + np.transpose(k_mat * y_r[0, i])  # = x_est + k * y_tild

            KH_SUM = KH_SUM + np.matmul(k_mat, np.transpose(h_jac_mat[i, :]))  # KH+KH+KH+...
            # self.__p_mat = np.matmul((self.__i_mat - np.matmul(k_mat, h_jac_mat[i, :, :])), self.__p_mat)  # = (I-KH)*P

        p_matrix_current = self.__p_mat
        tmp = np.zeros((4, 4))
        for i in range(number_of_measurements):
            s_mat = np.matmul(h_jac_mat[i, :],
                              np.matmul(p_matrix_current,
                                        h_jac_mat[i, :].transpose())) + self.__r_mat  # = H * P * H^t + R
            print("smat = " + str(np.linalg.inv(s_mat)))

            k_mat = np.matmul(p_matrix_current, h_jac_mat[i, :].transpose()) / s_mat
            # print(i_tag)
            print("k_mat = " + str(k_mat))
            # print("y_tild = " + str(y_tild_red))
            # print("hier")
            # print("y_tild", y_tild[i, :].shape)
            # print(k_mat.transpose())
            # print(measurements)
            # print("h_jac_mat[i, :, :]",h_jac_mat[i, :, :])
            self.__x_est = self.__x_est + np.transpose(k_mat * y_tild[0, i])  # = x_est + k * y_tild
            # self.__x_est[2, 0] = np.abs(self.__x_est[2, 0])
            # print(np.matmul(k_mat, h_jac_mat[i, :, :]))
            tmp = tmp + np.matmul(k_mat, np.transpose(h_jac_mat[i, :]))  # KH+KH+KH+...
            # self.__p_mat = np.matmul((self.__i_mat - np.matmul(k_mat, h_jac_mat[i, :, :])), self.__p_mat)  # = (I-KH)*P
        # print("self.__p_mat = " + str(self.__p_mat))

        # print("tmp",tmp)
        self.__p_mat = np.matmul((self.__i_mat - tmp), self.__p_mat)

        self.normalize_state()
        if self.__x_est[3] > 5:
            self.__x_est[3] = 0
        if self.__x_est[3] < -5:
            self.__x_est[3] = 0
        self.__x_est[1] = 0

        self.normalize_state()
        # print("self.__p_mat = " + str(self.__p_mat))
        # print("done")
        # print("x_up= " + str(self.__x_est.transpose()))
        return True
