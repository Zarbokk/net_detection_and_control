import numpy as np


class ExtendedKalmanFilter(object):
    def __init__(self, x0=[1, 0, 0, 0.5]):  # x y z d bei der gleichung point*n=d wobei n=[x y z]'
        number_of_states = 4
        """ initialize EKF """
        self.__x_est_0 = np.array([[x0[0]], [x0[1]], [x0[2]], [x0[3]]]).reshape((number_of_states, 1))
        self.__x_est = self.__x_est_0
        # standard deviations
        self.__sig_x1 = 0.0100
        self.__sig_x2 = 0.0100
        self.__sig_x3 = 0.0050
        self.__sig_x4 = 0.0050

        self.__p_mat_0 = np.array(np.diag([self.__sig_x1 ** 2,
                                           self.__sig_x2 ** 2,
                                           self.__sig_x3 ** 2,
                                           self.__sig_x4 ** 2]))
        self.__p_mat = self.__p_mat_0
        # process noise
        self.__sig_w1 = 0.0100
        self.__sig_w2 = 0.0100
        self.__sig_w3 = 0.0050
        self.__sig_w4 = 0.0050
        self.__q_mat = np.array(np.diag([self.__sig_w1 ** 2,
                                         self.__sig_w2 ** 2,
                                         self.__sig_w3 ** 2,
                                         self.__sig_w4 ** 2])) * 0.5

        # measurement noise
        # --> see measurement_covariance_model
        self.__sig_r = 2
        self.__r_mat = np.array(np.diag([self.__sig_r ** 2,
                                        self.__sig_r ** 2,
                                        self.__sig_r ** 2]))

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
        if norm !=0:
            self.__x_est[0:3] = self.__x_est[0:3] / norm

    # measurement function
    def h(self, x, measurements, number_of_measurements):  # distance between 0 and 1

        z = x[0] * measurements[:, 0] + x[1] * measurements[:, 1] + x[2] * measurements[:, 2] - x[3] * np.ones(
            (1, number_of_measurements))
        point_e = measurements - np.transpose(x[0:3] * z)
        return point_e

    # Jacobian of the measurement function
    def h_jacobian(self, x, measurements, number_of_measurements):  # dim : number_of_measurements X 3 X 4
        h_jacobian = np.zeros((number_of_measurements, 3, 4))
        for i in range(number_of_measurements):
            h_1_n0 = -2 * measurements[i, 0] * x[0] - x[1] * measurements[i, 1] - x[2] * measurements[i, 2] + x[3]
            h_1_n1 = -x[0] * measurements[i, 1]
            h_1_n2 = -x[0] * measurements[i, 2]
            h_1_d = x[0]

            h_2_n0 = -x[1] * measurements[i, 0]
            h_2_n1 = -2 * measurements[i, 1] * x[1] - x[0] * measurements[i, 0] - x[2] * measurements[i, 2] + x[3]
            h_2_n2 = -x[1] * measurements[i, 2]
            h_2_d = x[1]

            h_3_n0 = -x[2] * measurements[i, 0]
            h_3_n1 = -x[2] * measurements[i, 1]
            h_3_n2 = -2 * measurements[i, 2] * x[2] - x[0] * measurements[i, 0] - x[1] * measurements[i, 1] + x[3]
            h_3_d = x[2]

            h_jacobian[i, 0, 0] = h_1_n0
            h_jacobian[i, 0, 1] = h_1_n1
            h_jacobian[i, 0, 2] = h_1_n2
            h_jacobian[i, 0, 3] = h_1_d

            h_jacobian[i, 1, 0] = h_2_n0
            h_jacobian[i, 1, 1] = h_2_n1
            h_jacobian[i, 1, 2] = h_2_n2
            h_jacobian[i, 1, 3] = h_2_d

            h_jacobian[i, 2, 0] = h_3_n0
            h_jacobian[i, 2, 1] = h_3_n1
            h_jacobian[i, 2, 2] = h_3_n2
            h_jacobian[i, 2, 3] = h_3_d
            #print("h_jacobian[i,:,:]",h_jacobian[i,:,:])
        return h_jacobian  # dim : [number_of_measurements X 3 X 4]

    def prediction(self):
        """ prediction """
        self.__x_est = self.__x_est  # + np.random.randn(3, 1) * 1  # = I * x_est
        self.__p_mat = np.matmul(self.__i_mat,np.matmul(self.__p_mat,self.__i_mat)) + self.__q_mat
        return True

    def update(self, measurements):  # z_meas, x_pos, y_pos):
        """ innovation """
        number_of_measurements = measurements.shape[0]
        # iterate through all tx-rss-values
        # for itx in range(self.__tx_num):
        # estimate measurement from x_est
        # print("measurements",measurements)
        self.normalize_state()
        z_est = self.h(self.__x_est, measurements, number_of_measurements)

        y_tild = measurements - z_est
        #print("self.__x_est",self.__x_est)
        #print("z_meas",measurements[0:5,:])
        #print("z_est", z_est[0:5,:])
        #print("y_tild", y_tild[0:5,:])
        #print ("number_of_measurements",number_of_measurements)
        # print("zmeas = " + str(z_meas.transpose()))
        # print("yest = " + str(y_est.transpose()))
        # print("ytild = " + str(y_tild.transpose()))
        # calc K-gain

        h_jac_mat = self.h_jacobian(self.__x_est, measurements, number_of_measurements)
        #print("h_jac_mat",h_jac_mat[0,:,:])
        # print("h_jac_mat")
        # print(h_jac_mat.shape)
        # print(i_tag)
        # print(y_tild_red)
        p_matrix_current = self.__p_mat
        #print("p_matrix_current = " + str(p_matrix_current))
        tmp = np.zeros((4, 4))
        for i in range(number_of_measurements):
            s_mat = np.matmul(h_jac_mat[i, :, :],
                              np.matmul(p_matrix_current,
                                        h_jac_mat[i, :, :].transpose())) + self.__r_mat  # = H * P * H^t + R
            #print("smat = " + str(np.linalg.inv(s_mat)))

            k_mat = np.matmul(np.matmul(p_matrix_current, h_jac_mat[i, :, :].transpose()), np.linalg.inv(s_mat))
            # print(i_tag)
            #print("k_mat = " + str(k_mat))
            # print("y_tild = " + str(y_tild_red))
            # print("hier")
            #print("y_tild", y_tild[i, :].shape)
            # print(k_mat.transpose())
            # print(measurements)
            #print("h_jac_mat[i, :, :]",h_jac_mat[i, :, :])
            self.__x_est = self.__x_est + np.matmul(k_mat, y_tild[i, :]).reshape(4, 1)  # = x_est + k * y_tild
            # self.__x_est[2, 0] = np.abs(self.__x_est[2, 0])
            #print(np.matmul(k_mat, h_jac_mat[i, :, :]))
            tmp = tmp + np.matmul(k_mat, h_jac_mat[i, :, :])  # KH+KH+KH+...
            #self.__p_mat = np.matmul((self.__i_mat - np.matmul(k_mat, h_jac_mat[i, :, :])), self.__p_mat)  # = (I-KH)*P
        #print("self.__p_mat = " + str(self.__p_mat))

        #print("tmp",tmp)
        self.__p_mat = np.matmul((self.__i_mat - tmp), self.__p_mat)



        self.normalize_state()
        if self.__x_est[3]>5:
            self.__x_est[3]=0
        if self.__x_est[3] < 0:
            self.__x_est[3] = 0
        #self.__x_est[2]=0
        #print("self.__p_mat = " + str(self.__p_mat))
        #print("done")
        # print("x_up= " + str(self.__x_est.transpose()))
        return True
