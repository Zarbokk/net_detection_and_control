import ekf_class
import numpy as np
ekf = ekf_class.ExtendedKalmanFilter()
ekf.prediction()

A=np.asarray([[1,1,0],[0,3,2],[3,2,1]])

for i in range(5000):
    print(i)
    ekf.prediction()
    ekf.update(A)




