import numpy as np
from predict_light import sim_kf_predict


# Simulator options.
options = {}
options['FIG_SIZE'] = [8, 8]
options['ALLOW_SPEEDING'] = True

class KalmanFilter:
    def __init__(self):
        self.prev_t = 0.
        # Initial State [[pos_x], [pos_y], [velocity_x], [velocity_y]]
        self.x = np.array([[55.],
                           [3.],
                           [5.],
                           [0.]])
        # Uncertainity Matrix
        self.SIGMA = [0.]
        self.R = [0.]
        # State Transition Matrix
        self.A = [0.]
        # Observation Matrix
        self.C = [0.]
        # Measurement Uncertainty
        self.Q = [0.]

        # Identity Matrix [[1., 0., 0., 0.],
        #                  [0., 1., 0., 0.],
        #                  [0., 0., 1., 0.],
        #                  [0., 0., 0., 1.]]
        self.I = np.eye(4, )

    def predict(self, t):
        dt = t - self.prev_t
        return

    def measure_update(self, measurements, t):
        self.x = [0.]
        return [self.x[0], self.x[1]]

    def predict_light(self, light_location):
        light_duration = 3.
        x_new = [0.]
        if x_new[0] < light_location:
            return [False, x_new[0]]
        else:
            return [True, x_new[0]]

    def predict_light_speeding(self, light_location):
        check = self.predict_light(light_location)
        if check[0]:
            return check
        light_duration = 3   # yellow light lasts 3 seconds
        A_new = np.copy(self.A)
        B_new = np.copy(self.B)
        a = 1.5  # acceleration
        acc_time = 2
        B_new[0] = 0.5 * acc_time**2 # accelerating for one second
        B_new[2] = acc_time # accelerating for one second
        A_new[0, 2] = 1
        A_new[1, 3] = 1
        x_new = A_new @ self.x + B_new * a

        A_new[0, 2] = light_duration - 1
        A_new[1, 3] = light_duration - 1
        x_new = A_new @ x_new
        if x_new[0] < light_location:
            return [False, x_new[0]]
        else:
            return [True, x_new[0]]


for i in range(0, 5):
    '''simulate 5 times'''
    sim_kf_predict(options, KalmanFilter, i)