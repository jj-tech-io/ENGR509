import numpy as np
from tracking_1d import simulator_1d


class KalmanFilter:
    def __init__(self):
        self.v = 0.
        self.prev_t = 0.
        # Initial State Vector [[position], [velocity]]
        self.x = np.array([[0.],
                  [0.]])
        # Uncertainity Matrix
        self.SIGMA = np.array([[1000., 0], #1000*np.eye(2,2)
                      [0, 1000.]])
        self.R = np.array([[0.1, 0.],
                    [0., 0.1]])
        # State Transition Matrix [[1., dt],
        #                          [0., 1.]]
        self.A = np.array([[1.,0],
                           [0,1.]])
        # Observation Matrix
        self.C = np.array([[1., 0]])
        # Measurement Uncertainty
        self.Q = np.array([[0.01]])
        # Identity Matrix [[1., 0.],
        #                  [0., 1.]]
        self.I = np.eye(2, )

    def predict(self, t):
        dt = t - self.prev_t
        self.A[0,1] = dt
        self.x = self.A @ self.x
        self.SIGMA = self.A @ self.SIGMA @ self.A.T + self.R

        return self.x[0,0]

    def measure_and_update(self, measurements, t):
        S = self.C @ self.SIGMA @ np.transpose(self.C) + self.Q
        K = self.SIGMA @ np.transpose(self.C) @ np.linalg.inv(S)
        self.x = self.x + K * (np.array([measurements]) - self.C @ self.x)
        self.SIGMA = (self.I - K @ self.C) @ self.SIGMA
        ##don't forget to update the time
        self.prev_t = t
        self.v = self.x[1,0]
        return self.x[0,0]


# Simulator options.
options = {}
options['FIG_SIZE'] = [8, 8]
options['CONSTANT_SPEED'] = False
options["sensor_noise_variance"] = 0.3

simulator_1d(options, KalmanFilter)
