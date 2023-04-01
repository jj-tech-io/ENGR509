"""2D Kalman Filter for position and velocity.
Starter Code from ENGR 509
Student: Joel Johnson
SN: 37794112"""
import numpy as np
from tracking_2d import simulator_2d

# Simulator options.
options = {}
options['FIG_SIZE'] = [10,10]

options['DRIVE_IN_CIRCLE'] = False
# If False, measurements will be pos_x,pos_y.
# If True, measurements will be pos_x, pos_y, and current angle of the car.

# Required if driving in circle.
options['MEASURE_ANGLE'] = False
options['CONTROL_INPUTS'] = False
options['constant_velocity'] = False


class KalmanFilter:
    def __init__(self):
        self.prev_t = 0
        # Initial State Vector [[pos_x], [pos_y], [velocity_x], [velocity_y]]
        self.x = np.array([[0.], [0.], [0.], [0.]])
        # Uncertainty Matrix
        self.SIGMA = np.array([[1000., 0, 0, 0], [0, 1000., 0, 0], [0, 0, 1000., 0], [0, 0, 0, 1000.]])
        self.R = np.array([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]])
        # State Transition Matrix
        self.A = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])

        # Observation Matrix
        self.C = np.array([[1., 0, 0, 0], [0, 1., 0, 0]])

        # Observation Matrix
        # self.C = np.array([[1., 0]])
        # Measurement Uncertainty
        self.Q = np.array([[0.01], [0.01]])

        # Identity Matrix
        self.I = np.eye(4, )
        print("initializing KalmanFilter")
        print(F"shape of x {self.x.shape}")
        print(F"shape of SIGMA {self.SIGMA.shape}")
        print(F"shape of R {self.R.shape}")
        print(F"shape of A {self.A.shape}")
        print(F"shape of C {self.C.shape}")
        print(F"shape of Q {self.Q.shape}")
        print(F"shape of I {self.I.shape}")

    def predict(self, t):
        # Calculate dt.
        dt = t - self.prev_t
        # Update the state transition matrix.
        self.A = [[1., 0., dt, 0.], [0., 1., 0., dt], [0., 0., 1., 0.], [0., 0., 0., 1.]]
        # # Put dt into the state transition matrix.
        # dt = t - self.prev_t
        self.x = self.A @ self.x
        self.SIGMA = self.A @ self.SIGMA @ np.transpose(self.A) + self.R
        return

    def measure_update(self, measurements, t):
        measurements = np.array(measurements)
        # print(f"measurements {measurements.shape}")
        # print(f"measurements {measurements.shape}")
        # print(f"shape of C {self.C.shape}")
        # print(f"shape of SIGMA {self.SIGMA.shape}")
        # print(f"shape of Q {self.Q.shape}")
        # print(f"shape of measurements {np.array(measurements).shape}")
        # print(f"shape of x {self.x.shape}")
        # print(f"shape of k {np.array(self.SIGMA @ np.transpose(self.C)).shape}")
        # print(f"shape of S {np.array(self.C @ self.SIGMA @ np.transpose(self.C) + self.Q).shape}")
        S = self.C @ self.SIGMA @ np.transpose(self.C) + self.Q
        K = self.SIGMA @ np.transpose(self.C) @ np.linalg.inv(S)
        # print(f"x]{self.x.shape}]+[K]{K.shape}]*[measurements]{np.array([measurements]).shape}")
        measurement_residual = measurements - self.C @ self.x
        # print(f"measurement_residual {measurement_residual.shape}")
        self.x = self.x + K @ measurement_residual
        self.SIGMA = (self.I - K @ self.C) @ self.SIGMA
        # Don't forget to update the time.
        self.prev_t = t

        return self.x[0], self.x[1]

    # Required if driving in circle.
    def control_inputs(self, u_steer, u_pedal):
        return u_steer, u_pedal


simulator_2d(options, KalmanFilter)
