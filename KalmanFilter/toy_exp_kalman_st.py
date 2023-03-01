import numpy as np
from tracking_1d import simulator_1d

# Simulator options.
options = {}
options["FIG_SIZE"] = [8, 8]
options["CONSTANT_SPEED"] = True
options["sensor_noise_variance"] = 0.2


class KalmanFilterToy:
    def __init__(self):
        self.v = 0.  # velocity estimate
        self.prev_pos = 0.
        self.prev_t = 0.

    def predict(self, t):
        """predict position"""
        predict_pos = self.prev_pos + self.v * (t - self.prev_t)
        return predict_pos


    def measure_and_update(self, measure_pos, t):
        """estimate velocity"""
        #z -> measure_pos
        #x -> self.prev_pos
        measure_velocity = (measure_pos - self.prev_pos)/(t-self.prev_t)
        print(measure_velocity)
        K = .5
        self.v = self.v+K*(measure_velocity - self.v)
        self.prev_pos = measure_pos
        self.prev_t = t
        return

simulator_1d(options, KalmanFilterToy)



