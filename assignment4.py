import numpy as np
from sim.sim2d_prediction import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['ALLOW_SPEEDING'] = False

class KalmanFilter:
    def __init__(self):
        # Initial State
        self.x = np.matrix([[55.],
                            [3.],
                            [5.],
                            [0.]])

        # Uncertainity Matrix
        self.P = np.matrix([[0., 0.],
                            [0., 0.]])

        # Next State Function
        self.F = np.matrix([[0., 0.],
                            [0., 0.]])

        # Measurement Function
        self.H = np.matrix([[0., 0.]])

        # Measurement Uncertainty
        self.R = np.matrix([[0.0]])
        # Identity Matrix
        self.I = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])
    def predict(self, dt):
        return
    def measure_and_update(self,measurements, dt):
        return [self.x[0], self.x[1]]

    def predict_red_light(self,light_location):
        light_duration = 3
        F_new = np.copy(self.F)
        x_new =
        if x_new[0] < light_location:
            return [False, x_new[0]]
        else:
            return [True, x_new[0]]

    def predict_red_light_speed(self, light_location):
        light_duration = 3
        F_new = np.copy(self.F)
        x_new =
        if x_new[0] < light_location:
            return [False, x_new[0]]
        else:
            return [True, x_new[0]]


for i in range(0,5):
    sim_run(options,KalmanFilter,i)
