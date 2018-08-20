import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]

options['DRIVE_IN_CIRCLE'] = False
# If False, measurements will be x,y.
# If True, measurements will be x,y, and current angle of the car.
# Required if you want to pass the driving in circle.
options['MEASURE_ANGLE'] = False
options['RECIEVE_INPUTS'] = False

class KalmanFilter:
    def __init__(self):
        # Initial State
        self.x = np.matrix([[0.],
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

    def recieve_inputs(self, u_steer, u_pedal):
        return

sim_run(options,KalmanFilter)
