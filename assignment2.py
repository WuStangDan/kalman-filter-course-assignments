import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['CONSTANT_SPEED'] = True

class KalmanFilter:
    def __init__(self):
        self.v = 0
        self.prev_time = 0
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
        self.I = np.matrix([[1., 0.],
                            [0., 1.]])
    def predict(self,t):
        # Calculate dt.
        dt =
        # Put dt into the state transition matrix.
        self.F[0,1] = dt

        self.P =  *  * np.transpose(self.F)
        return self.x[0,0]
    def measure_and_update(self,measurements,t):
        dt =
        self.F[0,1] = dt
        Z = np.matrix(measurements)
        y =
        S =
        K =  *  * np.linalg.inv(S)

        self.v = self.x[1,0]
        self.prev_time = t
        return


sim_run(options,KalmanFilter)
