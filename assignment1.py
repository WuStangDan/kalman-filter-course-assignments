import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]


class KalmanFilterToy:
    def __init__(self):
        self.v = 0
        self.prev_x = 0
        self.prev_time = 0
    def measure(self,x,t):
        prediction = 0
        # Return prediction.
        return prediction


sim_run(options,KalmanFilterToy)
