import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]


class KalmanFilterToy:
    def __init__(self):
        self.v = 0
    def run(self,x):
        return x


sim_run(options,KalmanFilterToy)
