import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import time
from scipy.integrate import ode


def sim_run(options, KalmanFilter):
    start = time.clock()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]

    kalman_filter = KalmanFilter()


    def physics(t):
        x = t*2 #+ (np.random.rand(1)[0]-0.5)*.3
        return x

    state = []
    v_est_data_x = []
    v_est_data_y = []
    t = np.linspace(0,100,1001)
    for t0 in t:
        state += [physics(t0)]
        if t0%1.0 == 0.0:
            v_est_data_x += [t0]
            v_est_data_y += [kalman_filter.v+0.5+(np.random.rand(1)[0]-0.5)*0.3]


    ###################
    # SIMULATOR DISPLAY

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(14,8)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:, :3])

    plt.xlim(0, 8)
    ax.set_ylim([0, 31])
    plt.xticks([])
    #plt.yticks(np.arange(0,31,3))
    plt.title('Kalman 1d Toy')

    # Time display.
    time_text = ax.text(6, 0.5, '', fontsize=15)


    # Main plot info.
    car_l, car_r = ax.plot([], [], 'k-', [], [], 'k-')
    car_t, car_b = ax.plot([], [], 'k-', [], [], 'k-')

    # V Estimate plot.
    ax2 = fig.add_subplot(gs[0:4, 4:])
    v_est, = ax2.plot([], [], '-b')
    plt.title('V Estimate')
    plt.xticks([])
    ax2.set_ylim([0,4])
    ax2.set_yticks([0,2,4])

    def update_plot(num):
        #print(state[num])

        if state[num] > 18:
            ax.set_ylim([0+state[num]-18, 31+state[num]-18])
            time_text.set_position([6,state[num]-18+0.5])

        # Elevator.
        car_l.set_data([2, 2],[state[num], state[num]+10])
        car_r.set_data([6, 6],[state[num], state[num]+10])
        car_t.set_data([2, 6],[state[num]+10, state[num]+10])
        car_b.set_data([2, 6],[state[num], state[num]])


        if int(t[num]) < 20:
            ax2.set_xlim([0,v_est_data_x[int(t[num])+1]])
        else:
            ax2.set_xlim([v_est_data_x[int(t[num])-20],v_est_data_x[int(t[num])+1]])
        v_est.set_data(v_est_data_x[:int(t[num])], v_est_data_y[:int(t[num])])

        # Timer.
        time_text.set_text(str(100-t[num]))

        return car_l, car_r, car_t, car_b, time_text


    print("Compute Time: ", round(time.clock() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, update_plot, frames=range(0,len(t)), interval=50, repeat=False, blit=False)
    #car_ani.save('lines.mp4')

    plt.show()
