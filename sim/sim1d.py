import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import time


def sim_run(options, KalmanFilter):
    start = time.clock()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    CONSTANT_SPEED = options['CONSTANT_SPEED']


    kalman_filter = KalmanFilter()

    def physics(t0,dt,state):
        if len(state) == 0:
            x0 = 0
        else:
            x0 = state[-1]

        if not CONSTANT_SPEED:
            if t0 > 60:
                x1 = 3*dt + x0
                return x1
            if t0 > 40:
                x1 = 2*dt + x0
                return x1
            if t0 > 20:
                x1 = 0.5*dt + x0
                return x1
        x1 = 3*dt + x0
        return x1

    state = []
    est_data_t = []
    v_est_data = []
    x_est_data = []
    t = np.linspace(0.0,100,1001)
    dt = 0.1
    for t0 in t:
        state += [physics(t0,dt,state)]
        if t0%1.0 == 0.0:
            est_data_t += [t0]
            # Measure car location.
            state_with_noise = state[-1]+(np.random.rand(1)[0]-0.5)*0.3
            if t0 == 0.0:
                x_est_data += [0]
                v_est_data += [0]
                continue
            x_est_data += [kalman_filter.predict(t0) - state[-1]]
            kalman_filter.measure_and_update(state_with_noise,t0)
            v_est_data += [kalman_filter.v]


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
    v_est_status = ax2.text(0.0, -0.5, '', fontsize=20, color='g')
    plt.title('V Estimate')
    plt.xticks([])
    ax2.set_ylim([0,4])
    ax2.set_yticks([0,2,4])

    # X Estimate plot.
    ax3 = fig.add_subplot(gs[5:9, 4:])
    x_est, = ax3.plot([], [], '-b')
    x_est_status = ax3.text(0.0,-5, '', fontsize=20, color='g')
    plt.title('X Estimate Error')
    plt.xticks([])
    ax3.set_ylim(-4,4)
    ax3.set_yticks([-4,0,4])

    def update_plot(num):
        t_loc = int(t[num])
        if state[num] > 18:
            ax.set_ylim([0+state[num]-18, 31+state[num]-18])
            time_text.set_position([6,state[num]-18+0.5])

        # Elevator.
        car_l.set_data([2, 2],[state[num], state[num]+10])
        car_r.set_data([6, 6],[state[num], state[num]+10])
        car_t.set_data([2, 6],[state[num]+10, state[num]+10])
        car_b.set_data([2, 6],[state[num], state[num]])

        if int(t[num]) < 20:
            ax2.set_xlim([0,est_data_t[t_loc+1]])
            ax3.set_xlim([0,est_data_t[t_loc+1]])
        else:
            ax2.set_xlim([est_data_t[t_loc-20],est_data_t[t_loc+1]])
            ax3.set_xlim([est_data_t[t_loc-20],est_data_t[t_loc+1]])
            v_est_status.set_position([est_data_t[t_loc-20],-0.5])
            x_est_status.set_position([est_data_t[t_loc-20],-5])
        v_est.set_data(est_data_t[:t_loc], v_est_data[:t_loc])
        x_est.set_data(est_data_t[:t_loc], x_est_data[:t_loc])

        # Timer.
        time_text.set_text(str(100-t[num]))

        # Status
        #if abs( - v_est_data[t_loc]) < :
        #    v_est_status.set_text('PASS')
        if abs(x_est_data[t_loc]) < 0.15:
            x_est_status.set_text('PASS')
        else:
            x_est_status.set_text('')



        return car_l, car_r, car_t, car_b, time_text


    print("Compute Time: ", round(time.clock() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, update_plot, frames=range(1,len(t)), interval=100, repeat=False, blit=False)
    #car_ani.save('lines.mp4')

    plt.show()
