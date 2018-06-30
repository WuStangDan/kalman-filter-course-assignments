import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
import time


def sim_run(options, KalmanFilter, light_state):
    start = time.clock()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    ALLOW_SPEEDING = options['ALLOW_SPEEDING']


    kalman_filter = KalmanFilter()

    def physics(t0,dt,state, u_pedal):
        if len(state) == 0:
            x0 = 55
            y0 = 3
            v0 = 5
            theta0 = 0
            theta_dot0 = 0
        else:
            x0 = state[-1][0]
            y0 = state[-1][1]
            v0 = state[-1][2]
            theta0 = state[-1][3]
            theta_dot0 = state[-1][4]

        u_steer = 0

        x1 = v0*np.cos(theta0)*dt + x0
        y1 = v0*np.sin(theta0)*dt + y0
        v1 =(-v0 + 1.0*u_pedal)/0.8*dt + v0
        theta1 = theta_dot0*dt + theta0
        theta_dot1 = u_steer


        return [x1, y1, v1, theta1, theta_dot1]

    state = []
    est_data_t = []
    x_est_data = []
    noise_data = []
    u_pedal = 5
    predict_x_loc = []
    light_time = [4.5,4.7,4.8,5.0,5.2]
    light_time = light_time[light_state]
    print(light_time)
    first_prediction = True

    t = np.linspace(0.0,100,1001)
    dt = 0.1
    for t0 in t:
        state += [physics(t0,dt,state,u_pedal)]
        if True:#t0%1.0 == 0.0:
            est_data_t += [t0]
            # Measure car location.
            state_with_noise = []
            state_with_noise += [state[-1][0]+(np.random.rand(1)[0]-0.5)*0.5]
            state_with_noise += [state[-1][1]+(np.random.rand(1)[0]-0.5)*0.5]
            noise_data += [state_with_noise]

            if t0 == 0.0:
                x_est_data += [[0,0]]
                continue
            kalman_filter.predict(dt)
            x_est_data += [kalman_filter.measure_and_update(state_with_noise,dt)]
            if t0 >= light_time and first_prediction:
                if not ALLOW_SPEEDING:
                    predict_x_loc = kalman_filter.predict_red_light(95)
                else:
                    predict_x_loc = kalman_filter.predict_red_light_speed(95)
                if not predict_x_loc[0]:
                    u_pedal = 0
                elif ALLOW_SPEEDING:
                    u_pedal = 6
                first_prediction = False


    ###################
    # SIMULATOR DISPLAY

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(10,10)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:10, :10])

    plt.xlim(60, 110)
    ax.set_ylim([-20, 30])
    plt.xticks([])
    plt.yticks([])
    plt.title('Kalman 2D')


    # Main plot info.
    car, = ax.plot([], [], 'b-', linewidth = 5)
    light, = ax.plot([94,94], [4,2] , 'g-', linewidth = 3)
    est, = ax.plot([], [], 'ks', markersize=5, fillstyle='full', linewidth=4)
    est_light, = ax.plot([], [], 'g--', linewidth=1)
    meas, = ax.plot([], [], 'gs', markersize=30, fillstyle='none', linewidth=4)

    # First section.
    ax.plot([1,1], [9,1], 'k-')
    ax.plot([1,87], [9, 9], 'k-')
    ax.plot([1,85], [5,5], 'k--')
    ax.plot([1,95], [1,1], 'k-')
    ax.plot([87,87], [9,1], 'k--')

    # First intersection.
    ax.plot([95,110], [9, 9], 'k-')
    ax.plot([97,110], [5,5], 'k--')
    ax.plot([95,110], [1,1], 'k-')
    ax.plot([95,95], [9,1], 'k--')

    # Second section.
    ax.plot([87,87], [9, 87], 'k-')
    ax.plot([91,91], [11, 85], 'k--')
    ax.plot([95,95], [9, 87], 'k-')
    ax.plot([87,95], [9,9], 'k--')

    #second intersection.
    ax.plot([87,95], [87,87], 'k--')
    ax.plot([87,87], [87,95], 'k--')
    ax.plot([87,95], [95,95], 'k--')

    ax.plot([87,87], [95, 110], 'k-')
    ax.plot([91,91], [97, 110], 'k--')
    ax.plot([95,95], [87, 110], 'k-')
    ax.plot([92,94], [94,94] , 'g-', linewidth = 3)


    # Final Section.
    ax.plot([87,2], [87,87], 'k-')
    ax.plot([87,2], [91,91], 'k--')
    ax.plot([87,2], [95,95], 'k-')
    ax.plot([2,2], [95,87], 'k-')

    def update_plot(num):
        t_loc = int(t[num])

        # Car.
        car_loc = [state[num][0], state[num][1]]
        car_ang = state[num][3]
        car_cos = np.cos(car_ang)
        car_sin = np.sin(car_ang)
        car.set_data([car_loc[0], car_loc[0]+2*car_cos],
                        [car_loc[1], car_loc[1]+2*car_sin])
        # car_zoom.set_data([car_loc[0], car_loc[0]+2*car_cos],
        #                 [car_loc[1], car_loc[1]+2*car_sin])
        # axins.set_xlim(car_loc[0]-5, car_loc[0]+5)
        # axins.set_ylim(car_loc[1]-5, car_loc[1]+5)

        est.set_data([x_est_data[num][0]],[x_est_data[num][1]])

        #meas.set_data([noise_data[num][0]],[noise_data[num][1]])
        # est_zoom.set_data([x_est_data[num][0]],[x_est_data[num][1]])
        # meas_zoom.set_data([noise_data[num][0]],[noise_data[num][1]])
        if t_loc >= light_time:
            light.set_color('orange')
            est_light.set_data([predict_x_loc[1], predict_x_loc[1]],[1,9])
        if t_loc >= light_time+3:
            light.set_color('red')

        return car, light


    print("Compute Time: ", round(time.clock() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, update_plot, frames=range(1,len(t)),
                                    interval=100, repeat=False, blit=False)
    #car_ani.save('lines.mp4')

    plt.show()
