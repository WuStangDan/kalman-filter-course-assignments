import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import time


def sim_run(options, KalmanFilter):
    start = time.clock()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    DRIVE_IN_CIRCLE = options['DRIVE_IN_CIRCLE']


    kalman_filter = KalmanFilter()

    def physics(t0,dt,state):
        if len(state) == 0:
            x0 = 2
            y0 = 3
            v0 = 0
            theta0 = 0
            theta_dot0 = 0
        else:
            x0 = state[-1][0]
            y0 = state[-1][1]
            v0 = state[-1][2]
            theta0 = state[-1][3]
            theta_dot0 = state[-1][4]

        if x0 < 75 and t0 < 30:
            u_pedal = 5
            u_steer = 0
        elif t0 < 30:
            u_pedal = 0
            u_steer = 0
        elif theta0 < 1.45:
            u_pedal = 5
            u_steer = 0.35
        elif y0 < 80:
            u_pedal = 5
            u_steer = 3.14159/2 - theta0
        elif theta0 < 3.0:
            u_pedal = 5
            u_steer = 0.45
        elif x0 > 15:
            u_pedal = 5
            u_steer = 3.14159 - theta0
        else:
            u_pedal = 0
            u_steer = 0

        x1 = v0*np.cos(theta0)*dt + x0
        y1 = v0*np.sin(theta0)*dt + y0
        v1 =(-v0 + 1.0*u_pedal)/2.0*dt + v0
        theta1 = theta_dot0*dt + theta0
        theta_dot1 = u_steer


        return [x1, y1, v1, theta1, theta_dot1]

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
            x_est_data += [0]
            v_est_data += [0]
            #x_est_data += [kalman_filter.predict(t0) - state[-1]]
            #kalman_filter.measure_and_update(state_with_noise,t0)
            #v_est_data += [kalman_filter.v]


    ###################
    # SIMULATOR DISPLAY

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(10,10)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:10, :10])

    plt.xlim(0, 100)
    ax.set_ylim([0, 100])
    plt.xticks([])
    plt.yticks([])
    #plt.yticks(np.arange(0,31,3))
    plt.title('Kalman 2D')


    # Main plot info.
    car, = ax.plot([], [], 'r-', linewidth = 2)
    light, = ax.plot([94,94], [4,2] , 'r-', linewidth = 3)

    # First section.
    ax.plot([1,1], [9,1], 'k-')
    ax.plot([1,87], [9, 9], 'k-')
    ax.plot([1,85], [5,5], 'k--')
    ax.plot([1,95], [1,1], 'k-')
    ax.plot([87,87], [9,1], 'k--')

    # First intersection.
    ax.plot([95,105], [9, 9], 'k-')
    ax.plot([97,105], [5,5], 'k--')
    ax.plot([95,105], [1,1], 'k-')
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

    ax.plot([87,87], [95, 105], 'k-')
    ax.plot([91,91], [97, 105], 'k--')
    ax.plot([95,95], [87, 105], 'k-')
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

        if t_loc >= 29:
            light.set_color('green')

        return car, light


    print("Compute Time: ", round(time.clock() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, update_plot, frames=range(1,len(t)), interval=100, repeat=False, blit=False)
    #car_ani.save('lines.mp4')

    plt.show()
