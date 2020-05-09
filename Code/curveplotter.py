from constants import *
import matplotlib.pyplot as plt
from robot import TurtleBot

def plot_curve(ax, init_point, init_orient, velocity, bot_path_cost, adjust_angle, t_bot, color, bot_status, pause=False):
    if not bot_status[MOVING]:
        velocity, init_point, init_orient = bot_status[LAST_STATUS]
        bot_status[MOVING] = True
        print('resume',(velocity, init_point, (init_orient)))

    init_orient = math.radians(init_orient)
    prev_x, prev_y = init_point[0], init_point[1]
    new_x, new_y = prev_x, prev_y
    orient = init_orient
    r = t_bot.wheel_rad
    l = t_bot.dist_bet_wheels
    t = 0
    dt = velocity[TRANS][TIME_IND]/20 #delta_t/10 #0.1
    linear_v = velocity[TRANS][LIN]
    angular_v = velocity[ROT][ANG]
    

    orient += angular_v * velocity[ROT][TIME_IND]
    orient = math.radians(adjust_angle(orient))

    #new_x += linear_v * math.cos(orient) * velocity[TRANS][TIME_IND]
    #new_y += linear_v * math.sin(orient) * velocity[TRANS][TIME_IND]
    #ax.plot([prev_x, new_x], [prev_y, new_y], color=color)

    
    while t < velocity[TRANS][TIME_IND]:
        t = t + dt
        new_x += linear_v * math.cos(orient) * dt
        new_y += linear_v * math.sin(orient) * dt
        #orient += velocity[ANG] * dt
        ax.plot([prev_x, new_x], [prev_y, new_y], color=color)
        if pause:
            plt.pause(0.1)
        bot_path_cost += linear_v*dt 
        prev_x, prev_y = new_x, new_y

        if not bot_status[MOVING]:
            print((velocity, init_point, math.degrees(init_orient)))
            bot_status[LAST_STATUS] = (velocity, init_point, math.degrees(init_orient))
            break
    

    return new_x, new_y, math.degrees(orient), bot_path_cost

if __name__ == '__main__':
    fig, ax = plt.subplots()
    #t_bot = TurtleBot(radius=(0.354/2), clearance=0, wheel_rad=(0.076/2), dist_bet_wheels=0.354)
    t_bot = TurtleBot(radius=(0.105), clearance=clearance_req, wheel_rad=(0.033), dist_bet_wheels=0.16)
    actions = [[5,5],[5,0],[0,5],[5,10],[10,5]] #[[1,40], [40, 40], [20, 40], [40, 0]]

    velocity_list = []
    for omega in actions:
        linear_v = 0.5*t_bot.wheel_rad * (omega[L] + omega[R])
        angular_v = (t_bot.wheel_rad/t_bot.dist_bet_wheels) * (omega[R] - omega[L])
        velocity_list.append([linear_v, angular_v])

    init_x, init_y = 1, 1
    init_angle = 20
    r = t_bot.wheel_rad
    l = t_bot.dist_bet_wheels
    x = init_x
    y = init_y
    theta = init_angle
    print(x,y,theta)
    for v in velocity_list:
        x, y, theta = plot_curve(ax, (x,y), theta, v, t_bot, 'orange')

    plt.grid()

    ax.set_aspect('equal')

    plt.xlim(0,5)
    plt.ylim(0,5)

    plt.show()
    plt.close()
    


