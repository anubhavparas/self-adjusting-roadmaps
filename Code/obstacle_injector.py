import time
from constants import DONE, MOVING



def obstacle_injector(task_status, bot_status, c_space):
    obstacles = [
        
        [(0.5+c_space.padding), (0,-3)], 
        [(1+c_space.padding), (3,0)]
        #[(1.5+c_space.padding), (3, 2)],
        #[(1.5+c_space.padding), (2,0)],
        
        #[(0.5+c_space.padding), (4,-1)],
        #[(0.5+c_space.padding), (2, 3)],
        #[(0.4+c_space.padding), (-3, 3)]
    ]

    i = 0
    sleep_time = 4
    while not task_status[DONE]:
        time.sleep(sleep_time)
        if i < len(obstacles) and not task_status[DONE]:
            if bot_status[MOVING]:
                c_space.obstacle_list.append(obstacles[i])
                bot_status[MOVING] = False
                print('New obstacle added.')
                i += 1
                sleep_time = 10
        
    print('Task Done')
        

