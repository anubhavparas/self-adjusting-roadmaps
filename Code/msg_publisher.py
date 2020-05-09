from constants import *
import json
import time

def publish_velocity(linear_vel, angular_vel):
    # ROS publisher logic to be put in here
    print('publishing- ', 'linear_v: ' + str(linear_vel), 'angular_v: ' + str(angular_vel))


def publisher_if_bot_moves_continuously_with_one_val(action_velocity_list, delta_time):
    i = 0
    start_time = time.clock()
    publish_velocity(action_velocity_list[i][LIN], action_velocity_list[i][ANG])
    while True:
        current_time = time.clock()
        time_elapsed = current_time - start_time
        if time_elapsed >= delta_time:
            start_time = time.clock()
            publish_velocity(action_velocity_list[i][LIN], action_velocity_list[i][ANG])
            i = i + 1
        if i >= len(action_velocity_list):
            publish_velocity(0, 0)
            break

def publisher_if_bot_doesnt_move_with_one_val(action_velocity_list, delta_time):
    for velocity in action_velocity_list:
        time_elapsed = 0
        start_time = time.clock()
        while time_elapsed <= delta_time:
            publish_velocity(velocity[LIN], velocity[ANG])
            current_time = time.clock()
            time_elapsed = current_time - start_time


if __name__ == '__main__':
    file_name = 'action_velocity.json'
    with open(file_name) as json_file:
        params = json.load(json_file)
    
    action_velocity_list = params['velocity']
    delta_time = params['delta_time']

    #publisher_if_bot_doesnt_move_with_one_val(action_velocity_list, delta_time)
    publisher_if_bot_moves_continuously_with_one_val(action_velocity_list, delta_time)

    
    

