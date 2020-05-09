import cv2
import numpy as np
import math
from queue import Queue, deque
import time
import matplotlib.pyplot as plt
from simplepriorityqueue import SimplePriorityQueue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from cspaceplotter import CSpacePlotter
from constants import *
from curveplotter import plot_curve
import json
from videowriter import write_video
from heuristics import EUCL_HEURISTIC
import threading
import obstacle_injector as obj_inj
from obstructioncheck import check_obstacle_collision, get_covered_sample_nodes, adjust_intersecting_edges
from adjust_roadmaps import adjust_roadmap
import copy
import traceback


class PathExplorer:

    def __init__(self):
        self.path = []
        self.path_cost = 0
        



    def start_path_planning(self, initial_pos, target_pos, orientation, t_bot, c_space, heuristic_func):
        c_space_nodes, c_space_graph = c_space.get_prm_graph()
        initial_pos = (-2.54, -3) #list(c_space.sample_nodes)[3]  #(-2.54, -3) # (x,y) 
        target_pos = (4,4) #list(c_space.sample_nodes)[-7] # (4,4)

        c_space.get_prm_graph([initial_pos, target_pos], True)
        
        

        orientation = 30
        initial_node = {
            'pos': initial_pos,
            'orientation': orientation,
            'path': [],
            'parent': None,
            'cost': 0, 
            'cost_to_go': heuristic_func(initial_pos, target_pos)
            }

        latest_init_node = {
            'pos': initial_pos,
            'orientation': orientation,
            'path': [],
            'parent': None,
            'cost': 0, 
            'cost_to_go': heuristic_func(initial_pos, target_pos)
            }
        adjust_graph = False

        task_status = {DONE: False}
        bot_status = {MOVING: False, PATH_COVERED: []}
        obstacle_injector = threading.Thread(target=obj_inj.obstacle_injector, args=(task_status, bot_status, c_space))

        self.initialize_matplot(c_space)

        running = True
        obstacle_injector.start()

        try:
            while running:
                is_path_found = self.find_path(latest_init_node, target_pos, t_bot, c_space, heuristic_func)
                if is_path_found:
                    is_goal_reached, last_pos_node = self.start_visualization(latest_init_node, initial_node, target_pos, self.path, self.path_cost, c_space, t_bot, bot_status)
                    if not is_goal_reached:
                        latest_init_node = last_pos_node
                    else:
                        print('Goal Reached')
                        running = False
                else:
                    running = False

            task_status[DONE] = True
        except:
            traceback.print_exc()
            task_status[DONE] = True
                


    def find_path(self, initial_node, target_pos, t_bot, c_space, heuristic_func):
        initial_pos = initial_node['pos'] #initial_node['pos'] #(initial_pos[0], initial_pos[1])
        #orientation = initial_node['orientation']
        target_pos = (target_pos[0], target_pos[1])
        orientation = initial_node['orientation']
        
        if (not self.is_position_valid(initial_pos, c_space)) or (not self.is_position_valid(target_pos, c_space)):
            print("Either initial or target position lies in the obstacle space or out of the configuration space. Please check and try again.")
            return

        node_queue = SimplePriorityQueue()
        visited_nodes_set = set()
        visited_nodes_list = []


        cost_update_map = {}
        cost_update_map[self.get_rounded_pos_orient(initial_node)] = 0

        is_target_found = False

        # cost to reach to the node as a key to sort
        node_queue.put(0, initial_node)

        
        count = 0
        print('Starting path exploration...')
        start = time.clock()
        while (not node_queue.isempty()) and not is_target_found:
            count +=  1
            
            current_node = node_queue.get()[1]
            node_pos_orient = self.get_rounded_pos_orient(current_node)
            #if node_pos_orient not in visited_nodes_set:
            visited_nodes_list.append(current_node)
            visited_nodes_set.add(node_pos_orient)

            if self.is_within_goal_region(current_node['pos'], target_pos):
                print('\nTarget found')
                is_target_found = True
                solution_path, path_cost = current_node['path'], current_node['cost'] 
            else:
                next_positions = self.get_next_positions(current_node, c_space, target_pos, heuristic_func)
                for node in next_positions:
                    rounded_child = self.get_rounded_pos_orient(node)
                    if rounded_child not in visited_nodes_set:
                        old_cost_for_node = math.inf if rounded_child not in cost_update_map.keys() else cost_update_map[rounded_child]
                        new_cost_for_node = node['cost'] + node['cost_to_go']
                        if old_cost_for_node > new_cost_for_node:
                            cost_update_map[rounded_child] = node['cost']
                            node_queue.put(new_cost_for_node, node)

        end = time.clock()

        print('Time taken:', end-start, 'seconds(approx:', int((end-start)/60),'min:', int((end-start)%60), 'sec)' )
        if is_target_found:
            print('Cost of the path: ', path_cost, 'units')
            path_params = self.write_param_json(solution_path, (initial_pos[0], initial_pos[1], orientation))
            self.path = solution_path
            self.path_cost = path_cost
            return is_target_found
        else:
            print('Target cannot be reached')
            return False

        
    def is_within_goal_region(self, position, target_pos, goal_threshold_radius=GOAL_THRESH):
        return math.sqrt((target_pos[0]-position[0])**2 + (target_pos[1]-position[1])**2) <= goal_threshold_radius

    

    def get_rounded_pos_orient(self, node, pos_threshold=0.1, orientation_threshold=30):
        node_pos_x, node_pos_y = node['pos']
        orientation = node['orientation']
        node_pos_x = self.roundoff_value(node_pos_x, pos_threshold)
        node_pos_y = self.roundoff_value(node_pos_y, pos_threshold)
        orientation = self.roundoff_value(orientation, orientation_threshold)

        return (node_pos_x, node_pos_y, orientation)


    def roundoff_value(self, value, roundoff_threshold):
        return (round(value/roundoff_threshold))*roundoff_threshold

    def get_next_positions(self, node, c_space, target_pos, heuristic_func):
        next_postions = []

        c_space_nodes, c_space_graph = c_space.sample_nodes, c_space.graph
        parent_pos = node['pos']
        child_nodes = c_space_graph[parent_pos]
        
        for child in child_nodes:
            next_orient = self.get_next_orientation(child, node)
            velocity = self.get_linear_and_angular_vel(child, node, next_orient)
            node_path = node['path'].copy()
            node_path.append(velocity)
            next_postions.append({
                'pos': tuple(child),
                'orientation': next_orient,
                'path': node_path,
                'parent': (node['pos'], node['orientation']),
                'cost': node['cost'] + (velocity[TRANS][LIN] * velocity[TRANS][TIME_IND]),
                'cost_to_go': heuristic_func(child, target_pos)
            })
        
        return next_postions


    def get_next_orientation(self, child_pos, parent_node):
        parent_pos = np.array(parent_node['pos'])
        child_pos = np.array(child_pos)
        
        x_diff = child_pos[0] - parent_pos[0]
        y_diff = child_pos[1] - parent_pos[1]
        new_orient = math.degrees(np.arctan2(y_diff, x_diff)) # np.arctan2(y_diff, x_diff) 
        new_orient = (360 + new_orient) if new_orient < 0 else new_orient #(TWO_PI + new_orient) if new_orient < 0 else new_orient
        return new_orient
    
    def get_linear_and_angular_vel(self, child_pos, parent_node, next_orient):
        dist = EUCL_HEURISTIC(parent_node['pos'], child_pos)
        linear_v  = dist / DELTA_T_LIN

        parent_orient = parent_node['orientation']
        delta_angle = next_orient - parent_orient
        optimal_rot_ang = min(abs(delta_angle), (360 - abs(delta_angle)))
        #print(delta_angle)
        direction = delta_angle/abs(delta_angle) if delta_angle != 0 else 1
        optimal_rot_ang *= direction if optimal_rot_ang == abs(delta_angle) else -direction
        angular_v = math.radians(optimal_rot_ang) / DELTA_T_ANG

        return (0, angular_v, DELTA_T_ANG), (linear_v, 0, DELTA_T_LIN)


    def adjust_angle(self, orientation):
        orientation = math.degrees(orientation)
        if orientation < 0:
            orientation = 360 + orientation
        elif orientation >= 360:
            orientation = orientation % 360
        return orientation

    def is_position_valid(self, position, c_space):
        return not c_space.is_point_in_obstacle(position[0], position[1])


    def start_visualization(self, initial_node, first_init_node, target_pos, path, path_cost, c_space, t_bot, bot_status):
        is_goal_reached = True
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        fig = self.fig
        ax = self.ax
        
        # small circle of GOAL_THRESH unit radius to mark the goal region
        #a_circle = plt.Circle((target_pos[0], target_pos[1]), GOAL_THRESH)
        #ax.add_artist(a_circle)

        cmap_plotter = CSpacePlotter(c_space)
        cmap_plotter.plotMap(fig, ax)

        ax.scatter(initial_node['pos'][0], initial_node['pos'][1], c='r', s=60)
        ax.scatter(first_init_node['pos'][0], first_init_node['pos'][1], c='r', s=60)
        ax.scatter(target_pos[0], target_pos[1], c='g', s=60)
        
        dots = {0: '    ', 1: '.   ', 2: '..  ', 3: '... ', 4: '....'} 
        num_dots = len(dots)
        
        print('Visualization in process...\n')
        #out = cv2.VideoWriter("./media/astar_nonholonomic.mp4", fourcc, 1.0, (509, 524))
        
        position, orient = list(initial_node['pos']), initial_node['orientation']
        x_pos, y_pos = position[0], position[1]

        path_nodes = [(x_pos, y_pos, orient)]
        for action in path:
            x_pos, y_pos, orient, _ = self.plot_curve(ax, (x_pos, y_pos), orient, action, path_nodes, 0, PATH_CLR, bot_status, cmap_plotter, c_space, pause=False)
            path_nodes.append((x_pos, y_pos, orient))
        frame_num = len(bot_status[PATH_COVERED])
        frame_num = frame_num+1 if frame_num != 0 else frame_num 
        plt.savefig('./media/frame'+str(frame_num).zfill(5)+'.png', bbox_inches='tight')

        
        position, orient = list(first_init_node['pos']), first_init_node['orientation']
        x_pos, y_pos = position[0], position[1]
        for i in range(len(bot_status[PATH_COVERED])-1):
            pre_x, pre_y, _ = bot_status[PATH_COVERED][i]
            nex_x, nex_y, _ = bot_status[PATH_COVERED][i+1]
            ax.plot([pre_x, nex_x], [pre_y, nex_y], color=TRACK_CLR)
        
        
        position, orient = list(initial_node['pos']), initial_node['orientation']
        x_pos, y_pos = position[0], position[1]
        bot_path_cost = 0

        pause = True
        latest_init_node = None
        color = TRACK_CLR
        save_fig = True
        # start the injection thread
        print('Bot started moving')
        bot_status[LAST_STATUS] = None
        bot_status[MOVING] = True
        for i, action in enumerate(path):
            x_pos, y_pos, orient, bot_path_cost = self.plot_curve(ax, (x_pos, y_pos), orient, action, path_nodes, bot_path_cost, color, bot_status, cmap_plotter, c_space, pause=pause)
            
            if not bot_status[MOVING] and latest_init_node is None:
                print('Obstruction present')
                last_x, last_y, last_orient, last_bot_path_cost = bot_status[LAST_STATUS]

                bot_status[PATH_COVERED].append((last_x, last_y, last_orient))
                plt.savefig('./media/frame'+str(frame_num).zfill(5)+'.png', bbox_inches='tight')

                obstacle_info = c_space.obstacle_list[-1]
                covered_sample_nodes = get_covered_sample_nodes(c_space.sample_nodes, obstacle_info)
                #covered_points = set(covered_points)
                #covered_points.update(covered_sample_nodes)

                adjust_intersecting_edges(c_space, obstacle_info)
                
                adjust_roadmap(c_space.graph, covered_sample_nodes, c_space.sample_nodes, obstacle_info)
                
                
                c_space.get_prm_graph([(last_x, last_y)], True)
                latest_init_node = self.get_last_node((last_x, last_y, last_orient), path_nodes[0], target_pos, last_bot_path_cost)
                
                is_goal_reached = False
                pause = False
                save_fig = False
                color = GRAPH_CLR

            if len(path_nodes) > 0:
                if bot_status[MOVING]:
                    bot_status[PATH_COVERED].append(path_nodes[0])
                del path_nodes[0]

            if save_fig:
                frame_num = len(bot_status[PATH_COVERED])
                plt.savefig('./media/frame'+str(frame_num).zfill(5)+'.png', bbox_inches='tight')


        if not bot_status[MOVING]:
            return is_goal_reached, latest_init_node
        
        ax.set_title('Goal reached!', fontsize=13)
        plt.savefig('./media/frame.png', bbox_inches='tight')
            

        sample_num = 100
        
        fig.show()
        plt.draw()
        plt.show()
        #out.release()
        #write_video((len(visited_nodes)), sample_num)

        print('\nVisualization Complete.')

        return is_goal_reached, latest_init_node


    
    def plot_curve(self, ax, init_point, init_orient, velocity, path_nodes, bot_path_cost, color, bot_status, cmap_plotter, c_space, pause=False):
        init_orient = math.radians(init_orient)
        prev_x, prev_y = init_point[0], init_point[1]
        new_x, new_y = prev_x, prev_y
        orient = init_orient
        
        dt = velocity[TRANS][TIME_IND]/20 #delta_t/10 #0.1
        linear_v = velocity[TRANS][LIN]
        angular_v = velocity[ROT][ANG]
        
        orient += angular_v * velocity[ROT][TIME_IND]
        orient = math.radians(self.adjust_angle(orient))

        t = 0
        while t < velocity[TRANS][TIME_IND]:
            t = t + dt
            new_x += linear_v * math.cos(orient) * dt
            new_y += linear_v * math.sin(orient) * dt
            ax.plot([prev_x, new_x], [prev_y, new_y], color=color)
            if pause:
                plt.pause(0.1)

            bot_path_cost += linear_v*dt 
            prev_x, prev_y = new_x, new_y

            if pause and not bot_status[MOVING] and bot_status.get(LAST_STATUS) is None:
                #cmap_plotter.plot_cspace_obstacles(ax)
                ax.set_title(OBS_DETECTED, fontsize=13)
                is_obstruction_present = self.examine_bot_stop_reason(c_space, (new_x, new_y), path_nodes)
                if is_obstruction_present:
                    ax.set_title(OBST_PRESENT, fontsize=13)
                    pause = False
                    color = GRAPH_CLR
                    bot_status[LAST_STATUS] = (new_x, new_y, math.degrees(orient), bot_path_cost)
                else:
                    ax.set_title(NO_OBST, fontsize=13)
                    ax.set_title(BOT_MOVING, fontsize=13)
                    print('No obstruction present. resuming...')
                    bot_status[MOVING] = True

        return new_x, new_y, math.degrees(orient), bot_path_cost
    

    def examine_bot_stop_reason(self, c_space, current_pose, path_nodes):
        print('Bot stopped. Checking for obstruction.')
        is_obstruction_present = self.check_for_obsruction(c_space, current_pose, path_nodes)
        return is_obstruction_present
    


    def check_for_obsruction(self, c_space, current_point, remaining_path):
        new_obstacle = c_space.obstacle_list[-1]
        remaining_path = copy.deepcopy(remaining_path)
        del remaining_path[0]
        is_obstructing, covered_points = check_obstacle_collision(current_point, remaining_path, new_obstacle)
        return is_obstructing

    def get_last_node(self, current_pose, parent_node, target_pos, bot_path_cost):
        last_pos = current_pose[:2]
        latest_init_node = {
            'pos': last_pos,
            'orientation': current_pose[2],
            'path': [],
            'parent': parent_node,
            'cost': bot_path_cost, 
            'cost_to_go': EUCL_HEURISTIC(last_pos, target_pos)
            }
        return latest_init_node

    
    def initialize_matplot(self, c_space):
        fig, ax = plt.subplots()
        plt.xlim(c_space.x_limit[0]-0.1, c_space.x_limit[1]+0.1)
        plt.ylim(c_space.y_limit[0]-0.1, c_space.y_limit[1]+0.1)

        plt.grid()
        ax.set_aspect('equal')
        self.fig = fig
        self.ax = ax


    def write_param_json(self, action_path, init_pose):
        params_data = {}
        params_data['velocity'] = list(action_path)
        params_data['delta_time'] = delta_t
        params_data['init_pose'] = init_pose

        file_name = 'params/action_velocity.json'
        with open(file_name, 'w') as outfile:
            json.dump(params_data, outfile)
        return params_data

    
    def init_msg_queue(self, msg_queue):
        with open('msgs.txt', 'r') as msg_file:
            msg_queue.queue = deque(msg_file.readlines())
