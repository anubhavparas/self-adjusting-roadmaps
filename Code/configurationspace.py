import numpy as np
from obstacle_check import *
from prmplanner import PRMPlanner

class ConfigurationSpace:
    def __init__(self, x_limit, y_limit, radius_of_bot=0, clearance=0):
        self.x_limit = x_limit
        self.y_limit = y_limit
        self.height = y_limit[1] - y_limit[0]
        self.width = x_limit[1] - x_limit[0]
        self.padding = radius_of_bot + clearance
        padding = self.padding
        self.graph = {}
        self.sample_nodes = set()
        self.length_map = {}
        


        self.coord_sq1 = np.array([
            [-2.75-padding, 3.75+padding],
            [-1.25+padding, 3.75+padding],
            [-1.25+padding, 2.25-padding],
            [-2.75-padding, 2.25-padding], 
            [-2.75-padding, 3.75+padding]
        ])

        self.coord_sq2 = np.array([
            [-4.75-padding, 0.75+padding],
            [-3.25+padding, 0.75+padding],
            [-3.25+padding, -0.75-padding],
            [-4.75-padding, -0.75-padding],
            [-4.75-padding, 0.75+padding]
        ])

        self.coord_sq3 = np.array([
            [3.25-padding, 0.75+padding],
            [4.75+padding, 0.75+padding],
            [4.75+padding, -0.75-padding],
            [3.25-padding, -0.75-padding],
            [3.25-padding, 0.75+padding]
        ])

        self.circle1 = [(1+padding), (0, 0)]
        self.circle2 = [(1+padding), (2, 3)]
        self.circle3 = [(1+padding), (2, -3)]
        self.circle4 = [(1+padding), (-2, -3)]

        self.boundary_left = np.array([
            [-5, 5],
            [-5+padding, 5],
            [-5+padding, -5],
            [-5, -5]
            
        ])

        self.boundary_right = np.array([
            [5-padding, 5],
            [5, 5],
            [5, -5],
            [5-padding, -5],
        ])

        self.boundary_up = np.array([
            [-5, 5],
            [5, 5],
            [5, 5-padding],
            [-5, 5-padding],
        ])

        self.boundary_down = np.array([
            [-5, -5+padding],
            [5, -5+padding],
            [5, -5],
            [-5, -5],
        ])

        self.obstacle_list = [[(1+padding), (0, 0)]]


    def is_obstacle_in_path(self, p1, p2, icc, arc_rad):
        if is_intersecting_with_boundary(p2[0], p2[1], self.x_limit, self.y_limit, self.padding):
            return True
        
        for coord_sq in list([self.coord_sq1, self.coord_sq2, self.coord_sq3]):
           if is_arc_intersecting_polygon(p1, p2, icc, arc_rad, coord_sq):
               return True
        
        for circle in list([self.circle1, self.circle2, self.circle3, self.circle4]):
           if is_arc_intersecting_circle(p1, p2, icc, arc_rad, circle):
               return True
        
        return False
    

    def is_point_in_obstacle(self, x, y):
        '''
        if is_intersecting_with_boundary(x, y, self.x_limit, self.y_limit, self.padding):
            return True

        for coord_sq in list([self.coord_sq1, self.coord_sq2, self.coord_sq3]):
           if is_inside_square(x, y, coord_sq):
               return True
        
        for circle in list([self.circle1, self.circle2, self.circle3, self.circle4]):
           if is_inside_circle(x, y, circle):
               return True
        '''

        return False


    def get_prm_graph(self, nodes=[], adjust_graph=False, max_num_edges=10):
        if not adjust_graph:
            self.planner = PRMPlanner(100, 2, self.x_limit, self.y_limit)
            self.sample_nodes = self.planner.random_position(self.obstacle_list)
            nodes = self.sample_nodes
        self.graph = self.planner.generate_roadmap(nodes, self.graph, self.obstacle_list, max_num_edges)

        self.sample_nodes.update(nodes)

        
        for key, val in self.graph.items():
            self.length_map[key] = len(val)

        print('Length', self.length_map.values())

        return self.sample_nodes, self.graph
        
         




