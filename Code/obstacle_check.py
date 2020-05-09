import math
import numpy as np
import matplotlib.pyplot as plt

'''
Checking if the arc intersects closed figure
1. Circle
2. Polygon. here, squares
'''

def get_line_circle_intersection(p1, p2, center, radius):
    m, y_int, x_int = lineModelGenerator(p1,p2)
    q = center[1]
    p = center[0]
    inf_slope = False
    if m == np.inf:
        A = 1
        B = -2*q
        C = q**2 + x_int**2 + p**2 - radius**2 - 2*x_int*p
        inf_slope = True
    else:
        A = 1+m**2
        B = 2*(m*y_int - m*q - p)
        C = q**2 - radius**2 + p**2 -2*y_int*q + y_int**2
    disc = B**2 - 4*A*C
   
    if disc < 0:
        return None
    else:
        if not inf_slope:
            root1 = (-B+math.sqrt(disc))/(2*A)
            root2 = (-B-math.sqrt(disc))/(2*A)
            rooty1 = m*root1 + y_int
            rooty2 = m*root1+y_int
        else:
            root1 = x_int
            root2 = root1
            rooty1 = (-B+math.sqrt(disc))/(2*A)
            rooty2 = (-B-math.sqrt(disc))/(2*A)
        return (root1, rooty1), (root2, rooty2)



def get_circle_circle_intersection(icc, arc_rad, obs_center, obs_rad):
    dist_center = math.sqrt((icc[0]-obs_center[0])**2 + (icc[1]-obs_center[1])**2)
    if dist_center > (arc_rad + obs_rad):
        return None
    if dist_center < abs(arc_rad - obs_rad):
        return None

    if dist_center == (arc_rad + obs_rad):
        sol_x = (obs_rad*icc[0] + arc_rad*obs_center[0])/(arc_rad + obs_rad)
        sol_y = (obs_rad*icc[1] + arc_rad*obs_center[1])/(arc_rad + obs_rad)
        return (sol_x, sol_y), (sol_x, sol_y)
    else:
        ax1_trans = (arc_rad**2 - obs_rad**2 + dist_center**2)/(2*dist_center)
        ax2_trans = math.sqrt(arc_rad**2 - ax1_trans**2)
        
        unit_vec_ax1 = (np.subtract([obs_center[0], obs_center[1]], [icc[0], icc[1]]))/dist_center
        rot_mat = np.float32([[0, -1], [1, 0]])  # +90deg rotation
        unit_vec_ax2 = rot_mat.dot(unit_vec_ax1)
        
        root_p1 = np.array([icc[0], icc[1]]) + ax1_trans*unit_vec_ax1 + ax2_trans*unit_vec_ax2
        root_p2 = np.array([icc[0], icc[1]]) + ax1_trans*unit_vec_ax1 - ax2_trans*unit_vec_ax2
        return root_p1, root_p2


def is_arc_intersecting_circle(p1, p2, icc, arc_rad, circle_params):
    obs_center, obs_rad = circle_params[1], circle_params[0]
    is_intersecting = False
    solution = get_circle_circle_intersection(icc, arc_rad, obs_center, obs_rad)
    if solution is not None:
        root1, root2 = solution[0], solution[1]
        if solution_in_range(root1, p1, p2) or solution_in_range(root2, p1, p2):
            is_intersecting = True
    return is_intersecting


def is_arc_intersecting_polygon(p1, p2, icc, arc_rad, poly_coords):
    is_intersecting = False
    for i in range(len(poly_coords)-1):
        vertex1 = poly_coords[i]
        vertex2 = poly_coords[i+1]
        solution = get_line_circle_intersection(vertex1, vertex2, icc, arc_rad)
        if solution is not None:
            root1, root2 = solution[0], solution[1]
            if solution_in_range(root1, p1, p2) or solution_in_range(root2, p1, p2):
                is_intersecting = True
                break
    return is_intersecting


def is_inside_circle(x, y, circle_params):
    center, radius = circle_params[1], circle_params[0]
    return ((x-center[0])**2 + (y - center[1])**2 - (radius)**2) <=0

def is_inside_square(x, y, coord_square):
    X, Y = 0, 1
    min_x, max_x = np.min(coord_square[:, X]), np.max(coord_square[:, X])
    min_y, max_y = np.min(coord_square[:, Y]), np.max(coord_square[:, Y]) 
    return (min_x <= x <= max_x) and (min_y <= y <= max_y)


def is_intersecting_with_boundary(x, y, x_limit, y_limit, padding):
    min_x, max_x = x_limit[0], x_limit[1]
    min_y, max_y = y_limit[0], y_limit[1]
    return (x <= (min_x + padding)) or (x >= (max_x - padding)) or (y <= (min_y + padding))  or (y >= (max_y - padding))


def lineModelGenerator(p1, p2):
    if p1[0] == p2[0]:
        m = np.inf
        y_intercept = np.inf
        x_intercept = p1[0]
    else:
        m = (p2[1]-p1[1])/(p2[0]-p1[0])
        y_intercept = p2[1]-m*p2[0]
        if m == 0:
            x_intercept = np.inf
        else:
            x_intercept = -y_intercept/m
    return m, y_intercept, x_intercept

def solution_in_range(solution, p1, p2):
    sol_x, sol_y = solution[0], solution[1]
    x_min, x_max = min(p1[0], p2[0]), max(p1[0], p2[0])
    y_min, y_max = min(p1[1], p2[1]), max(p1[1], p2[1])
    return (x_min <= sol_x <= x_max) and (y_min <= sol_y <= y_max)




