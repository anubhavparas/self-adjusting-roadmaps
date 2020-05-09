import math
import numpy as np
import copy


def check_obstacle_collision(current_position, path, obstacle_info):
    if len(obstacle_info) == 2:
        print("Circle")
        center, radius = obstacle_info[1], obstacle_info[0]
        flag = 0
        collided_points = []
        if is_point_inside_circle(current_position, center, radius):
            return False, None
        for i in range(len(path) - 1):
            if i == 0:
                p1 = current_position
                p2 = path[i][:2]
            else:
                p1 = path[i][:2]
                p2 = path[i + 1][:2]
            if is_point_inside_circle(p2, center, radius):
                print('point inside')
                flag = flag + 1
                collided_points.append((round(p2[0], 1), round(p2[1], 1)))
            if is_line_circle_intersecting(p1, p2, center, radius):
                print('line intersecting')
                flag = flag + 1
        if flag > 0:
            return True, collided_points
        return False, None


def get_covered_sample_nodes(sample_nodes, obstacle_info):
    radius, center = obstacle_info
    covered_samples = set()
    sample_nodes_copy = copy.deepcopy(sample_nodes)
    for sample in sample_nodes_copy:
        if is_point_inside_circle(sample, center, radius):
            covered_samples.add(sample)
            sample_nodes.remove(sample)
    return covered_samples

def adjust_intersecting_edges(c_space, obstacle_info):
    radius, center = obstacle_info
    graph = copy.deepcopy(c_space.graph)
    for parent_key, children in graph.items():
        for child in children:
            if is_line_circle_intersecting(parent_key, child, center, radius):
                parent_children = c_space.graph[parent_key]
                if child in parent_children:
                    ind = parent_children.index(child)
                    del parent_children[ind]

                child_children = c_space.graph[child]
                if parent_key in child_children:
                    ind = child_children.index(parent_key)
                    del child_children[ind]


def check_repeating(c_space):
    for key, val in c_space.graph.items():
        l_list = len(val)
        l_set = set(val)
        if l_set != l_set:
            print('repeating')
            return False
            

    



def is_point_inside_circle(p1, center, radius):
    p1_x, p1_y = p1[0], p1[1]
    # p2_x, p2_y = p2[0], p2[1]
    center_x, center_y = center[0], center[1]
    if (p1_x - center_x) ** 2 + (p1_y - center_y) ** 2 <= radius ** 2:
        return True
    return False


def is_line_circle_intersecting(p1, p2, center, radius):
    solution = get_line_circle_intersection(p1, p2, center, radius)
    if solution is not None:
        sol1, sol2 = solution[0], solution[1]
        root1, rooty1 = sol1[0], sol1[1]
        root2, rooty2 = sol2[0], sol2[1]
        x_min, x_max = min(p1[0], p2[0]), max(p1[0], p2[0])
        y_min, y_max = min(p1[1], p2[1]), max(p1[1], p2[1])
        if (x_min <= root1 <= x_max) and (y_min <= rooty1 <= y_max) or (x_min <= root2 <= x_max) and (
                y_min <= rooty2 <= y_max):
            return True
    return False


def get_line_circle_intersection(p1, p2, center, radius):
    m, y_int, x_int = lineModelGenerator(p1, p2)
    q = center[1]
    p = center[0]
    inf_slope = False
    if m == np.inf:
        A = 1
        B = -2 * q
        C = q ** 2 + x_int ** 2 + p ** 2 - radius ** 2 - 2 * x_int * p
        inf_slope = True
    else:
        A = 1 + m ** 2
        B = 2 * (m * y_int - m * q - p)
        C = q ** 2 - radius ** 2 + p ** 2 - 2 * y_int * q + y_int ** 2
    disc = B ** 2 - 4 * A * C

    if disc < 0:
        return None
    else:
        if not inf_slope:
            root1 = (-B + math.sqrt(disc)) / (2 * A)
            root2 = (-B - math.sqrt(disc)) / (2 * A)
            rooty1 = m * root1 + y_int
            rooty2 = m * root1 + y_int
        else:
            root1 = x_int
            root2 = root1
            rooty1 = (-B + math.sqrt(disc)) / (2 * A)
            rooty2 = (-B - math.sqrt(disc)) / (2 * A)
        return (root1, rooty1), (root2, rooty2)


def lineModelGenerator(p1, p2):
    if p1[0] == p2[0]:
        m = np.inf
        y_intercept = np.inf
        x_intercept = p1[0]
    else:
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        y_intercept = p2[1] - m * p2[0]
        if m == 0:
            x_intercept = np.inf
        else:
            x_intercept = -y_intercept / m
    return m, y_intercept, x_intercept
