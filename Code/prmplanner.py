import random
import numpy as np
import math
import obstructioncheck as oc
import copy

class PRMPlanner:
    def __init__(self, n, dist, x_limit, y_limit):
        self.number_of_nodes = n
        self.distance = dist
        self.node = dict(parent=[],child=[])
        self.graph = {}
        self.x_limit = x_limit
        self.y_limit = y_limit
        self.sample = set()

    def addGraph(self, graph, new_child_nodes, new_parent_node):
        if graph.get(new_parent_node) is None:
            graph[new_parent_node] = new_child_nodes
        else:
            values = graph.get(new_parent_node)
            values.extend(new_child_nodes)
            updated_val = set(values)
            graph[new_parent_node] = list(updated_val)

        for child in new_child_nodes:
            if graph.get(child) is None:
                graph[child] = [new_parent_node]
            else:
                if new_parent_node not in set(graph.get(child)):
                    graph.get(child).extend([new_parent_node])

    def random_position(self, obstacle_list):
        random.seed(1)
        x_lo = self.x_limit[0]
        x_hi = self.x_limit[1]
        y_lo = self.y_limit[0]
        y_hi = self.y_limit[1]

        n = 0
        while n < self.number_of_nodes:
            px, py = random.uniform(x_lo, x_hi), random.uniform(y_lo, y_hi)
            px, py = round(px, 1), round(py, 1)

            if (self.within_range(px, py)) and (not self.is_in_obstacles((px,py), obstacle_list)) and (px,py) not in self.sample: #Changed
                self.sample.add((px,py))  
                n = n + 1 
        return self.sample

    def within_range(self, px, py):
        x_lo = self.x_limit[0]
        x_hi = self.x_limit[1]
        y_lo = self.y_limit[0]
        y_hi = self.y_limit[1]
        return (x_lo < px < x_hi and y_lo < py < y_hi)


    def is_in_obstacles(self, point, obstacle_list):
        for obstacle in obstacle_list:
            if oc.is_point_inside_circle(point, obstacle[1], obstacle[0]):
                return True
        return False

    def is_edge_intersecting(self, p1, p2, obstacle_list):
        for obstacle in obstacle_list:
            if oc.is_line_circle_intersecting(p1, p2, obstacle[1], obstacle[0]):
                return True
        return False

    def get_euclidean_distance(self, current_position, next_position):
        x1, y1 = current_position[0], current_position[1]
        x2, y2 = next_position[0], next_position[1]
        distance = np.sqrt((math.pow(x2 - x1, 2)) + (math.pow(y2 - y1, 2)))
        return distance

    def within_distance(self, current_position, next_position):
        required_distance = self.distance
        distance = self.get_euclidean_distance(current_position, next_position)
        return (distance <= required_distance)

    def generate_roadmap(self, sample_nodes, graph, obstacle_list, max_num_edges=5):
        samples = set(sample_nodes)
        if len(graph.keys()) != 0:
            existing_nodes = graph.keys()
        else:
            existing_nodes = samples
        
        for current_node in samples:
            if current_node not in graph.keys():
                child_nodes = []
                distance_map = []
                parent_node = current_node
                for sample in existing_nodes:
                    if sample!=current_node and self.within_distance(current_node, sample) and not self.is_edge_intersecting(sample, current_node, obstacle_list):
                        distance_map.append((self.get_euclidean_distance(sample, current_node),sample))
                distance_map = sorted(distance_map, key=lambda x : x[0])
                count = 0
                for pair in distance_map:
                    if count >= max_num_edges:
                        break
                    child_nodes.append(pair[1])
                    count+=1
                    
                #graph[parent_node] = child_nodes
                self.sample.update([parent_node])
                self.addGraph(graph, child_nodes, parent_node)
            else:
                pass
                #print('Already there')
        self.graph = graph
        roadmap = graph
        return roadmap



if __name__ == "__main__":    
    start = (-4.0, 4.0)
    goal = (4.0, 4.0)
    x_limit = (-5.0, 5.0)
    y_limit = (-5.0, 5.0)
    roadmap = PRMPlanner(100, 2, x_limit, y_limit)
    
    parent = (10, 11)
    child = (4, 4)
    
    sample_nodes = roadmap.random_position([])
    
    print(len(roadmap.generate_roadmap(sample_nodes, roadmap.graph, []).keys()))
    graph_copy = copy.deepcopy(roadmap.graph)
    print(len(roadmap.sample))
    
    key = list(roadmap.graph.keys())[0]
    roadmap.generate_roadmap([(3,3)], roadmap.graph, [], 6)
    
    val = roadmap.graph[(3,3)]
    for k,v in graph_copy.items():
        v_val = roadmap.graph[k]
        diff = len(v_val) - len(v)
        if diff != 0:
             print(diff)
             print((3,3) in v_val, v)
        
    print(roadmap.graph[(3,3)])