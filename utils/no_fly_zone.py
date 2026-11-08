import heapq
import math
from utils.problem_definition import calculate_haversine_distance

class Node:
    def __init__(self, node_id, circle_id, circular_idx, coordinates):
        self.node_id = node_id
        self.circle_id = circle_id
        self.circular_idx = circular_idx
        self.coordinates = coordinates
        self.neighbors = set()
        self.came_from = None
        self.valid = True 

    def add_neighbor(self, neighbor):
        self.neighbors.add(neighbor)

    def __lt__(self, other):
        return self.node_id < other.node_id

def generate_circle_graph(nodes_on_nfzs, num_points, start_node, goal_node):
    node_id = 0
    graph = {}
    all_circular_coordinates = {}
    
    # 1. Create nodes for all NFZ perimeters
    for _ in range(len(num_points)):
        circular_coordinates = nodes_on_nfzs[_]        
        all_circular_coordinates[_] = circular_coordinates
        circular_idx = 0
        for i in range(num_points[_]):
            node = Node(node_id, _, circular_idx, circular_coordinates[i])
            graph[node_id] = node
            node_id += 1
            circular_idx += 1
        
        # Connect perimeter neighbors (Ring topology)
        for i in range(num_points[_]):
            graph_idx = sum(num_points[:_]) + i
            node = graph[graph_idx]
            
            if graph_idx == sum(num_points[:_]): 
                prev_node = graph[node_id - 1]
            else: 
                prev_node = graph[graph_idx - 1]
                
            if graph_idx == node_id - 1: 
                next_node = graph[sum(num_points[:_])]
            else: 
                next_node = graph[graph_idx + 1]

            node.add_neighbor(prev_node)
            node.add_neighbor(next_node)

    # 2. Create Start and Goal Nodes
    s_id = len(num_points)
    g_id = len(num_points) + 1
    
    start_node_obj = Node(node_id, s_id, 0, start_node)
    graph[node_id] = start_node_obj
    
    goal_node_obj = Node(node_id + 1, g_id, 0, goal_node)
    graph[node_id + 1] = goal_node_obj
    
    return graph, all_circular_coordinates

def is_point_inside_polygon(point, polygon):
    """Ray-casting algorithm to check if a point is inside a polygon."""
    x, y = point[0], point[1]
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def construct_graph_with_connections(nodes_on_nfzs, num_points, start_node, goal_node):
    graph, all_circular_coordinates = generate_circle_graph(nodes_on_nfzs, num_points, start_node, goal_node)
    
    # Validation Step: Mark nodes invalid if they fall inside other NFZs
    for node in graph.values():
        if node.circle_id >= len(nodes_on_nfzs): continue # Skip Start/Goal
        point = node.coordinates
        for nfz_id, nfz_coords in all_circular_coordinates.items():
            if nfz_id == node.circle_id: continue 
            if is_point_inside_polygon(point, nfz_coords):
                node.valid = False
                break

    # Group nodes
    num_sets = num_points + [1, 1]
    all_circular_nodes = []
    current_idx = 0
    for count in num_sets:
        group = []
        for _ in range(count):
            group.append(graph[current_idx])
            current_idx += 1
        all_circular_nodes.append(group)
    
    # Connect nodes
    for circle_nodes in all_circular_nodes:
        other_nodes_groups = [g for g in all_circular_nodes if g != circle_nodes]
        other_nodes = [node for group in other_nodes_groups for node in group]
        
        for a_node_on_circle in circle_nodes:
            if not a_node_on_circle.valid: continue
            for other_node in other_nodes:
                if not other_node.valid: continue
                
                is_inter_sect = False
                for nfz_id, nfz_coords in all_circular_coordinates.items():
                    nfz_tuple = (nfz_id, nfz_coords)
                    if check_intersection(a_node_on_circle, other_node, nfz_tuple):
                        is_inter_sect = True
                        break
                
                if not is_inter_sect:
                    a_node_on_circle.add_neighbor(other_node)
                    
    return graph

def a_star(graph):
    start_id = len(graph) - 2
    goal_id = len(graph) - 1
    
    if start_id not in graph or goal_id not in graph: return None
        
    start_node = graph[start_id]
    goal_node = graph[goal_id]
    
    open_list = [(0, start_node)]
    closed_list = set()
    
    g_score = {node.node_id: float('inf') for node in graph.values()}
    g_score[start_node.node_id] = 0
    
    f_score = {node.node_id: float('inf') for node in graph.values()}
    f_score[start_node.node_id] = calculate_haversine_distance(start_node.coordinates, goal_node.coordinates)

    while open_list:
        current_node = heapq.heappop(open_list)[1]
        if current_node == goal_node:
            path = []
            while current_node:
                path.append([current_node.node_id, current_node.coordinates])
                current_node = current_node.came_from
            return path[::-1]

        closed_list.add(current_node)

        for neighbor in current_node.neighbors:
            if not neighbor.valid: continue
            if neighbor in closed_list: continue
                
            tentative_g_score = g_score[current_node.node_id] + calculate_haversine_distance(current_node.coordinates, neighbor.coordinates)
            
            if tentative_g_score < g_score[neighbor.node_id]:
                neighbor.came_from = current_node
                g_score[neighbor.node_id] = tentative_g_score
                f_score[neighbor.node_id] = tentative_g_score + calculate_haversine_distance(neighbor.coordinates, goal_node.coordinates)
                heapq.heappush(open_list, (f_score[neighbor.node_id], neighbor))

    return None

def check_intersection(node_1, node_2, circular_points):
    """
    Checks intersection. Handles both list (legacy) and tuple (with ID) inputs.
    """
    # Robust check: Determine if input is (id, coords) or just coords
    circle_id = -1
    circular_coordinates = circular_points
    
    if isinstance(circular_points, tuple):
        # Only treat as (id, coords) if the first element is an int (ID)
        # This prevents confusion if a tuple of coordinates ((lat,lon),...) is passed
        if len(circular_points) >= 2 and isinstance(circular_points[0], int):
            circle_id = circular_points[0]
            circular_coordinates = circular_points[1]
    
    point1 = node_1.coordinates if isinstance(node_1, Node) else node_1
    point2 = node_2.coordinates if isinstance(node_2, Node) else node_2
    
    node1_on_this_nfz = isinstance(node_1, Node) and node_1.circle_id == circle_id
    node2_on_this_nfz = isinstance(node_2, Node) and node_2.circle_id == circle_id
    
    for i in range(len(circular_coordinates)):
        p1 = circular_coordinates[i]
        p2 = circular_coordinates[(i + 1) % len(circular_coordinates)]
        
        # Optimization: Skip adjacent edges only if we know the ID (graph construction)
        if circle_id != -1:
            if node1_on_this_nfz:
                if i == node_1.circular_idx or i == (node_1.circular_idx - 1) % len(circular_coordinates):
                    continue
            if node2_on_this_nfz:
                if i == node_2.circular_idx or i == (node_2.circular_idx - 1) % len(circular_coordinates):
                    continue

        if do_intersect(point1, point2, p1, p2):
            return True
            
    return False

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0: return 0
    return 1 if val > 0 else 2

def do_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4: return True
    if o1 == 0 and on_segment(p1, p2, q1): return True
    if o2 == 0 and on_segment(p1, q2, q1): return True
    if o3 == 0 and on_segment(p2, p1, q2): return True
    if o4 == 0 and on_segment(p2, q1, q2): return True
    return False

def on_segment(p, q, r):
    if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
        q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
        return True
    return False

def nfz_wp_altitude(poi_i, poi_j, wp):
    di = calculate_haversine_distance(poi_i[:2], wp)
    dj = calculate_haversine_distance(poi_j[:2], wp)
    d = dj + di
    if d == 0: return poi_i[2]
    return dj/d * poi_i[2] + di/d * poi_j[2]

def nfz_avoided_path(nodes_on_nfzs, poi_i, poi_j):
    # Construct graph with Start and Goal
    graph = construct_graph_with_connections(
        [nds[:-1] for nds in nodes_on_nfzs], 
        [len(_) - 1 for _ in nodes_on_nfzs], 
        poi_i[:2], 
        poi_j[:2]
    )
    path = a_star(graph)
    if path:
        return [[p[1][0], p[1][1], nfz_wp_altitude(poi_i, poi_j, p[1])] for p in path[1:-1]]
    else:
        return []