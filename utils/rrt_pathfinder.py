import random
import math
import numpy as np

# We will re-use your helper functions from no_fly_zone.py
# This assumes rrt_pathfinder.py is in the same directory as no_fly_zone.py
# If not, you may need to adjust the import
from utils.no_fly_zone import check_intersection, nfz_wp_altitude, calculate_haversine_distance

class RRTNode:
    """A node in the RRT tree."""
    def __init__(self, p):
        self.p = p  # (lat, lon) coordinate tuple
        self.parent = None
        self.cost = 0.0  # Cost (distance) from the start node

class RRT_Pathfinder:
    """
    RRT* (R-star) pathfinding class.
    Works in 2D (lat, lon) space.
    """
    def __init__(self, p_start, p_goal, nfzs, all_addresses, max_iter=1500, goal_sample_rate=0.1, step_size=0.01):
        self.p_start = RRTNode(p_start[:2])  # Start node
        self.p_goal = RRTNode(p_goal[:2])    # Goal node
        self.nfzs = nfzs                     # List of NFZ polygons
        
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.step_size = step_size  # This may need tuning (in degrees lat/lon)

        self.node_list = [self.p_start]
        
        # --- Calculate Bounding Box ---
        all_lats = [addr[0] for addr in all_addresses]
        all_lons = [addr[1] for addr in all_addresses]
        for nfz in nfzs:
            all_lats.extend([p[0] for p in nfz[:-1]])
            all_lons.extend([p[1] for p in nfz[:-1]])
        
        self.min_lat = min(all_lats) - 0.01
        self.max_lat = max(all_lats) + 0.01
        self.min_lon = min(all_lons) - 0.01
        self.max_lon = max(all_lons) + 0.01
        self.search_radius = self.step_size * 5  # "Near" radius for RRT*

    def solve(self):
        """Run the RRT* algorithm."""
        for i in range(self.max_iter):
            # 1. Get a random sample point
            p_rand = self.get_random_point()
            
            # 2. Find nearest node in the tree
            n_near = self.get_nearest_node(p_rand)
            
            # 3. Steer from nearest node to random point
            p_new = self.steer(n_near, p_rand)
            
            # 4. If the path is collision-free
            if self.is_collision_free(n_near.p, p_new.p):
                # 5. Find nearby nodes (for RRT*)
                near_nodes = self.get_near_nodes(p_new)
                
                # 6. Find the best parent (minimum cost)
                n_min = self.choose_best_parent(p_new, near_nodes, n_near)
                
                # 7. Add new node to the tree
                if n_min:
                    p_new.parent = n_min
                    p_new.cost = n_min.cost + calculate_haversine_distance(n_min.p, p_new.p)
                    self.node_list.append(p_new)
                else:
                    continue # No valid parent
                
                # 8. Rewire nearby nodes (RRT*)
                self.rewire(p_new, near_nodes)
            
            # 9. Check for goal
            if self.is_near_goal(p_new):
                if self.is_collision_free(p_new.p, self.p_goal.p):
                    # We found a path!
                    final_node = RRTNode(self.p_goal.p)
                    final_node.parent = p_new
                    final_node.cost = p_new.cost + calculate_haversine_distance(p_new.p, self.p_goal.p)
                    self.node_list.append(final_node) # Add goal to list
                    
                    # --- ✅ GET AND PRUNE THE PATH ---
                    raw_path = self.get_final_path(final_node)
                    #pruned_path = self.prune_path(raw_path)
                    pruned_path = raw_path
                    return pruned_path
                    # ---

        return None # No path found

    def get_random_point(self):
        """Get a random sample point (lat, lon)."""
        if random.random() < self.goal_sample_rate:
            return self.p_goal.p
        
        lat = random.uniform(self.min_lat, self.max_lat)
        lon = random.uniform(self.min_lon, self.max_lon)
        return (lat, lon)

    def get_nearest_node(self, p_rand):
        """Find the nearest node in self.node_list to the random point."""
        dlist = [(node.p[0] - p_rand[0])**2 + (node.p[1] - p_rand[1])**2 for node in self.node_list]
        min_index = dlist.index(min(dlist))
        return self.node_list[min_index]

    def steer(self, from_node, to_p):
        """Create a new node 'step_size' away from from_node, in the direction of to_p."""
        
        # Use simple linear interpolation (approximation)
        d_lat = to_p[0] - from_node.p[0]
        d_lon = to_p[1] - from_node.p[1]
        dist_approx = math.sqrt(d_lat**2 + d_lon**2)
        
        # Safety check for divide by zero
        if dist_approx == 0:
            return RRTNode(from_node.p)
        
        if dist_approx < self.step_size:
            return RRTNode(to_p)
        
        scale = self.step_size / dist_approx
        new_lat = from_node.p[0] + d_lat * scale
        new_lon = from_node.p[1] + d_lon * scale
        
        return RRTNode((new_lat, new_lon))

    def is_collision_free(self, p1, p2):
        """Check if the line from p1 to p2 intersects any NFZ."""
        for nfz_nodes in self.nfzs:
            # Pass only the 2D polygon part
            if check_intersection(p1, p2, nfz_nodes[:-1]):
                return False  # Collision!
        return True  # No collisions

    def get_near_nodes(self, p_new):
        """Find nodes in the tree within self.search_radius of the new node."""
        near_nodes = []
        for node in self.node_list:
            dist = calculate_haversine_distance(node.p, p_new.p)
            if dist < self.search_radius:
                near_nodes.append(node)
        return near_nodes

    def choose_best_parent(self, p_new, near_nodes, n_near):
        """From the nearby nodes, find the one that gives the shortest path to p_new."""
        n_min = n_near
        min_cost = n_near.cost + calculate_haversine_distance(n_near.p, p_new.p)
        
        for node in near_nodes:
            new_cost = node.cost + calculate_haversine_distance(node.p, p_new.p)
            if new_cost < min_cost:
                if self.is_collision_free(node.p, p_new.p):
                    min_cost = new_cost
                    n_min = node
        
        # Final check on the best parent
        if not self.is_collision_free(n_min.p, p_new.p):
             return None
             
        p_new.cost = min_cost
        return n_min

    def rewire(self, p_new, near_nodes):
        """RRT* Rewire: Check if any near nodes can find a shorter path by parenting to p_new."""
        for node in near_nodes:
            if node == p_new.parent:
                continue
                
            new_cost = p_new.cost + calculate_haversine_distance(p_new.p, node.p)
            
            if new_cost < node.cost:
                if self.is_collision_free(p_new.p, node.p):
                    node.parent = p_new
                    node.cost = new_cost
                    # Update costs of children (optional but good)
                    self.propagate_cost_to_children(node)

    def propagate_cost_to_children(self, parent_node):
        """Recursively update the cost of all children of a rewired node."""
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = parent_node.cost + calculate_haversine_distance(parent_node.p, node.p)
                self.propagate_cost_to_children(node)

    def is_near_goal(self, node):
        """Check if a node is within one step_size of the goal."""
        dist = calculate_haversine_distance(node.p, self.p_goal.p)
        return dist < self.step_size

    def get_final_path(self, goal_node):
        """Backtrack from the goal to the start to get the path."""
        path = [goal_node.p]
        node = goal_node
        while node.parent:
            path.append(node.parent.p)
            node = node.parent
        path.reverse()
        return path

    # --- ✅ NEW FUNCTION: PATH PRUNING ---
    def prune_path(self, path):
        """
        Prunes (smooths) a wiggly RRT path.
        Iteratively connects nodes with a straight line if collision-free.
        """
        if not path or len(path) < 2:
            return path
            
        pruned_path = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            farthest_idx = current_idx + 1
            # Look ahead to find the farthest node we can connect to
            for look_ahead_idx in range(len(path) - 1, current_idx, -1):
                if self.is_collision_free(path[current_idx], path[look_ahead_idx]):
                    farthest_idx = look_ahead_idx
                    break
            
            pruned_path.append(path[farthest_idx])
            current_idx = farthest_idx
            
        return pruned_path


def nfz_avoided_path_rrt(nodes_on_nfzs, poi_i, poi_j, all_addresses):
    """
    Computes a path from poi_i to poi_j avoiding NFZs using RRT*.
    
    Args:
        nodes_on_nfzs (list): List of NFZ nodes with altitude.
        poi_i (tuple): Start PoI (lat, lon, alt).
        poi_j (tuple): Goal PoI (lat, lon, alt).
        all_addresses (list): All addresses, for bounding box.

    Returns:
        list: List of (lat, lon, alt) coordinates along the path.
    """
    # 1. Run RRT* in 2D
    # We use a high iteration count for a better path. This can be tuned.
    rrt_star = RRT_Pathfinder(poi_i, poi_j, nodes_on_nfzs, all_addresses, max_iter=2000, step_size=0.005)
    path_2d = rrt_star.solve()
    
    if not path_2d:
        # RRT failed to find a path, return a simple (but colliding) path
        # In a real app, you'd raise an error.
        print(f"RRT* FAILED to find path from {poi_i[:2]} to {poi_j[:2]}")
        return []
    
    # 2. Interpolate altitude (same as A* method)
    path_3d = []
    for wp_2d in path_2d:
        alt = nfz_wp_altitude(poi_i, poi_j, wp_2d)
        path_3d.append([wp_2d[0], wp_2d[1], alt])
        
    # Return path *without* start and end, just like original nfz_avoided_path
    return path_3d[1:-1]