import yaml
import math
from pathlib import Path

def interpolate_position(p1, p2, t1, t2, t_target):
    """
    Linearly interpolates 3D position between p1 and p2 at time t_target.
    p1, p2: tuples (lat, lon, alt)
    """
    if t2 == t1: return p1
    alpha = (t_target - t1) / (t2 - t1)
    # Clamp alpha to [0, 1] to stay within segment
    alpha = max(0.0, min(1.0, alpha))
    
    lat = p1[0] + alpha * (p2[0] - p1[0])
    lon = p1[1] + alpha * (p2[1] - p1[1])
    alt = p1[2] + alpha * (p2[2] - p1[2])
    return (lat, lon, alt)

def get_node_coords(node_idx, uas_data, mission_data, num_starts, num_ends):
    """
    Resolves node index to (lat, lon, alt) based on specific mapping:
    0..S-1 : Start Depots (uas_data['initial_...'])
    S..S+E-1 : End Depots (uas_data['final_...'])
    S+E..end : PoIs (mission_data['poi_...'])
    """
    # Mapping based on user description: 
    # Indexes 0-4 (Start), 5-9 (End), 10-39 (PoIs)
    # So PoIs start after Starts + Ends.
    
    s_idx_start = 0
    e_idx_start = num_starts
    p_idx_start = num_starts + num_ends
    
    # Check if node is Start Depot
    if s_idx_start <= node_idx < s_idx_start + num_starts:
        local_idx = node_idx
        return (uas_data['initial_lat'][local_idx], 
                uas_data['initial_long'][local_idx], 
                uas_data['initial_alt'][local_idx])
    
    # Check if node is End Depot
    elif e_idx_start <= node_idx < e_idx_start + num_ends:
        local_idx = node_idx - e_idx_start
        return (uas_data['final_lat'][local_idx], 
                uas_data['final_long'][local_idx], 
                uas_data['final_alt'][local_idx])
    
    # Check if node is PoI
    else:
        local_idx = node_idx - p_idx_start
        # Safety check
        if local_idx < len(mission_data['poi_lat']):
            return (mission_data['poi_lat'][local_idx], 
                    mission_data['poi_long'][local_idx], 
                    mission_data['poi_alt'][local_idx])
        else:
            return (0,0,0) # Error or virtual node

def perform_replanning_update(arrival_times, routes, problem_data, replan_time, uas_to_remove_idx, config_dir):
    """
    Updates uas.yaml and mission.yaml based on replanning state.
    """
    
    # 1. Load YAML files
    uas_path = config_dir / 'uas.yaml'
    mission_path = config_dir / 'mission.yaml'
    
    with open(uas_path, 'r') as f:
        uas_yaml = yaml.safe_load(f)
    with open(mission_path, 'r') as f:
        mission_yaml = yaml.safe_load(f)

    num_starts = uas_yaml['start_depot_num']
    num_ends = uas_yaml['end_depot_num']
    
    # 2. Identify Visited PoIs
    # Mapping: PoI index k in yaml corresponds to Solver Node (num_starts + num_ends + k)
    # NOTE: User log showed PoIs at 10-39. Starts 0-4, Ends 5-9.
    # Logic: Start=5, End=5.
    offset = num_starts + num_ends
    
    visited_poi_indices = [] # Indices relative to mission.yaml lists
    
    for i in range(len(mission_yaml['poi_lat'])):
        solver_node_idx = offset + i
        # If node was visited BEFORE or AT replan_time, mark as visited
        if solver_node_idx in arrival_times:
            if arrival_times[solver_node_idx] <= replan_time:
                visited_poi_indices.append(i)
                
    # 3. Calculate New Start Positions for Remaining UAS
    # We iterate over current routes to find where each UAS is at replan_time
    new_initial_positions = {} # {original_uas_idx: (lat, lon, alt)}
    
    for v_id, route_info in routes.items():
        if v_id == uas_to_remove_idx:
            continue
            
        # Route nodes sequence: e.g. [0, 18, 22, ..., 5]
        nodes = route_info['nodes']
        
        # Find segment
        current_pos = None
        
        # Default: if not started, it's at start node
        start_node_id = nodes[0]
        current_pos = get_node_coords(start_node_id, uas_yaml, mission_yaml, num_starts, num_ends)
        
        for k in range(len(nodes) - 1):
            u = nodes[k]
            v = nodes[k+1]
            
            t_u = arrival_times.get(u, 0)
            t_v = arrival_times.get(v, 0)
            
            # Check if drone is on this leg
            if t_u <= replan_time <= t_v:
                p_u = get_node_coords(u, uas_yaml, mission_yaml, num_starts, num_ends)
                p_v = get_node_coords(v, uas_yaml, mission_yaml, num_starts, num_ends)
                current_pos = interpolate_position(p_u, p_v, t_u, t_v, replan_time)
                break
            
            # Check if drone already finished this leg and is past it
            if replan_time > t_v and k == len(nodes) - 2:
                # Finished mission, sitting at end node
                current_pos = get_node_coords(v, uas_yaml, mission_yaml, num_starts, num_ends)

        new_initial_positions[v_id] = current_pos

    
     # --- PRINT NEW POSITIONS (REQUESTED FEATURE) ---
    print("\n" + "="*50)
    print(f"REPLANNING STATE AT T={replan_time}s")
    print("="*50)
    print(f"UAS {uas_to_remove_idx} Removed (Failure/Emergency)")
    print("New Start Coordinates for Remaining Fleet:")
    for v_id, pos in new_initial_positions.items():
        if pos:
            print(f"  -> UAS {v_id}: Lat: {pos[0]:.6f}, Lon: {pos[1]:.6f}, Alt: {pos[2]:.2f}")
        else:
            print(f"  -> UAS {v_id}: Position Unknown (Inactive)")
    print("-" * 50)
    print(f"Visited PoIs count: {len(visited_poi_indices)}")
    print("="*50 + "\n")
    
    # 4. Update UAS YAML Data
    # Remove index 'uas_to_remove_idx' from all lists
    # Update 'initial' lists with new positions for remaining indices
    
    keys_to_filter = ['autonomy',  
                      'fov_x', 'fov_y', 'initial_alt', 'initial_lat', 'initial_long',
                      'max_alt', 'max_ias', 'max_rol', 'rlos', 'uas_gs']
    
    for key in keys_to_filter:
        if key in uas_yaml and isinstance(uas_yaml[key], list):
            # Create new list excluding the removed UAS
            old_list = uas_yaml[key]
            new_list = [x for i, x in enumerate(old_list) if i != uas_to_remove_idx]
            
            # If this is an 'initial' key, we must update values for the remaining UAS
            if key.startswith('initial_'):
                # new_list now has length (N-1). Indices 0..(N-2).
                # We map back to original indices to get correct position
                # Remaining original indices: [0, 1, ..., remove-1, remove+1, ...]
                
                mapping_idx = 0
                for orig_idx in range(len(old_list)):
                    if orig_idx == uas_to_remove_idx: continue
                    
                    if orig_idx in new_initial_positions:
                        pos = new_initial_positions[orig_idx]
                        if key == 'initial_lat': new_list[mapping_idx] = pos[0]
                        if key == 'initial_long': new_list[mapping_idx] = pos[1]
                        if key == 'initial_alt': new_list[mapping_idx] = pos[2]
                    
                    mapping_idx += 1
            
            uas_yaml[key] = new_list

    # Update counts
    uas_yaml['uas_num'] = uas_yaml['uas_num'] - 1
    uas_yaml['start_depot_num'] = uas_yaml['uas_num']
    #uas_yaml['end_depot_num'] = uas_yaml['uas_num']
    
    # 5. Update Mission YAML Data (Remove Visited PoIs)
    poi_keys = ['poi_lat', 'poi_long', 'poi_alt', 'poi_toa']
    
    for key in poi_keys:
        if key in mission_yaml and isinstance(mission_yaml[key], list):
            old_list = mission_yaml[key]
            # Keep only indices NOT in visited_poi_indices
            new_list = [x for i, x in enumerate(old_list) if i not in visited_poi_indices]
            mission_yaml[key] = new_list

    # 6. Save Updated Files
    # (Here we overwrite as requested, or you can save to _replan.yaml)
    with open(uas_path, 'w') as f:
        yaml.dump(uas_yaml, f)
    
    with open(mission_path, 'w') as f:
        yaml.dump(mission_yaml, f)
        
    return True