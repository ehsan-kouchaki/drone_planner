from flask import Blueprint, current_app, request, jsonify
from flask_babel import gettext
from utils.load_parameters import load_solver_configuration
from utils.auxiliary import *
from utils.or_tools_method import *
from utils.problem_definition import generate_distance_matrix, generate_flight_distance_matrix, calculate_haversine_distance
from utils.no_fly_zone import *
from app.routes.vertice_smoother import *
from utils.rrt_pathfinder import nfz_avoided_path_rrt
from utils.replanning import perform_replanning_update
from pathlib import Path
import math
import time  # Added for explicit timing


# Define a Flask blueprint for handling routes-related routes
routes_blueprint = Blueprint("routes_blueprint", __name__)


@routes_blueprint.route("/create-routes", methods=["POST"])
@measure_execution_time
def generate_route():

    """
    Generate a route based on the provided form data and the
    problem and solver data stored in the application context.
    This includes generating a distance matrix if it doesn't exist,
    finding routes using OR-Tools, and generating a solution.

    Returns:
        JSON object: A JSON object containing the success status,
            a message, the generated routes, and the problem data.
    """
    
    # Toggle between 'A_STAR' and 'RRT' to compare
    PATHFINDER_TO_USE = 'A_STAR'
    # PATHFINDER_TO_USE = 'RRT'

    if request.method == "POST":
        
        if not current_app.solver_data:
            current_app.solver_data = load_solver_configuration()
        
        # Distance matrix creation
        if "distance_matrix" not in current_app.problem_data.keys():
            if current_app.problem_data["travel_mode"] == "flight":
                current_app.problem_data[
                    "distance_matrix"
                ] = generate_flight_distance_matrix(
                    current_app.problem_data["addresses"]
                )
            else:
                current_app.problem_data["distance_matrix"] = generate_distance_matrix(
                    current_app.problem_data["addresses"],
                    current_app.config["API_KEY"],
                    current_app.config["GEOCODE_API_URL"],
                    current_app.config["DISTANCE_MATRIX_API_URL"],
                    mode=current_app.problem_data["travel_mode"],
                )
        
        
        # Internal no-fly zones specifications
        '''
        In this part, all of the peripheral points of all NFZs are stored in a list of lists:
        nodes_on_nfzs = [[nodes_on_nfz_1], [nodes_on_nfz_2], ... [nodes_on_nfz_n]]
        Each internal list is a list of tuples and each tuple is a (lat, long) pair of each
        peripheral point. Except the final tuple which is (min_alt_of_nfz, max_alt_of_nfz)
        pair:
        nodes_on_nfz_i = [(alt_1, long_1), ... (alt_m, long_m), (min_alt_of_nfz, max_alt_of_nfz)]
        '''
        nodes_on_nfzs = []
        nodes_on_nfzs_layer1 = []
        if current_app.problem_data['nfz_int_circ_lat']:
            num_nfz_c = len(current_app.problem_data['nfz_int_circ_lat'])
            nfz_center = [[current_app.problem_data['nfz_int_circ_lat'][_], current_app.problem_data['nfz_int_circ_long'][_]] for _ in range(num_nfz_c)]
            nfz_alt_min = current_app.problem_data['nfz_int_circ_min_alt']
            nfz_alt_max = current_app.problem_data['nfz_int_circ_max_alt']
            for _ in range(num_nfz_c):
                # A safety margin equal to 10 percent of the radius is chosen for the circular NFZs
                nfz_radius = current_app.problem_data['nfz_safety_margin'] * current_app.problem_data['nfz_int_circ_r'][_]
            
                # The number of peripheral points on each circle is defined based on the minimum allowed distance between WPs.
                theta = current_app.problem_data['min_dist_wp'] / nfz_radius
                nfz_num_points = int(2 * pi / theta)
                nfz_num_points_layer1 = 16
                nodes = circle_peripheral_nodes(nfz_center[_], nfz_radius, nfz_num_points)
                nodes_layer1 = circle_peripheral_nodes(nfz_center[_], nfz_radius, nfz_num_points_layer1)
                nodes.append((nfz_alt_min[_], nfz_alt_max[_]))
                nodes_layer1.append((nfz_alt_min[_], nfz_alt_max[_]))
                nodes_on_nfzs.append(nodes) 
                nodes_on_nfzs_layer1.append(nodes_layer1)
                
        if current_app.problem_data['nfz_int_poly_lat']:
            num_nfz_p = len(current_app.problem_data['nfz_int_poly_lat'])    
            for _ in range(num_nfz_p):
                poly_coord_orig = [(current_app.problem_data['nfz_int_poly_lat'][_][__], current_app.problem_data['nfz_int_poly_long'][_][__])
                                    for __ in range(len(current_app.problem_data['nfz_int_poly_lat'][_]))]
                # nfz with safety margin
                poly_coord = scale_polygon(poly_coord_orig)
                nfz_alt_min = current_app.problem_data['nfz_int_poly_min_alt'][_]
                nfz_alt_max = current_app.problem_data['nfz_int_poly_max_alt'][_]
                poly_coord.append((nfz_alt_min, nfz_alt_max))
                nodes_on_nfzs.append(poly_coord)
                nodes_on_nfzs_layer1.append(poly_coord)

        # Modify the distance matrix for no-fly zones in 'visit' problems
        '''
        This part navigates on the addresses. For each pair of addresses,
        if their connections cross at least one NFZ, a new path avoiding
        the NFZ is computed using A*, and the length of the new path
        substitutes the corresponding distance matrix element.
        '''
        if current_app.problem_data['mission_type'] == 'visit':
            addresses = current_app.problem_data["addresses"]
            print(f"--- Running Pathfinding using {PATHFINDER_TO_USE} ---")

            for i in range(len(addresses)):
                for j in range(i + 1, len(addresses)):
                    effective_nodes_on_nfzs = []
                    for nodes_on_nfz in nodes_on_nfzs_layer1:
                        alt_min_max = nodes_on_nfz[-1]
                        is_intersect  = check_intersection(addresses[i][:2], addresses[j][:2], nodes_on_nfz[:-1])
                        if is_intersect and nfz_intersect_vertically(alt_min_max, addresses[i], addresses[j]):
                            effective_nodes_on_nfzs.append(nodes_on_nfz)

                    if effective_nodes_on_nfzs:

                        # --- A* or RRT* is chosen here ---
                        if PATHFINDER_TO_USE == 'A_STAR':
                            new_path_coords = nfz_avoided_path(effective_nodes_on_nfzs, addresses[i], addresses[j])
                        else: # 'RRT'
                            # RRT* needs all addresses for its bounding box
                            new_path_coords = nfz_avoided_path_rrt(effective_nodes_on_nfzs, addresses[i], addresses[j], addresses)
                        
                        # --- Recalculate distance ---
                        new_path = [addresses[i]] + new_path_coords + [addresses[j]]
                        new_distance = 0
                        for k in range(len(new_path) - 1):
                            new_distance += calculate_haversine_distance(new_path[k], new_path[k + 1])
                        
                        current_app.problem_data["distance_matrix"][i][j] = new_distance
                        current_app.problem_data["distance_matrix"][j][i] = new_distance 
        
        
        #current_app.problem_data["distance_matrix"][23][16] *= 1.1
       
        # Converts 'poi_toa' list (YAML) -> 'poi_max_arrival_time' dict (OR-Tools)
        
        if "poi_toa" in current_app.problem_data and current_app.problem_data["poi_toa"]:
            # Start nodes come first in the index list.
            # Assuming address order: [Starts, PoIs, Ends]
            num_starts = len(current_app.problem_data.get("start_nodes", []))
            num_ends = len(current_app.problem_data.get("end_nodes", []))
            
            poi_deadline_dict = {}
            # Map each PoI in the list to its corresponding Node Index
            for i, deadline in enumerate(current_app.problem_data["poi_toa"]):
                node_idx = num_starts + num_ends + i
                poi_deadline_dict[node_idx] = deadline #+ 4000
                
            current_app.problem_data["poi_max_arrival_time"] = poi_deadline_dict

            #print("poi_deadline_dict:\n", poi_deadline_dict)
        
        
        # --- Measure Layer 1 Time ---
        t0_layer1 = time.time()
        data, manager, routing, solution = find_routes(
            current_app.problem_data, current_app.solver_data
        )        
        ortools_routes = adapt_solution(
            data, manager, routing, solution, current_app.problem_data["addresses"]
        )
        current_app.routes = ortools_routes
        
        
        '''
        layer1_energy_map = {}
        arrival_times_map = {}
        if solution:
            # Arrival Times
            arrival_times_map = extract_arrival_times(
                manager, routing, solution, current_app.problem_data["uas_num"]
            )
            layer1_arrival_times = {k: v for d in arrival_times_map for k, v in d.items()}

            print("Arrival Times (Node Index -> Seconds):")
            for ti in arrival_times_map:
                print(ti)
            # Energy
            layer1_energy_map = extract_final_energy_consumption(
                manager, routing, solution, current_app.problem_data["uas_num"], current_app.problem_data
            )
        '''
        
        t1_layer1 = time.time()
        layer1_computation_time = t1_layer1 - t0_layer1

        # =============================================================================
        # Print Layer 1 Statistics (Before Smoothing) ###
        # =============================================================================
        print("\n--- Layer 1 (VRP) Statistics ---")
        print(f"Computation Time: {layer1_computation_time:.4f} s")
        
        for vehicle_id, route_data in current_app.routes['routes'].items():
            # We calculate the distance manually from coordinates to ensure it matches 
            # the method used for Layer 2 (Haversine sum of segments) exactly.
            layer1_coords = route_data['coordinates']
            #print('layer1_coords:\n', layer1_coords)
            l1_dist_meters = 0
            for k in range(len(layer1_coords) - 1):
                if layer1_coords[k] in [sublist for sublist in data['addresses'][:len(data["start_nodes"]) + len(data["end_nodes"])]
                                       ] and layer1_coords[k+1] in [sublist for sublist in data['addresses'][:len(data["start_nodes"]) + len(data["end_nodes"])]]:
                    break
                l1_dist_meters += calculate_haversine_distance(layer1_coords[k], layer1_coords[k+1])
            
            print(f"Vehicle {vehicle_id}:\n            Distance: {l1_dist_meters:.2f} meters")
            velocity = data["uas_gs"][vehicle_id] * 0.2777777778
            route_time = l1_dist_meters / velocity
            route_mins = route_time // 60
            route_secs = route_time % 60
            print(f"            Time to complete the route: {route_mins}min {route_secs:.0f}secs")

            # Print Energy if available
            '''
            if vehicle_id in layer1_energy_map:
                consumed = layer1_energy_map[vehicle_id]
                print(f"            Energy Consumed: {consumed} J")
                if "battery_capacity_joules" in current_app.problem_data:
                    total_cap = current_app.problem_data["battery_capacity_joules"][vehicle_id]
                    rem_pct = ((total_cap - consumed) / total_cap) * 100
                    print(f"            Remaining Battery: {rem_pct:.1f}%")
            '''
            
        print("-" * 40 + "\n")
        # =============================================================================

        # ---  A* / RRT* for visualization ---
        
        if nodes_on_nfzs:
            for i in range(len(current_app.routes['routes'])):
                added_nodes = {}
                coord = current_app.routes['routes'][i]['coordinates']
                for j in range(len(coord) - 1):
                    effective_nodes_on_nfzs = []
                    for nodes_on_nfz in nodes_on_nfzs:
                        alt_min_max = nodes_on_nfz[-1]
                        is_intersect  = check_intersection(coord[j][:2], coord[j + 1][:2], nodes_on_nfz[:-1])
                        if is_intersect and nfz_intersect_vertically(alt_min_max, coord[j], coord[j + 1]):
                            effective_nodes_on_nfzs.append(nodes_on_nfz)
                    
                    if effective_nodes_on_nfzs:
                        # --- A* or RRT* is chosen here ---
                        if PATHFINDER_TO_USE == 'A_STAR':
                            added_nodes[j] = nfz_avoided_path(effective_nodes_on_nfzs, coord[j], coord[j + 1])
                        else: # 'RRT'
                            added_nodes[j] = nfz_avoided_path_rrt(effective_nodes_on_nfzs, coord[j], coord[j + 1], current_app.problem_data["addresses"])

                for key, value in reversed(added_nodes.items()):
                    coord[key + 1 : key + 1] = value
        

        smoothing = True
        # Add this flag to control the mode
        USE_OVERFLY_SMOOTHING = False  # Set to False to use your original Circle A* method

        if smoothing:
            if USE_OVERFLY_SMOOTHING:
                print("\n--- Applying Overfly (S-Turn) Smoothing ---")
                for i in range(len(current_app.routes['routes'])):
                    phi_max = current_app.problem_data["max_rol"][i]
                    V_G = current_app.problem_data["uas_gs"][i] / 3.6
                    # Kinematic radius in Meters
                    R_min = V_G ** 2 / (9.81 * math.tan(math.radians(phi_max)))
                    
                    coord = current_app.routes['routes'][i]['coordinates']
                    new_coord_list = []
                    
                    # Add start point
                    new_coord_list.append(coord[0])
                    
                    if len(coord) > 2:
                        # Iterate through vertices (excluding start/end depots)
                        for j in range(1, len(coord) - 1):
                            p_prev = coord[j-1]
                            p_curr = coord[j]
                            p_next = coord[j+1]

                            if p_curr not in [sublist for sublist in data['addresses']]:
                                new_coord_list.append(coord[j+1])
                                continue
                            if p_curr in [sublist for sublist in data['addresses'][:len(data["start_nodes"]) + len(data["end_nodes"])]]:
                                new_coord_list.append(coord[j])
                                continue
                            
                            # Pass the converted radius (R_min_deg) to the geometric function
                            overfly_segment = generate_overfly_path(p_prev, p_curr, p_next, R_min)
                            
                            # Extend list
                            new_coord_list.extend(overfly_segment)
                                                                        
                        # Add the final straight leg to the end depot
                        new_coord_list.append(coord[-1])
                        
                        # Update the route coordinates
                        current_app.routes['routes'][i]['coordinates'] = new_coord_list

            else:
                # Route smoothing by difining NO-FLY zone in corners and path finding using A*
                current_app.problem_data["art_nfz_lat"] = []
                current_app.problem_data["art_nfz_lon"] = []
                current_app.problem_data["art_nfz_r"] = []
                for i in range(len(current_app.routes['routes'])):
                    phi_max = current_app.problem_data["max_rol"][i]
                    V_G = current_app.problem_data["uas_gs"][i] / 3.6
                    R_min = V_G ** 2 / (9.81 * math.tan(math.radians(phi_max)))
                    added_nodes = {}
                    coord = current_app.routes['routes'][i]['coordinates']
                    smoothing_nodes = []
                    if len(coord) > 2:                                          
                        for j in range(1, len(coord) - 1):
                            point1 = coord[j - 1]
                            point2 = coord[j]
                            point3 = coord[j + 1]
                            
                            if point1 not in [sublist for sublist in data['addresses']
                                            ] and point2 not in [sublist for sublist in data['addresses']
                                                                                                 ]  and point3 not in [sublist for sublist in data['addresses']]:
                                continue
        
                            if len(smoothing_nodes) > 1:
                                smoothing_nodes.pop(0)
                            
                            if point2 not in [sublist for sublist in data['addresses'][:len(data["start_nodes"]) + len(data["end_nodes"])]]:
                                this_center, nodes = get_bisector_circle_robust(point1[:2], point2[:2], point3[:2], R_min)
                                nodes.append((-1, -1))
                                smoothing_nodes.append(nodes)

                                if point2 in [sublist for sublist in data['addresses']]:
                                    current_app.problem_data["art_nfz_lat"].append(this_center[0])
                                    current_app.problem_data["art_nfz_lon"].append(this_center[1])
                                    current_app.problem_data["art_nfz_r"].append(R_min-1)
                            
                            added_nodes[j - 1] = nfz_avoided_path(smoothing_nodes, point1, point2)
                            if j == len(coord) - 2:
                                added_nodes[j] = nfz_avoided_path(smoothing_nodes, point2, point3)
                                
                    for key, value in reversed(added_nodes.items()):
                        coord[key + 1 : key + 1] = value
    
    # =============================================================================
    # ### NEW CODE BLOCK: Calculate and Print Smoothed Statistics (Layer 2) ###
    # =============================================================================
    if smoothing:
        print("\n--- Final Smoothed Trajectory Statistics (Layer 2) ---")
        
        # To track the makespan after smoothing
        max_smoothed_time = 0 

        for vehicle_id, route_data in current_app.routes['routes'].items():
            # 1. Get the final list of coordinates (after VRP + NFZ Avoidance + Smoothing)
            final_coords = route_data['coordinates']
            #print('final_coords:\n', final_coords)
            
            # 2. Calculate total physical distance (sum of haversine segments)
            total_dist_meters = 0
            for k in range(len(final_coords) - 1):
                dist_segment = calculate_haversine_distance(final_coords[k], final_coords[k+1])
                if final_coords[k] in [sublist for sublist in data['addresses'][:len(data["start_nodes"]) + len(data["end_nodes"])]
                                       ] and final_coords[k+1] in [sublist for sublist in data['addresses'][:len(data["start_nodes"]) + len(data["end_nodes"])]]:
                    break
                total_dist_meters += dist_segment
                
            # 3. Calculate time based on velocity
            # Retrieve velocity (km/h) and convert to m/s
            velocity_ms = current_app.problem_data["velocity"][vehicle_id] * 0.2777777778
            
            # Avoid division by zero if velocity is invalid
            if velocity_ms > 0:
                total_time_sec = total_dist_meters / velocity_ms
            else:
                total_time_sec = 0
                
            # Update the global makespan counter
            if total_time_sec > max_smoothed_time:
                max_smoothed_time = total_time_sec

            # 4. Format for printing
            t_min = int(total_time_sec // 60)
            t_sec = int(total_time_sec % 60)
            
            # Optional: Update the route dictionary if you want to send this new data to the frontend
            route_data["smoothed_distance_m"] = round(total_dist_meters, 2)
            route_data["smoothed_time_s"] = round(total_time_sec, 2)

            print(f"Vehicle {vehicle_id}:")
            print(f"  Final Smoothed Distance: {total_dist_meters:.2f} meters")
            print(f"  Final Smoothed Time:     {t_min}min {t_sec}secs")

        # Print the final Makespan
        final_min = int(max_smoothed_time // 60)
        final_sec = int(max_smoothed_time % 60)
        print(f"\nTotal Mission Makespan (Smoothed): {final_min}min {final_sec}secs")
        print("-" * 40 + "\n")
        # =============================================================================


    return jsonify(
        success=True,
        message=gettext("Routes created successfully"),
        routes=current_app.routes,
        problem_data=current_app.problem_data,
    )