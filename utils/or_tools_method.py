from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from utils.auxiliary import measure_execution_time
from utils.problem_definition import calculate_haversine_distance
import math


@measure_execution_time
def find_routes(problem_data, routing_data):
    """
    Find routes using a given problem data and routing data.

    This function takes in problem data and routing data and
    uses them to find routes using the Google OR-Tools library.

    Args:
        problem_data (dict): A dictionary containing the problem data, including the distance
            matrix, number of vehicles, start nodes, end nodes, max flight time, and velocity.
        routing_data (dict): A dictionary containing the routing data, including the first
            solution strategy, local search strategy, solution limit, time limit,
            log search flag, and LNS time limit.

    Returns:
        tuple: A tuple containing the problem data, routing index manager,
            routing model, and the solution to the routing problem.
    """
    for i in range(len(problem_data["distance_matrix"])):
        problem_data["distance_matrix"][i] = [int(s) for s in problem_data["distance_matrix"][i]]

    # --- SHARED POOL LOGIC ---
    # We treat end_nodes as pools.
    # We assign an end node to each vehicle from these pools.
    uas_num = problem_data["uas_num"]
    start_nodes = problem_data["start_nodes"]
    all_end_nodes = problem_data["end_nodes"]
    assigned_end_nodes = [all_end_nodes[i % len(all_end_nodes)] for i in range(uas_num)]
    
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(
        len(problem_data["distance_matrix"]),
        uas_num,
        start_nodes,
        assigned_end_nodes,
    )

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # --- Helper to get Wind at a specific Node ---
    def get_node_wind(node_index):
        """
        Retrieves (speed, direction) for a specific node index.
        Handles cases where wind data might be a list (per node) or scalar (global).
        """
        w_speed_data = problem_data.get("wind_intencity", 0)
        w_dir_data = problem_data.get("wind_direction", 0)
        
        # 1. Handle Speed
        if isinstance(w_speed_data, list):
            # List matches total nodes (Starts + Ends + PoIs)
            if node_index < len(w_speed_data):
                w_s = w_speed_data[node_index]
            else:
                w_s = 0 # Fallback if index out of bounds (should not happen if data is consistent)
        else:
            # Scalar global value
            w_s = w_speed_data

        # 2. Handle Direction
        if isinstance(w_dir_data, list):
            # List matches total nodes (Starts + Ends + PoIs)
            if node_index < len(w_dir_data):
                w_d = w_dir_data[node_index]
            else:
                w_d = 0 # Fallback
        else:
            # Scalar global value
            w_d = w_dir_data
            
        return w_s, w_d

    # -------------------------------------------------------------------------
    # 1. TIME DIMENSION
    # -------------------------------------------------------------------------
    def time_callback(from_index, to_index, vehicle):
        """
        Returns the travel time between the two nodes.
        """
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        # Virtual travel between end depots in the pool is cost 0
        if from_node in all_end_nodes and to_node in all_end_nodes:
            return 0
                
        # --- BEST DEPOT SELECTION LOGIC ---
        #if to_node in all_end_nodes:
            # Distance is min distance from source to ANY end node
        #    distance = min(problem_data["distance_matrix"][from_node][e] for e in all_end_nodes)
        
        #else:
            # Normal travel between PoIs
        distance = problem_data["distance_matrix"][from_node][to_node]
        
        # Velocity in problem_data is Ground Speed (km/h) -> m/s
        ground_speed_ms = problem_data["velocity"][vehicle] * 0.2777777778

        # --- IAS CONSTRAINT CHECK ---
        # Calculates required Airspeed Vector (V_TAS) = V_Ground - V_Wind
        # Checks magnitude against max_ias
        
        if "max_ias" in problem_data and "wind_intencity" in problem_data:
            # max_ias in YAML is km/h -> convert to m/s
            max_allowed_ias = problem_data["max_ias"][vehicle] * 0.2777777778
            
            # Get wind at start and end nodes
            ws1, wd1 = get_node_wind(from_node)
            ws2, wd2 = get_node_wind(to_node)
            
            # Linear averaging as requested
            w_speed = (ws1 + ws2) / 2.0
            w_dir_avg_deg = (wd1 + wd2) / 2.0
            
            # Convert Compass (0=N, 90=E) to Math (0=E, CCW)
            w_dir_rad = math.radians(90 - w_dir_avg_deg)
            
            # Wind vector components
            # Note: Using 'wind_intencity' as per user YAML typo
            #w_speed = problem_data["wind_intencity"]
            #w_dir_deg = problem_data["wind_direction"]
            # Convert Compass (0=N, 90=E) to Math (0=E, CCW)
            #w_dir_rad = math.radians(90 - w_dir_deg)

            # Get Coordinates
            p1 = problem_data["addresses"][from_node]
            p2 = problem_data["addresses"][to_node]
            
            # Calculate Path Angle (Bearing)
            lat_mid = (p1[0] + p2[0]) / 2.0
            m_per_deg_lat = 111320
            m_per_deg_lon = 40075000 * math.cos(math.radians(lat_mid)) / 360
            delta_y = (p2[0] - p1[0]) * m_per_deg_lat
            delta_x = (p2[1] - p1[1]) * m_per_deg_lon
            path_angle = math.atan2(delta_y, delta_x) # Angle from East (CCW)

            # Vector Components
            vg_x = ground_speed_ms * math.cos(path_angle)
            vg_y = ground_speed_ms * math.sin(path_angle)
            
            vw_x = w_speed * math.cos(w_dir_rad)
            vw_y = w_speed * math.sin(w_dir_rad)
            
            # V_TAS = V_Ground - V_Wind
            vtas_x = vg_x - vw_x
            vtas_y = vg_y - vw_y
            
            required_ias = math.sqrt(vtas_x**2 + vtas_y**2)
            
            # If required airspeed is higher than max allowed, ban this edge
            if required_ias > max_allowed_ias:
                return 100000000 # Return Infinity (Large penalty) to make edge infeasible
        
        travel_time = int(math.ceil(distance / ground_speed_ms))
        #travel_time = int(distance / ground_speed_ms)
        return travel_time 
    
        
    transit_callback_indices = []
    for vehicle_id in range(problem_data["uas_num"]):
        def transit_callback(from_index, to_index, v=vehicle_id):
            return time_callback(from_index, to_index, v)
        transit_callback_index = routing.RegisterTransitCallback(transit_callback)
        transit_callback_indices.append(transit_callback_index)
        routing.SetArcCostEvaluatorOfVehicle(transit_callback_index, vehicle_id)


    # Autonomy Constraint (Max Flight Time)
    routing.AddDimensionWithVehicleTransitAndCapacity(
        transit_callback_indices,
        0,  # no slack
        [ int(60 * t) for t in problem_data["autonomy"]], 
        True,
        "Time",
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    # --- Set the Objective: Minimize Makespan ---
    time_dimension.SetGlobalSpanCostCoefficient(max(problem_data["autonomy"]) * 60)

    # -------------------------------------------------------------------------
    # 2. POI DEADLINE CONSTRAINT
    # -------------------------------------------------------------------------
    if "poi_max_arrival_time" in problem_data:
        for node_index, max_time_sec in problem_data["poi_max_arrival_time"].items():
            index = manager.NodeToIndex(node_index)
            if index == -1: continue
            time_dimension.CumulVar(index).SetRange(0, int(max_time_sec))

    # -------------------------------------------------------------------------
    # 3. ENERGY DIMENSION
    # -------------------------------------------------------------------------
    if "battery_capacity_joules" in problem_data:
        
        def energy_callback(from_index, to_index, vehicle):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            
            if from_node == to_node: return 0
            if from_node in all_end_nodes and to_node in all_end_nodes: return 0

            # --- BEST DEPOT SELECTION LOGIC (For Energy) ---
            #if to_node in all_end_nodes:
            #    dist_m = min(problem_data["distance_matrix"][from_node][e] for e in all_end_nodes)
            #else:
            dist_m = problem_data["distance_matrix"][from_node][to_node]
            
            # --- IMPLEMENT EQUATION 1 HERE ---
            # Use problem_data["uas_params"] etc.
            
            # 1. Get Physical Constants
            rho = problem_data["air_density"]
            S = problem_data["wing_area"][vehicle]
            Cd = problem_data["drag_coeff"][vehicle]
            eta = problem_data["prop_efficiency"][vehicle]
            V_batt = problem_data["battery_voltage"][vehicle]
            
            # 2. Get Velocities
            # Ground Speed (m/s)
            v_g_ms = problem_data["velocity"][vehicle] * 0.277778 
            
            # Wind Vector (We need projection W_parallel)
            # Calculate heading of the segment (from_node -> to_node)
            # addresses list contains [lat, lon, alt]
            p1 = problem_data["addresses"][from_node]
            p2 = problem_data["addresses"][to_node]

            ws1, wd1 = get_node_wind(from_node)
            ws2, wd2 = get_node_wind(to_node)
            
            w_speed = (ws1 + ws2) / 2.0
            w_dir_deg = (wd1 + wd2) / 2.0
            w_dir_rad = math.radians(90 - w_dir_deg)
            
            # Calculate bearing (angle of flight path) in radians
            # Actually for simple projection:
            # Wind direction in YAML is usually degrees (0=North, 90=East)
            # Convert both to standard math angle (0=East, CCW) to be safe
            
            # Simplified projection logic:
            # If wind_direction is meteorological (coming FROM), convert to blowing TO.
            # Assuming wind_direction in YAML is "Blowing TO" in degrees (0=N, 90=E)
                        
            # Calculate path angle
            # lat/lon to meters approx for direction
            lat_mid = (p1[0] + p2[0]) / 2.0
            m_per_deg_lat = 111320
            m_per_deg_lon = 40075000 * math.cos(math.radians(lat_mid)) / 360
            delta_y = (p2[0] - p1[0]) * m_per_deg_lat
            delta_x = (p2[1] - p1[1]) * m_per_deg_lon
            path_angle = math.atan2(delta_y, delta_x)
            
            # Project wind onto path: |W| * cos(path_angle - wind_angle)
            w_parallel = w_speed * math.cos(path_angle - w_dir_rad)
            
            # 3. Calculate Equation Terms
            # E = (1 / 2*eta) * rho * S * Cd * (Vg - W_par)^3 * (dist / Vg)
            
            term_const = 1.0 / (2.0 * eta)
            term_aero = rho * S * Cd
            
            # V_tas estimated as V_g - W_parallel (Scalar approximation)
            # Max/Min check to avoid negative airspeed in extreme tailwind (unlikely but safe)
            v_tas_est = max(0, v_g_ms - w_parallel) 
            
            term_cubic = v_tas_est ** 3
            time_segment = dist_m / v_g_ms
            
            energy_joules = term_const * term_aero * term_cubic * time_segment
            
            return int(energy_joules)

        energy_indices = []
        for vehicle_id in range(uas_num):
            def e_callback(from_index, to_index, v=vehicle_id):
                return energy_callback(from_index, to_index, v)
            e_index = routing.RegisterTransitCallback(e_callback)
            energy_indices.append(e_index)

        min_level_ratio = problem_data["min_battery_level_ratio"]
        usable_capacities = []
        
        for cap in problem_data["battery_capacity_joules"]:
            usable = int(1 * (cap * (1.0 - min_level_ratio)))
            #usable = int(cap)
            usable_capacities.append(usable)

        routing.AddDimensionWithVehicleTransitAndCapacity(
            energy_indices,
            0, 
            usable_capacities,
            True,
            "Energy"
        )

    # -------------------------------------------------------------------------
    # Solver Parameters & Execution
    # -------------------------------------------------------------------------
    # Setting first solution heuristic
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    strategy_value = getattr(
        routing_enums_pb2.FirstSolutionStrategy, routing_data["first_solution_strategy"]
    )
    search_parameters.first_solution_strategy = strategy_value

    # Setting search strategy
    strategy_value = getattr(
        routing_enums_pb2.LocalSearchMetaheuristic, routing_data["local_search_strategy"],
    )
    search_parameters.local_search_metaheuristic = strategy_value

    # Additional options to the routing problem
    if routing_data["solution_limit"] is not None:
        search_parameters.solution_limit = routing_data["solution_limit"]
    if routing_data["time_limit"] is not None:
        search_parameters.time_limit.seconds = routing_data["time_limit"]
    if routing_data["log_search"] is not None:
        search_parameters.log_search = routing_data["log_search"]
    if routing_data["lns_time_limit"] is not None:
        search_parameters.lns_time_limit.seconds = routing_data["lns_time_limit"]

    # Solve the problem and return the solution
    return (
        problem_data,
        manager,
        routing,
        routing.SolveWithParameters(search_parameters),
    )

def extract_arrival_times(manager, routing, solution, uas_num):
    """
    Extracts the arrival time (cumul var value) for every visited node.
    
    Args:
        manager: The RoutingIndexManager.
        routing: The RoutingModel.
        solution: The Assignment solution.
        uas_num: Number of vehicles.
        
    Returns:
        dict: A mapping of {node_index: arrival_time_seconds}
    """
    if not solution:
        return {}
        
    arrival_times_list = []
    time_dimension = routing.GetDimensionOrDie("Time")
    
    for vehicle_id in range(uas_num):
        arrival_times = {}
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            # Get the cumulative time variable for the current node
            time_var = time_dimension.CumulVar(index)
            # The solution stores the value (Min() usually suffices if fixed)
            arrival_time = solution.Min(time_var)
            
            # Convert internal index to actual node index
            node_index = manager.IndexToNode(index)
            arrival_times[node_index] = arrival_time
            
            # Move to next node
            index = solution.Value(routing.NextVar(index))
            
        # Capture the end node (depot arrival)
        time_var = time_dimension.CumulVar(index)
        arrival_time = solution.Min(time_var)
        node_index = manager.IndexToNode(index)
        arrival_times[node_index] = arrival_time
        arrival_times_list.append(arrival_times)
        
    return arrival_times_list

def extract_final_energy_consumption(manager, routing, solution, uas_num, problem_data=None):
    """
    Extracts the total energy consumed (Joules) for each vehicle at the end of its route.
    Returns: dict {vehicle_id: consumed_joules}
    """
    if not solution: return {}
    
    # Robust Check: Avoid HasDimension which fails in some SWIG wrappers
    has_energy_config = problem_data and "battery_capacity_joules" in problem_data
    if not has_energy_config:
        return {}
        
    try:
        energy_dimension = routing.GetDimensionOrDie("Energy")
    except Exception:
        # Fallback if dimension wasn't created despite config
        return {}
        
    consumed_energy = {}
    
    for vehicle_id in range(uas_num):
        # Get the index of the End Node for this vehicle
        end_index = routing.End(vehicle_id)
        # Get the cumulative variable at the end node
        energy_var = energy_dimension.CumulVar(end_index)
        # Extract value
        consumed_energy[vehicle_id] = solution.Min(energy_var)
        
    return consumed_energy
