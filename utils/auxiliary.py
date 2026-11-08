import os
import time
import base64
import re
from typing import Dict, Any
import yaml
import cv2
import numpy as np
from math import pi, cos, sin, radians, degrees, tan, sqrt, atan2
from shapely.geometry import Point, Polygon
from utils.problem_definition import calculate_haversine_distance
import rasterio
from geopy.distance import geodesic

# Decorators
def measure_execution_time(func):
    """
    Decorator function that measures the execution time of a given function.

    Args:
        func (callable): The function to be wrapped and measured.

    Returns:
        callable: The wrapper function that measures the execution time."""

    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"Execution time of {func.__name__}: {execution_time:.2f}s.")
    
        return result

    return wrapper


def string2list(input_str):
    """
    Converts a string to a list. If the string contains only one value,
    the function returns that value as a list containing that integer.
    If the string contains multiple values separated by commas,
    the function returns a list of those values as integers.

    Args:
        input_str (str): The input string to be converted.

    Returns:
        int or list(int): The converted value(s) as integers.
        If the string contains only one value, it is returned as an integer.
        If the string contains multiple values, they are returned as a list of integers.
    """
    parts = input_str.split(",")
    parts = [p.strip() for p in parts if p.strip() != ""]
    if len(parts) >= 1:
        return [int(i) for i in parts]
    else:
        return False


def compress_frame(frame, codec="jpg"):
    """
    Frame compression using cv2.imencode

    Args:
        frame (numpy array): The input frame to be compressed.
        codec (str, optional): The compression codec to be used (default is "jpg").

    Returns:
        numpy array: The compressed frame."""
    _, compressed_frame = cv2.imencode("." + codec, frame)
    
    return compressed_frame


def decompress_frame(frame):
    """
    Decompress frame using a chosen codec.

    Args:
        frame (np.ndarray): The frame to be decompressed.

    Returns:
        np.ndarray: The decompressed frame."""
    decompressed_frame = cv2.imdecode(frame, cv2.IMREAD_UNCHANGED)
    
    return decompressed_frame


def buffer2base64(buffer):
    """
    Encodes a binary buffer as a base64 string.

    Args:
        buffer (bytes): The binary buffer to be encoded.

    Returns:
        str: The base64 encoded string."""
    
    return base64.b64encode(buffer).decode("utf-8")


def base642image(base64_string):
    """
    The base64_to_image function accepts a base64 encoded string and returns an image

    Args:
        base64_string (str): The base64 encoded string representing an image.

    Returns:
        numpy.ndarray: The decoded image in the form of a NumPy array."""
    image_bytes = base64.b64decode(base64_string)
    image_array = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    
    return image


def load_yaml(file: str) -> Dict:
    """
    Load a YAML file into a dictionary.

    Args:
        file (str): Path to the YAML file.

    Returns:
        Dict: Dictionary containing the contents of the YAML file.
    """
    assert os.path.exists(file), f"File not found in path {file}"
    assert isinstance(file, str) and file.endswith(
        ".yaml"
    ), "Not the proper format, must be a yaml file string variable"
    with open(file, errors="ignore", encoding="utf-8") as f:
        s = f.read()  # string
        # Remove special characters
        if not s.isprintable():
            s = re.sub(
                r"[^\x09\x0A\x0D\x20-\x7E\x85\xA0-\uD7FF\uE000-\uFFFD\U00010000-\U0010ffff]+",
                "",
                s,
            )
        params = yaml.safe_load(s)
    
    return params


def update_with_new_nfz(problem_data: Dict[str, Any], nfz_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Update problem_data by either appending or replacing values with data from nfz_data.
    
    - If the key starts with "nfz_ext", override the value in problem_data with the value from nfz_data.
    - Otherwise, append the values from nfz_data to the lists in problem_data.

    Args:
        problem_data (Dict[str, Any]): The original problem definition data.
        nfz_data (Dict[str, Any]): The data from new_nfz.yaml.

    Returns:
        Dict[str, Any]: The updated problem data.
    """
    for key, nfz_value in nfz_data.items():
        if key in problem_data:
            if key.startswith("nfz_ext"):
                # Override the value for keys that start with "nfz_ext"
                problem_data[key] = nfz_value
            else:
                # For other keys, append if the values are lists
                problem_value = problem_data[key]
                if isinstance(problem_value, list) and isinstance(nfz_value, list):
                    problem_data[key] = problem_value + nfz_value
                else:
                    # If they are not lists, we assume it's scalar (but no override here)
                    pass
    
    return problem_data


def update_with_new_area_poi(problem_data: Dict[str, Any], new_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Update problem_data by either appending or replacing values with data from new_data.
    
    - If the key starts with "search", override the value in problem_data with the value from new_data.
    - Otherwise, append the values from new_data to the lists in problem_data.

    Args:
        problem_data (Dict[str, Any]): The original problem definition data.
        new_data (Dict[str, Any]): The data from new_area.yaml.

    Returns:
        Dict[str, Any]: The updated problem data.
    """
    for key, new_value in new_data.items():
        if key in problem_data:
            if key.startswith("search"):
                # Override the value for keys that start with "search"
                problem_data[key] = new_value
            else:
                # For other keys, append if the values are lists
                problem_value = problem_data[key]
                if isinstance(problem_value, list) and isinstance(new_value, list):
                    problem_data[key] = problem_value + new_value
                else:
                    # If they are not lists, we assume it's scalar (but no override here)
                    pass
    
    return problem_data


def update_with_new_uas(problem_data: Dict[str, Any], new_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Update problem_data by either appending values with data from new_data.
    
    - If the 'master_uas' key is null, add the new UAS in the same level as a master

    Args:
        problem_data (Dict[str, Any]): The original problem definition data.
        new_data (Dict[str, Any]): The data from new_uas.yaml.

    Returns:
        Dict[str, Any]: The updated problem data.
    """
    if new_data['master_uas']:
        # Add items to the problem definition
        master_index = new_data['master_uas']
        for key, new_value in new_data.items():
            if key in problem_data:
                if key == 'uas_num':
                    problem_data[key] = new_value + 1
                else:
                    problem_data[key] = [problem_data[key][master_index - 1]] + new_value
        problem_data['master_uas'] = master_index        
    else:
        for key, new_value in new_data.items():
            if key in problem_data:
                if key == 'uas_num':
                   # Add it to the previous UAS number"
                    problem_data[key] += new_value
                else:
                    # For other keys, append if the values are lists
                    problem_value = problem_data[key]
                    if isinstance(problem_value, list) and isinstance(new_value, list):
                       problem_data[key] = problem_value + new_value
                    else:
                        # If they are not lists, we assume it's scalar (but no override here)
                        pass 

    return problem_data

def save_yaml(file, data):
    """
    This function save data in a yaml file

    Args:
        file (str): The file name or path where the data will be saved.
        data (dict): The data to be saved in the yaml file.

    Returns:
        None"""
    with open(file, "w") as file:
        yaml.dump(data, file)


def ensure_folder_exist(path):
    """
    Ensure that a folder exists at the specified path.

    This function takes a path as input and creates any necessary folders in the
    path so that a folder exists at the final path.
    If the folder(s) already exist, the function does not perform any action.

    Args:
        path (str): The path for which the folders need to be created.

    Returns:
        bool: True if the folder(s) already exist(s), False if the folder(s) needed to be created.
    """
    path = str(path)
    separated = path.split(os.path.sep)
    # To consider absolute paths
    if separated[0] == "":
        separated.pop(0)
        separated[0] = os.path.sep + separated[0]
    exists = True
    for f in range(len(separated)):
        path = (
            os.path.sep.join(separated[: f + 1])
            if f > 0
            else (separated[0] + os.path.sep)
        )
        if not os.path.exists(path):
            os.mkdir(path)
            exists = False
    
    return exists


def generate_n_colors(n):
    """
    Generates a list of n colors using the HSV color space.

    This function takes an integer n as input and generates
    a list of n colors in the form of hexadecimal RGB values.
    The colors are evenly distributed in hue with maximum saturation and value.

    Args:
        n (int): The number of colors to generate.

    Returns:
        List[str]: A list of n colors represented as hexadecimal RGB values."""
    from colorsys import hsv_to_rgb

    colors = []
    increment = 360 / n
    for i in range(n):
        hue = int(i * increment)
        rgb = hsv_to_rgb(hue / 360, 1, 1)
        color = f"#{''.join(format(int(c * 255), '02X') for c in rgb)}"
        colors.append(color)
    
    return colors


def adapt_solution(data, manager, routing, solution, coordinates_list):
    """
    Generates a dictionary with the routing data

    Args:
        data (dict): A dictionary with data related to the problem.
        manager (ortools.routing.RoutingIndexManager): A manager object to retrieve nodes.
        routing (ortools.routing.RoutingModel): A routing model.
        solution (ortools.routing.RoutingSolution): A solution object.
        coordinates_list (list): A list of coordinates.

    Returns:
        dict: A dictionary containing the routing data."""
    
    print("\n--- OR-Tools Solution ---")
    print(f"Objective: {solution.ObjectiveValue()}")
    max_route_time = 0
    routes_dict = {}
    routes_dict["routes"] = {}
    routes = []
    # Generate a color for each vehicle route
    colors = generate_n_colors(data["uas_num"])
    #colors = ['#FF0000', "#5719CA", "#000200D6", "#F08E0E"]
    for vehicle_id in range(data["uas_num"]):
        routes_dict["routes"][vehicle_id] = {}
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        #route_dist = 0
        route_time = 0
        nodes = []
        coordinates = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            nodes.append(node)
            coordinates.append(coordinates_list[node])
            plan_output += f" {node} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            #route_dist += routing.GetArcCostForVehicle(
            #    previous_index, index, vehicle_id
            #)
            route_time += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        node = manager.IndexToNode(index)
        nodes.append(node)
        coordinates.append(coordinates_list[node])
        velocity = data["uas_gs"][vehicle_id] * 0.2777777778
        #route_time = int(route_dist / velocity)
        route_dist = route_time * velocity
        route_mins = route_time // 60
        route_secs = route_time % 60
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Time to complete the route: {route_mins}min {route_secs}secs\n"
        plan_output += f"Total distance of the route: {route_dist}m\n"
        print(plan_output)
        max_route_time = max(route_time, max_route_time)
        routes.append(nodes)
        # Save route information
        routes_dict["routes"][vehicle_id]["coordinates"] = coordinates
        routes_dict["routes"][vehicle_id]["nodes"] = nodes
        routes_dict["routes"][vehicle_id]["time"] = route_time
        routes_dict["routes"][vehicle_id]["distance"] = route_dist
        routes_dict["routes"][vehicle_id]["color"] = colors[vehicle_id]
        routes_dict["routes"][vehicle_id]["velocity"] = velocity
    max_route_mins = max_route_time // 60
    max_route_secs = max_route_time % 60
    routes_dict["total_time"] = max_route_time
    print(
        f"Time to complete the longest route: {max_route_mins}min {max_route_secs}secs\n"
    )
    
    return routes_dict


def adapt_solution_general_format(solution_dict, problem_data, coordinates_list):
   
    routes_dict = {"routes": {}}
    max_route_time = 0

    # Import colors
    from utils.auxiliary import generate_n_colors
    colors = generate_n_colors(problem_data["uas_num"])

    # If objective is present, print it
    if "objective" in solution_dict:
        print(f"Objective: {solution_dict['objective']}")

    for vehicle_id, route_info in solution_dict.items():
        # Skip non-vehicle keys like "objective"
        if not isinstance(vehicle_id, int):
            continue
        
        nodes = route_info["nodes"]
        route_time = route_info["time"]
        route_distance = route_info["distance"]

        max_route_time = max(max_route_time, route_time)

        coordinates = [coordinates_list[node] for node in nodes]
        velocity = problem_data["velocity"][vehicle_id] * 0.2777777778

        # ✅ PRINT like OR-Tools does
        print(f"Route for vehicle {vehicle_id}:")
        print(" " + " -> ".join(str(n) for n in nodes))
        route_mins = route_time // 60
        route_secs = route_time % 60
        print(f"Time to complete the route: {route_mins}min {route_secs}secs\n")

        # Store the route
        routes_dict["routes"][vehicle_id] = {
            "coordinates": coordinates,
            "nodes": nodes,
            "time": route_time,
            "distance": route_distance,
            "color": colors[vehicle_id],
            "velocity": velocity
        }

    # ✅ Final print like OR-Tools
    max_route_mins = max_route_time // 60
    max_route_secs = max_route_time % 60
    routes_dict["total_time"] = max_route_time
    print(f"Time to complete the longest route: {max_route_mins}min {max_route_secs}secs\n")

    return routes_dict

def adapt_solution_gurobi_format(solution_dict, problem_data, coordinates_list):
    """
    Convert a Gurobi solution to the same format as adapt_solution() expects
    and print results similarly to OR-Tools.
    
    We reuse the Generl adapter as we designed the output to match.
    """
    print("\n--- Gurobi Solution ---")
    # If the solution dictionary is empty, return an empty structure
    if not solution_dict:
        print("Gurobi found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_scip_format(solution_dict, problem_data, coordinates_list):
    """
    Convert a SCIP solution to the standard application format.
    
    NOTE: Since our find_routes_scip function formats the output to match
    Gurobi's, we can simply reuse the Gurobi adapter. This demonstrates
    the power of a standardized data format.
    """
    print("\n--- SCIP Solution ---")
    # If the solution dictionary is empty, return an empty structure
    if not solution_dict:
        print("SCIP found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_cplex_format(solution_dict, problem_data, coordinates_list):
    """
    Convert a CPLEX (docplex) solution to the standard application format.
    We reuse the Gurobi adapter as we designed the output to match.
    """
    print("\n--- CPLEX Solution ---")
    if not solution_dict:
        print("CPLEX found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_pygad_format(solution_dict, problem_data, coordinates_list):
    """
    Convert a PyGAD (Genetic Algorithm) solution to the standard application format.
    We reuse the Gurobi adapter as we designed the output to match.
    """
    print("\n--- PyGAD Solution ---")
    if not solution_dict:
        print("PyGAD found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_sa_format(solution_dict, problem_data, coordinates_list):
    """
    Convert a Simulated Annealing (simanneal) solution to the standard format.
    We reuse the Gurobi adapter.
    """
    print("\n--- Simulated Annealing (SA) Solution ---")
    if not solution_dict:
        print("SA found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_aco_format(solution_dict, problem_data, coordinates_list):
    """
    Convert an Ant Colony Optimization (ACO) solution to the standard format.
    We reuse the Gurobi adapter.
    """
    print("\n--- Ant Colony Optimization (ACO) Solution ---")
    if not solution_dict:
        print("ACO found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_receding_horizon_format(solution_dict, problem_data, coordinates_list):
    
    print("\n--- Running Receding Horizon Planning ---")
    if not solution_dict:
        print("Receding Horizon Planning found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)

def adapt_solution_decentralized_swarm_format(solution_dict, problem_data, coordinates_list):
    
    print("\n--- Running Decentralized Swarm (CNP) ---")
    if not solution_dict:
        print("Decentralized Swarm (CNP) found no feasible solution.")
        return {"routes": {}, "total_time": 0}
        
    return adapt_solution_general_format(solution_dict, problem_data, coordinates_list)



def line_orientation(wp_i, wp_j):
    """
    Computes the orientation of a line between two waypoints wp_i, wp_j.
    Args:
        wp_i (list): Two dimensional coordinate of the start point of the line.
        wp_j (list): Two dimensional coordinate of the end point of the line.

    Returns:
        Float: The angle (in radians) between the positive x-axis and the line.
    """
    delta_x = wp_j[1] - wp_i[1] # In this code, the wp coordinates are as [lat, lon]
    delta_y = wp_j[0] - wp_i[0]
    
    return np.arctan2(delta_y, delta_x)

def meterTo_lon_lat(displacement, latitude):
    """
    Converts a displacement in meters to geographic degrees.
    Args:
        displacement (float): The displacement in meters.
        latitude (float): The latitude of the point in degrees.

    Returns:
        Tuple: The latitude and longitude displacement.
    """    
    r_earth = 6371000
    delta_longitude = (displacement / r_earth) * (180 / pi) / cos(latitude * pi/180)
    delta_latitude  = (displacement / r_earth) * (180 / pi)
    
    return delta_latitude, delta_longitude

def generate_points_within_polygon(polygon, grid_spacing, altitude, nfz_safety_margin, nfz_center=None, nfz_radius=None, nfz_polygon=None):
    """
    Generates random waypoints uniformly within a specified polygonal search region.

    This function takes a polygonal region defined by its vertices and generates a number
    of waypoints distributed uniformly. No-fly zones are considered to avoid putting points
    within them.

    Args:
        polygon (list): The (latitude, longitude) of the polygon's vertices.
        grid_spacing (float): The minimum distance between the points in degrees.
        altitude (float): The altitude of the drone in meters over the search area.
        nfz_safety_margin (float): A factor multiplied by the circular no-fly zone's radius
            to provide a safety margin around them.
        nfz_polygon (list): A list of lists, each list containing the (latitude, longitude)
            of the vertices of each polygon no-fly zone.
        nfz_center (list): A list of tuples, each tuple containing the (latitude, longitude)
            of the center of each circular no-fly zone.
        nfz_radius (list): Contains the radius of each circular no-fly zone in meter.
        

    Returns:
        list[tuple]: A list of tuples, each tuple containing the (latitude, longitude) of
            each generated waypoints.
    """    
    # Get the bounding box of the polygon
    min_x, min_y, max_x, max_y = polygon.bounds
    
    # Generate points within the bounding box with the specified grid spacing
    x_coords = [x for x in frange(min_x + grid_spacing[0]/2, max_x , grid_spacing[0])]
    y_coords = [y for y in frange(min_y + grid_spacing[1]/2, max_y , grid_spacing[1])]

    points_inside_polygon = []
    boundary_added_points = []
    
    for x in x_coords:
        for y in y_coords:
            point = Point(x, y)
            new_point = [x, y, altitude]
            
            # Check if the point is within the main polygon
            if polygon.contains(point):
                points_inside_polygon.append(new_point)
            elif not polygon.contains(point):
                added_points = [[x + grid_spacing[0]/2, y, altitude], [x - grid_spacing[0]/2, y, altitude],
                                [x, y + grid_spacing[1]/2, altitude], [x, y - grid_spacing[1]/2, altitude]]
                new_added_points = []
                for pnt in added_points:
                    if polygon.contains(Point(pnt[0], pnt[1])):
                        new_added_points.append([pnt[0], pnt[1], altitude])
                        
                
                points_inside_polygon += new_added_points
                boundary_added_points += new_added_points
    
    points_inside_nfzs = []
    points_generated_by_nfzs = []
    for pnt in points_inside_polygon:
        x, y = pnt[0], pnt[1]
        # Check if the new point falls within no-fly zones
        # Circular NFZ
        if nfz_center:
            for i in range(len(nfz_center)):
                # A safety margin equal to 12 percent of the radius is chosen for the circular NFZs
                if point_inside_circle(nfz_center[i], (nfz_safety_margin + 0.02) * nfz_radius[i], pnt[:2]):
                    points_inside_nfzs.append(pnt)
                    generated_points = [[x + grid_spacing[0]/2, y, altitude], [x - grid_spacing[0]/2, y, altitude],
                                        [x, y + grid_spacing[1]/2, altitude], [x, y - grid_spacing[1]/2, altitude]]
                    for gpnt in generated_points:
                        if not point_inside_circle(nfz_center[i], (nfz_safety_margin + 0.02) * nfz_radius[i], (gpnt[0], gpnt[1])) and polygon.contains(Point(gpnt[0], gpnt[1])):
                            points_generated_by_nfzs.append(gpnt)
        # Polygon NFZ                
        if nfz_polygon:
            for i in range(len(nfz_polygon)):
                if point_inside_polygon(nfz_polygon[i], Point(x, y)):
                    points_inside_nfzs.append(pnt)
                    generated_points = [[x + grid_spacing[0]/2, y, altitude], [x - grid_spacing[0]/2, y, altitude],
                                        [x, y + grid_spacing[1]/2, altitude], [x, y - grid_spacing[1]/2, altitude]]
                    for gpnt in generated_points:
                        if not point_inside_polygon(nfz_polygon[i], Point(gpnt[0], gpnt[1])) and polygon.contains(Point(gpnt[0], gpnt[1])):
                            points_generated_by_nfzs.append(gpnt)

    for pnt in points_inside_nfzs:
        points_inside_polygon.remove(pnt)
    points_inside_polygon += points_generated_by_nfzs

    repeated_pnts = []
    for i in range(len(points_inside_polygon)):
        for j in range(i + 1, len(points_inside_polygon)):
            if calculate_haversine_distance(points_inside_polygon[i], points_inside_polygon[j]) < 1:
                repeated_pnts.append(points_inside_polygon[i])
    for pnt in repeated_pnts:
        points_inside_polygon.remove(pnt)

    # A final check for no-fly zones is also needed 
    
    for pnt in points_inside_polygon:
        # Circular NFZ
        if nfz_center:
            for i in range(len(nfz_center)):
                if point_inside_circle(nfz_center[i], (nfz_safety_margin + 0.02) * nfz_radius[i], pnt[:2]):
                    points_inside_polygon.remove(pnt)
                    break
        # Polygon NFZ                
        if nfz_polygon:
            for i in range(len(nfz_polygon)):
                if point_inside_polygon(nfz_polygon[i], Point(pnt[0], pnt[1])):
                    points_inside_polygon.remove(pnt)
                    break

    return points_inside_polygon


def frange(start, stop, step):
    while start < stop:
        yield start
        start += step

def point_inside_polygon(nfz_poly, point):
    """
    Checks if any point falls within a polygon no-fly zone.

    Args:
        nfz_poly (list): A list of lists, each list containing the (latitude, longitude)
            of the vertices of each polygon no-fly zone.
        point (tuple): (latitude, longitude) of the point.
        
    Returns:
        Boolean: Specifies if the point is inside the no-fly zone region.
    """        
    polygon = Polygon(nfz_poly)
    is_inside = polygon.contains(point)
    
    return is_inside

def point_inside_circle(circle_center, circle_r, point):
    """
    Checks if any point falls within a circular no-fly zone.

    Args:
        circle_center (list): A list of tuples, each tuple containing the
            (latitude, longitude) of the center of each circular no-fly zone.
        circle_r (list): Contains the radius of each circular no-fly zone in meter.
        point (tuple): (latitude, longitude) of the point.
        
    Returns:
        Boolean: Specifies if the point is inside the no-fly zone region.
    """   
    distance = calculate_haversine_distance(circle_center, point)
    if distance < circle_r:
        return True

    return False

def read_dem(filename):
    """
    Reads a data elevation model file.

    Args:
        filename (str): A path to the DEM file.
     
    Returns:
        numpy array: The elevation values of the region.
        Affine: The transform matrix.
    """   
    with rasterio.open(filename) as dataset:
        dem_data = dataset.read(1)  # Read the first band (DEM data is usually single-band)
        transform = dataset.transform  # Affine transform for georeferencing

    return dem_data, transform

def get_elevation(dem_data, transform, lon, lat):
    # Convert longitude and latitude to row, column in the raster
    col, row = ~transform * (lon, lat)
    
    # Convert to integer indices
    row, col = int(round(row)), int(round(col))
    
    # Check if indices are within the raster bounds
    return dem_data[row, col]


def get_max_elevation(dem_data, transform, lon1, lat1, lon2, lat2):
   
    # Convert geographic coordinates to raster indices
    col1, row1 = ~transform * (lon1, lat1)  # Top-left
    col2, row2 = ~transform * (lon2, lat2)  # Bottom-right

    # Convert to integer indices
    row1, col1 = int(round(row1)), int(round(col1))
    row2, col2 = int(round(row2)), int(round(col2))

    # Ensure row1 is smaller than row2, and col1 is smaller than col2
    row_min, row_max = sorted([row1, row2])
    col_min, col_max = sorted([col1, col2])

    # Validate bounds
    if (
        0 <= row_min < dem_data.shape[0] and 0 <= row_max < dem_data.shape[0] and
        0 <= col_min < dem_data.shape[1] and 0 <= col_max < dem_data.shape[1]
    ):
        # Extract the subregion and find max elevation
        subregion = dem_data[row_min:row_max+1, col_min:col_max+1]
        return subregion.max()
    else:
        raise ValueError("The specified subregion is out of the DEM bounds.")


def nfz_intersect_vertically(alt_min_max, poi_i, poi_j):
    if (poi_i[2] >= alt_min_max[0] and poi_i[2] <= alt_min_max[1]) or (poi_j[2] >= alt_min_max[0] and poi_j[2] <= alt_min_max[1]):
        return True
    else:
        return False


# Finds all num_points nodes on a circle primeter and start and goal nodes as well 
def circle_peripheral_nodes(center, radius, num_points=32):
    # Convert meter-based coordinates to latitude and longitude coordinates
    center_lat, center_lon = center
    
    # Calculate scaling factors for latitude and longitude
    lat_scale = radius / 111000  # 1 degree of latitude is approximately 111 kilometers
    lon_scale = radius / (111000 * cos(center_lat * pi / 180))  # Scaling factor for longitude

    theta = [2 * pi * i / num_points for i in range(num_points)]
    circle_x = [center_lon + lon_scale * cos(t) for t in theta]
    circle_y = [center_lat + lat_scale * sin(t) for t in theta]

    nodes_on_circle = list(zip(circle_y, circle_x))  # Note: latitude corresponds to y-axis, longitude to x-axis
    
    return nodes_on_circle

def generate_peripheral_points(center, radius, num_points=32):
    """
    Generates a specific number of peripheral points uniformly on a circle given
    the circle center and its radius.

    Args:
        center (tuple): The (latitude, longitude) of the center of the circle.
        radius (float): The radius of the circle in meter.
        num_points (int): The number of distribuited points.
           
    Returns:
        List: A list of generated points.
    """
    lat, lon = center
    points = []
    
    # Calculate angle increment for the given number of points
    angle_increment = 360 / num_points
    
    for i in range(num_points):
        # Calculate the angle in radians
        angle = radians(i * angle_increment)
        
        # Calculate the destination point given the bearing and distance
        destination = geodesic(meters=radius).destination((lat, lon), degrees(angle))
        points.append([destination.latitude, destination.longitude])
    
    return points

def scale_polygon(vertices, margin = 0.002):
    """
    Scale the polygon outward from the centroid to preserve angles.
    Each vertex is moved away from the centroid by a scale factor.
    """
    original_polygon = Polygon(vertices)
    scale_center = original_polygon.centroid
    scaled_vertices = []
    for x, y in vertices:
        # Vector from centroid to vertex
        dx = x - scale_center.x
        dy = y - scale_center.y
        distance = np.hypot(dx, dy)
        if distance == 0:
            # Skip scaling the centroid itself if it’s a vertex
            scaled_vertices.append((float(x), float(y)))
            continue
        scale = (distance + margin) / distance
        new_x = scale_center.x + dx * scale
        new_y = scale_center.y + dy * scale
        scaled_vertices.append((float(new_x), float(new_y)))
    return scaled_vertices


