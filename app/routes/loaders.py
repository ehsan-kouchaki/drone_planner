from flask import Blueprint, current_app, request, jsonify
from flask_babel import gettext
from utils.problem_definition import detect_address_format
from utils.load_parameters import *
from utils.auxiliary import *

# Define a Flask blueprint for handling loading-related routes
loaders_blueprint = Blueprint("loaders_blueprint", __name__)


@loaders_blueprint.route("/load-problem", methods=["GET"])
def load_yaml_problem():
    """
    Load problem definition data from a YAML file, convert the addresses to coordinates,
        and store them in the application context.

    Args:
        None

    Returns:
        None"""
    coord_inds = []
    if request.method == "GET":
        current_app.problem_data = load_problem_definition()
       
        # Add basic parameters
        current_app.problem_data['simulation_time_step'] = 10
        current_app.problem_data['simulation_time_step_factor'] = 0.01
        current_app.problem_data['nfz_safety_margin'] = 1.1 # The factor that will be multiplied by the radius
        current_app.problem_data['scan_overlap'] = 0.2 # fraction of WP distances
        current_app.problem_data['travel_mode'] = 'flight'
        current_app.problem_data['autonomy'] = [_ // 60 for _ in current_app.problem_data['autonomy']]
        current_app.problem_data['velocity'] = current_app.problem_data['uas_gs']
       
        
        # No-fly zonesz
        nfz_radius = nfz_center = nfz_polygon = None
        if current_app.problem_data['nfz_int_circ_r']:
            num_circular_nfz = len(current_app.problem_data['nfz_int_circ_r'])
            nfz_center = [[current_app.problem_data['nfz_int_circ_lat'][_], current_app.problem_data['nfz_int_circ_long'][_]] for _ in range(num_circular_nfz)]
            nfz_radius = current_app.problem_data['nfz_int_circ_r']                
        
        if current_app.problem_data['nfz_int_poly_lat']:
            nfz_p_lat = current_app.problem_data['nfz_int_poly_lat']
            nfz_p_long = current_app.problem_data['nfz_int_poly_long']
            nfz_polygon_original = [[(nfz_p_lat[j][i], nfz_p_long[j][i]) for i in range(len(nfz_p_lat[j]))] for j in range(len(nfz_p_lat))]
            # nfz with safety margin
            nfz_polygon = []
            for nfz in nfz_polygon_original:
                nfz_polygon.append(scale_polygon(nfz))
        
        # Adding depotes addresses
        uas_num = current_app.problem_data['uas_num']
        start_depot_num = current_app.problem_data['uas_num']
        end_depot_num = current_app.problem_data['end_depot_num']
        current_app.problem_data['start_nodes'] = [_ for _ in range(start_depot_num)]
        current_app.problem_data['end_nodes'] = [_ for _ in range(start_depot_num, start_depot_num + end_depot_num)]
        addresses = []
        addresses += [[current_app.problem_data['initial_lat'][_], current_app.problem_data['initial_long'][_], current_app.problem_data['initial_alt'][_]]
                        for _ in range(start_depot_num)]
        addresses += [[current_app.problem_data['final_lat'][_], current_app.problem_data['final_long'][_], current_app.problem_data['final_alt'][_]]
                            for _ in range(end_depot_num)]
        
        # Cummulate all nodes for constructing the DEM for the region of interest
        nodes_on_nfzs = []
        if nfz_center:
            nfz_num_points = [int(_/30) for _ in nfz_radius]
            for _ in range(len(nfz_center)):
                nodes_on_nfzs += circle_peripheral_nodes(nfz_center[_], nfz_radius[_], nfz_num_points[_])
        if nfz_polygon:
            for poly_coord in nfz_polygon:
                nodes_on_nfzs += poly_coord 
        node_of_gs = (current_app.problem_data['gs_lat'], current_app.problem_data['gs_long'])

        if not 'master_uas' in current_app.problem_data:
            if current_app.problem_data['mission_type'] == 'visit':            
                addresses += [[current_app.problem_data['poi_lat'][_], current_app.problem_data['poi_long'][_], current_app.problem_data['poi_alt'][_]]
                            for _ in range(len(current_app.problem_data['poi_lat']))]
                
            elif current_app.problem_data['mission_type'] == 'search':            
                overlap = current_app.problem_data['scan_overlap']
                if current_app.problem_data['search_poly_lat']:
                    search_alt = current_app.problem_data['search_poly_alt']
                    # Max ground level altitude
                    max_GL = 1000
                    width_of_view = 2 * (1 - overlap) * (search_alt - max_GL) * tan(radians(min(min(current_app.problem_data['fov_x']), min(current_app.problem_data['fov_y']))))
                    wp_distance = max(width_of_view, current_app.problem_data['min_dist_wp'])
                    polygonCoords = [[current_app.problem_data['search_poly_lat'][_], current_app.problem_data['search_poly_long'][_]]
                                    for _ in range(len(current_app.problem_data['search_poly_lat']))]
                    polygonCoords += [polygonCoords[0]]
                    polygon = Polygon(polygonCoords)                
                    grid_spacing = meterTo_lon_lat(wp_distance, current_app.problem_data['search_poly_lat'][0])               
                    addresses += generate_points_within_polygon(polygon, grid_spacing, search_alt,
                                                                current_app.problem_data['nfz_safety_margin'], nfz_center, nfz_radius, nfz_polygon)

                if current_app.problem_data['search_circ_lat']:
                    search_alt = current_app.problem_data['search_circ_alt']
                    center = current_app.problem_data['search_circ_lat'], current_app.problem_data['search_circ_long']
                    radius = current_app.problem_data['search_circ_r']
                    polygonCoords = circle_peripheral_nodes(center, radius)
                    
                    # Max ground level altitude
                    max_GL = 1000
                    width_of_view = 2 * (1 - overlap) * (search_alt - max_GL) * tan(radians(min(min(current_app.problem_data['fov_x']), min(current_app.problem_data['fov_y']))))
                    wp_distance = max(width_of_view, current_app.problem_data['min_dist_wp'])
                    polygonCoords.append(polygonCoords[0])
                    polygon = Polygon(polygonCoords)
                    grid_spacing = meterTo_lon_lat(wp_distance, current_app.problem_data['search_circ_lat'])
                    addresses += generate_points_within_polygon(polygon, grid_spacing, search_alt,
                                                                current_app.problem_data['nfz_safety_margin'], nfz_center, nfz_radius, nfz_polygon)
           
        elif 'master_uas' in current_app.problem_data:
            master_index = current_app.problem_data['master_uas']
            with open('output/poi_routes.yaml', errors="ignore", encoding="utf-8") as f:
                s = f.read()  # string
            poi_routes = yaml.safe_load(s)
            addresses += poi_routes[master_index - 1][1:-1]
        
    
        current_app.problem_data['addresses'] = addresses            
        
        for i, address in enumerate(current_app.problem_data["addresses"]):
            c_format = detect_address_format(address)
            if c_format == "str-coordinates":
                coordinates = address.replace(" ", "").split(",")
            elif c_format == "name":
                coordinates = current_app.converter.address2coords(address)
            else:
                coordinates = address
            if coordinates is None:
                return jsonify(
                    success=False,
                    message=gettext("Failed to convert address"),
                    coordinates=None,
                )
            coordinates = [float(i) for i in coordinates]
            if coordinates not in current_app.problem_data["addresses"]:
                current_app.problem_data["addresses"].append(coordinates)
                idx = len(current_app.problem_data["addresses"]) - 1
            else:
                idx = current_app.problem_data["addresses"].index(coordinates)
            coord_inds.append((coordinates, idx))
            current_app.problem_data["coord_inds"] = coord_inds
        return jsonify(
            success=True,
            message=gettext("Data loaded properly"),
            problem_data=current_app.problem_data,
        )
