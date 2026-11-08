import math
import numpy as np

def calculate_bearing(start_point, end_point):
    """
    Calculates the initial compass bearing from start to end point.
    Formula: Forward Azimuth on a Sphere.
    """
    lat1 = math.radians(start_point[0])
    lon1 = math.radians(start_point[1])
    lat2 = math.radians(end_point[0])
    lon2 = math.radians(end_point[1])

    d_lon = lon2 - lon1

    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - \
        math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)

    initial_bearing = math.atan2(y, x)
    
    # Normalize to 0-360 degrees
    initial_bearing = math.degrees(initial_bearing)
    return (initial_bearing + 360) % 360

def get_destination_point(start_point, bearing_degrees, distance_meters):
    """
    Calculates a new coordinate given a start point, bearing, and distance.
    Formula: Direct Geodesic Problem (Spherical approximation).
    """
    R = 6371000  # Earth Radius in meters

    lat1 = math.radians(start_point[0])
    lon1 = math.radians(start_point[1])
    bearing = math.radians(bearing_degrees)

    # Calculate new latitude
    new_lat = math.asin(math.sin(lat1) * math.cos(distance_meters / R) +
                        math.cos(lat1) * math.sin(distance_meters / R) * math.cos(bearing))

    # Calculate new longitude
    new_lon = lon1 + math.atan2(math.sin(bearing) * math.sin(distance_meters / R) * math.cos(lat1),
                                math.cos(distance_meters / R) - math.sin(lat1) * math.sin(new_lat))

    return (math.degrees(new_lat), math.degrees(new_lon))

def get_bisector_circle_robust(p1, p2, p3, distance_R, num_points=16):
    """
    Generates points on a circle centered on the angle bisector using Spherical Trigonometry.
    Ensures P2 is one of the points.
    
    Args:
        p1, p2, p3: (lat, lon) tuples.
        distance_R: Radius of the circle (distance from P2 to Center).
        num_points: Number of points on the circle.
    """
    
    # --- 1. Calculate Bearings from P2 ---
    # Bearing P2 -> P1
    theta1 = calculate_bearing(p2, p1)
    # Bearing P2 -> P3
    theta2 = calculate_bearing(p2, p3)

    # --- 2. Calculate Bisector Bearing ---
    # We use vector addition of the angles to find the mean angle (bisector).
    # This automatically handles the 0/360 degree wrapping (e.g., bisector of 350 and 10 is 0).
    u1_x = math.cos(math.radians(theta1))
    u1_y = math.sin(math.radians(theta1))
    
    u2_x = math.cos(math.radians(theta2))
    u2_y = math.sin(math.radians(theta2))
    
    # Sum of unit vectors bisects the angle
    bisect_x = u1_x + u2_x
    bisect_y = u1_y + u2_y
    
    # Calculate angle of the bisector
    bisector_bearing = math.degrees(math.atan2(bisect_y, bisect_x))
    bisector_bearing = (bisector_bearing + 360) % 360

    # Note: The vector sum naturally finds the bisector of the "inner" angle.
    # If the vectors are 180 degrees apart, result is (0,0) and angle is undefined.
    if bisect_x == 0 and bisect_y == 0:
        # Edge case: Straight line. Bisector is perpendicular (+90 deg).
        bisector_bearing = (theta1 + 90) % 360

    # --- 3. Find the Center of the Circle (C) ---
    # Move 'distance_R' along the bisector bearing from P2
    center_point = get_destination_point(p2, bisector_bearing, distance_R)

    # --- 4. Generate Perimeter Points ---
    perimeter_points = []
    
    # To ensure P2 is in the list, we need the exact bearing from Center -> P2.
    # On a sphere, Bearing(C->P2) is NOT exactly Bearing(P2->C) + 180.
    # We calculate it directly to be precise.
    start_angle = calculate_bearing(center_point, p2)
    
    angle_step = 360.0 / num_points

    for i in range(num_points):
        # Current bearing from Center
        current_bearing = start_angle + (i * angle_step)
        
        # Find point coordinate
        pt = get_destination_point(center_point, current_bearing, distance_R-1)
        perimeter_points.append(pt)

    return center_point, perimeter_points

# =============================================================================
# 1. PROJECTION HELPERS
# =============================================================================
def latlon_to_meters(lat, lon, lat_ref, lon_ref):
    """Projects Lat/Lon to local Cartesian meters (Flat Earth approximation)."""
    R = 6378137.0  # Earth radius in meters
    x = R * math.radians(lon - lon_ref) * math.cos(math.radians(lat_ref))
    y = R * math.radians(lat - lat_ref)
    return x, y

def meters_to_latlon(x, y, lat_ref, lon_ref):
    """Un-projects local Cartesian meters back to Lat/Lon."""
    R = 6378137.0
    lat = lat_ref + math.degrees(y / R)
    lon = lon_ref + math.degrees(x / (R * math.cos(math.radians(lat_ref))))
    return lat, lon

# =============================================================================
# 2. GEOMETRY GENERATORS
# =============================================================================
def generate_arc_points(center, r, ang_start, ang_end, direction, num_points=32):
    """
    Generates points along an arc.
    direction: +1 for Counter-Clockwise, -1 for Clockwise
    """
    # Normalize angle difference to ensure we draw the short/correct way
    # If we want to turn Left (1), we expect angle to increase.
    # If we want to turn Right (-1), we expect angle to decrease.
    
    if direction == 1: # CCW
        while ang_end < ang_start:
            ang_end += 2 * math.pi
    else: # CW
        while ang_end > ang_start:
            ang_end -= 2 * math.pi
            
    # Dynamic resolution: ensure curves look smooth even if large
    arc_angle = abs(ang_end - ang_start)
    calc_points = int(arc_angle * 20) # resolution factor
    n = max(num_points, calc_points)
    
    angles = np.linspace(ang_start, ang_end, n)
    points = []
    for a in angles:
        px = center[0] + r * math.cos(a)
        py = center[1] + r * math.sin(a)
        points.append((px, py))
    return points

def generate_overfly_path(p_prev, p_curr, p_next, r_min_meters):
    """
    Generates an 'Overfly' S-turn maneuver.
    
    Logic:
    1. Determines turn direction based on the corner angle.
    2. Calculates an 'Intercept Heading' that is 45 degrees *past* the outgoing heading.
    3. Arc 1: Turns from incoming heading to Intercept Heading.
    4. Straight: Flies on Intercept Heading until it is at the correct lateral offset from the output line.
    5. Arc 2: Turns from Intercept Heading back to Outgoing Heading, ending tangent to the line.
    """
    
    # 1. Setup Local Projection Reference (centered at p_curr)
    ref_lat, ref_lon = p_curr[0], p_curr[1]
    
    # 2. Project points to Meters
    prev_m = latlon_to_meters(p_prev[0], p_prev[1], ref_lat, ref_lon)
    curr_m = latlon_to_meters(p_curr[0], p_curr[1], ref_lat, ref_lon) 
    next_m = latlon_to_meters(p_next[0], p_next[1], ref_lat, ref_lon)
    
    # Convert to numpy
    P_prev = np.array(prev_m)
    P_curr = np.array(curr_m)
    P_next = np.array(next_m)
    
    # 3. Vectors & Headings
    v_in = P_curr - P_prev
    v_out = P_next - P_curr
    
    psi_in = math.atan2(v_in[1], v_in[0])
    psi_out = math.atan2(v_out[1], v_out[0])
    
    # 4. Determine Turn Direction (+1 Left/CCW, -1 Right/CW)
    angle_diff = (psi_out - psi_in + math.pi) % (2 * math.pi) - math.pi
    turn_dir = 1 if angle_diff > 0 else -1
    
    # 5. Define Intercept Heading 
    # We turn PAST the psi_out by 45 degrees.
    intercept_angle_val = math.radians(45)
    psi_int = psi_out + (turn_dir * intercept_angle_val)
    
    # =========================================================
    # ARC 1: The Overshoot Turn (P_curr -> P1)
    # =========================================================
    # Center C1 is perpendicular to incoming heading
    c1_angle = psi_in + (turn_dir * math.pi / 2)
    C1 = P_curr + r_min_meters * np.array([math.cos(c1_angle), math.sin(c1_angle)])
    
    # Start angle on C1
    ang_s1 = c1_angle - math.pi
    # End angle on C1 (where tangent is psi_int)
    ang_e1 = psi_int - (turn_dir * math.pi / 2)
    
    arc1_pts_m = generate_arc_points(C1, r_min_meters, ang_s1, ang_e1, turn_dir)
    P1_end = np.array(arc1_pts_m[-1]) # This is where we start the straight line
    
    # =========================================================
    # STRAIGHT LEG & ARC 2 (Geometric Solver)
    # =========================================================
    # We define the "Output Line" passing through P_curr with heading psi_out.
    # We want to fly straight (heading psi_int) until we reach a point P2.
    # From P2, we execute Arc 2 (Turn -turn_dir) to merge.
    # For smooth merge, P2 must be at a specific signed distance from the Output Line.
    
    # A. Calculate Required Signed Distance (The 'Versine' offset)
    # If we turn 'turn_dir' to overshoot, we end up on that side (Left or Right).
    # To return, we turn '-turn_dir'.
    # The geometric offset created by a 45 deg turn of radius R is R * (1 - cos(45)).
    # Since we are on the 'overshoot' side, the distance is positive relative to the normal?
    # Let's define Normal Vector 'n' pointing 90 deg Left of psi_out.
    n_angle = psi_out + (math.pi / 2)
    n_vec = np.array([math.cos(n_angle), math.sin(n_angle)])
    
    # If turn_dir is +1 (Left), we overshoot to the Left. We are at positive distance along n_vec.
    # We need to be at distance +R(1-cos45) to start the return Right turn.
    # If turn_dir is -1 (Right), we overshoot to Right (negative n_vec). 
    # We need to be at distance -R(1-cos45).
    target_dist = -turn_dir * r_min_meters * (1 - math.cos(intercept_angle_val))
    
    # B. Calculate Current Signed Distance of P1 from Line
    # Vector from P_curr (point on line) to P1
    vec_curr_p1 = P1_end - P_curr
    current_dist = np.dot(vec_curr_p1, n_vec)
    
    # C. Calculate required straight length L
    # We fly along vector u (psi_int).
    # How much does 'u' move us along the normal 'n'? 
    # rate = dot(u, n).
    # We need to cover (target_dist - current_dist).
    # L * rate = target_dist - current_dist
    u_vec = np.array([math.cos(psi_int), math.sin(psi_int)])
    closing_rate = np.dot(u_vec, n_vec)
    
    # Avoid division by zero (unlikely with 45 deg offset)
    if abs(closing_rate) < 1e-5:
        L = 0
    else:
        L = (target_dist - current_dist) / closing_rate
        
    # If L is negative, it means Arc 1 overshot TOO FAR for the required return geometry.
    # In real autopilots, this means the turn radius was too large for the corner angle.
    # We clamp to 0, which results in a slight "snap" but preserves connectivity.
    if L < 0: 
        L = 0

    # D. Calculate P2 (Start of Arc 2)
    P2_start = P1_end + L * u_vec
    
    # =========================================================
    # ARC 2: The Recovery Turn (P2 -> Tangent)
    # =========================================================
    # Turn Direction is OPPOSITE to the overshoot (-turn_dir)
    recovery_dir = -turn_dir
    
    # Center C2 is perpendicular to heading at P2 (psi_int)
    # Angle to C2 = psi_int + (recovery_dir * 90)
    c2_angle = psi_int + (recovery_dir * math.pi / 2)
    C2 = P2_start + r_min_meters * np.array([math.cos(c2_angle), math.sin(c2_angle)])
    
    # Start angle on C2 (from Center to P2)
    ang_s2 = c2_angle - math.pi
    
    # End angle on C2 (Tangent to psi_out)
    # Angle of radius = Tangent - (turn_dir * 90)
    # Target tangent is psi_out. Turn is recovery_dir.
    ang_e2 = psi_out - (recovery_dir * math.pi / 2)
    
    arc2_pts_m = generate_arc_points(C2, r_min_meters, ang_s2, ang_e2, recovery_dir)
    
    # =========================================================
    # COMBINE & UNPROJECT
    # =========================================================
    
    # Straight leg points (add start of P2 so we draw the line from P1 to P2)
    straight_pts_m = [tuple(P1_end), tuple(P2_start)]
    
    # Full list
    full_path_m = arc1_pts_m + straight_pts_m[1:] + arc2_pts_m
    
    # Convert back to Lat/Lon
    final_path_ll = []
    alt = p_curr[2] if len(p_curr) > 2 else 0
    
    for pm in full_path_m:
        lat, lon = meters_to_latlon(pm[0], pm[1], ref_lat, ref_lon)
        final_path_ll.append((lat, lon, alt))
        
    return final_path_ll