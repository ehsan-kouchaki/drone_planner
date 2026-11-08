import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import CubicSpline

# =============================================================================
# 1. HELPER FUNCTIONS (GEOMETRY & PROJECTION)
# =============================================================================

def latlon_to_meters(lat, lon, lat_ref, lon_ref):
    """Simple flat-earth projection for small areas."""
    R = 6378137  # Earth radius
    x = R * math.radians(lon - lon_ref) * math.cos(math.radians(lat_ref))
    y = R * math.radians(lat - lat_ref)
    return x, y

def generate_arc_points(center, r, ang_start, ang_end, direction, num_points=50):
    """
    Generates points along an arc.
    direction: +1 for Counter-Clockwise (Left), -1 for Clockwise (Right)
    """
    # Normalize angles to [0, 2pi) for easier comparison, or handle wrapping
    # We ensure the traversal matches the direction
    
    if direction == 1: # CCW
        # If end < start, we cross 0/2pi boundary
        if ang_end < ang_start:
            ang_end += 2 * math.pi
    else: # CW
        # If end > start, we cross 0/2pi boundary
        if ang_end > ang_start:
            ang_end -= 2 * math.pi
            
    # Ensure we don't generate a full circle if angles are theoretically equal
    if abs(ang_end - ang_start) < 1e-6:
        return []

    angles = np.linspace(ang_start, ang_end, num_points)
    points = []
    for a in angles:
        px = center[0] + r * math.cos(a)
        py = center[1] + r * math.sin(a)
        points.append((px, py))
    return points

# =============================================================================
# 2. SMOOTHING ALGORITHMS
# =============================================================================

def method_spline(points_m):
    """Cubic Spline Interpolation."""
    points_m = np.array(points_m)
    x = points_m[:, 0]
    y = points_m[:, 1]
    
    # Parameter t based on distance
    dist = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    if np.sum(dist) == 0: return points_m
    
    t = np.concatenate(([0], np.cumsum(dist)))
    
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    
    t_fine = np.linspace(0, t[-1], 500)
    return list(zip(cs_x(t_fine), cs_y(t_fine)))

def method_dubins_fillet(points_m, r_min):
    """
    Standard 'Dubins-like' Fillet smoothing. 
    Turns EARLY to inscribe a circle of radius R_min tangent to incoming/outgoing legs.
    """
    path = []
    path.append(points_m[0]) # Start
    
    for i in range(1, len(points_m) - 1):
        p_prev = np.array(points_m[i-1])
        p_curr = np.array(points_m[i])
        p_next = np.array(points_m[i+1])
        
        v1 = p_curr - p_prev
        v2 = p_next - p_curr
        
        # Length check
        len_v1 = np.linalg.norm(v1)
        len_v2 = np.linalg.norm(v2)
        if len_v1 == 0 or len_v2 == 0: continue

        psi1 = math.atan2(v1[1], v1[0])
        psi2 = math.atan2(v2[1], v2[0])
        
        # Turn direction
        angle_diff = (psi2 - psi1 + math.pi) % (2 * math.pi) - math.pi
        turn_dir = 1 if angle_diff > 0 else -1
        
        # Half angle of the turn
        alpha = abs(angle_diff) / 2.0
        if alpha < 1e-3: # Straight line
            path.append(tuple(p_curr))
            continue
            
        # Distance from vertex to tangent point (Geometric definition of Fillet)
        d_tan = r_min * math.tan(alpha / 2.0) # Note: tan(theta/2) relation
        
        # Safety Check: Don't start turn before the previous point
        limit = min(len_v1, len_v2) / 2.0
        actual_d_tan = min(d_tan, limit)
        # If we clamp d_tan, we must adjust radius effectively (not implemented here for simplicity of pure Dubins comparison)
        
        # Tangent start/end points
        p_tan_start = p_curr - (v1 / len_v1) * d_tan
        p_tan_end = p_curr + (v2 / len_v2) * d_tan
        
        # Circle Center Calculation
        # Center is perpendicular to the incoming velocity v1 at the tangent point
        # Direction: psi1 + 90 deg (if Left Turn)
        norm_angle = psi1 + (turn_dir * math.pi / 2)
        center = p_tan_start + r_min * np.array([math.cos(norm_angle), math.sin(norm_angle)])
        
        # Arc Angles (Vector from Center -> Point)
        # Start Point Angle: opposite to normal vector
        ang_start = norm_angle + math.pi 
        
        # End Point Angle: Tangent is psi2. Center is perpendicular.
        # Vector Center->End is (psi2 + turn_dir*90 + 180)
        ang_end = psi2 + (turn_dir * math.pi / 2) + math.pi
        
        # Generate Arc
        arc = generate_arc_points(center, r_min, ang_start, ang_end, turn_dir)
        
        # Add straight segment from previous point to tangent start
        path.append(tuple(p_tan_start))
        path.extend(arc)
        path.append(tuple(p_tan_end))
        
    path.append(points_m[-1]) # End
    return path

def method_our_nfz_bubble(points_m, r_min):
    """
    'Our Method': Approximated geometric equivalent.
    In the paper, A* creates a path *around* a bubble at the vertex.
    This typically results in an arc connecting tangent lines on the bubble surface.
    """
    path = []
    path.append(points_m[0])
    
    for i in range(1, len(points_m) - 1):
        p_prev = np.array(points_m[i-1])
        p_curr = np.array(points_m[i])
        p_next = np.array(points_m[i+1])
        
        v_in = p_curr - p_prev
        v_out = p_next - p_curr
        
        u_in = v_in / np.linalg.norm(v_in)
        u_out = v_out / np.linalg.norm(v_out)
        u_in_rev = -u_in
        
        # Bisector vector
        bisector = u_in_rev + u_out
        norm_bi = np.linalg.norm(bisector)
        if norm_bi < 1e-6:
            path.append(tuple(p_curr))
            continue
            
        u_bisector = bisector / norm_bi
        
        # "Our Method" places the circle so the vertex P_curr is on the perimeter
        center = p_curr + r_min * u_bisector
        
        # Determine turn direction
        psi_in = math.atan2(v_in[1], v_in[0])
        psi_out = math.atan2(v_out[1], v_out[0])
        angle_diff = (psi_out - psi_in + math.pi) % (2 * math.pi) - math.pi
        turn_dir = 1 if angle_diff > 0 else -1
        
        # The path hugs the "outside" of this circle.
        # Tangent angle on the circle is perpendicular to radius.
        # Incoming: Tangent is psi_in. Radius angle = psi_in - turn_dir * 90
        rad_ang_in = psi_in - (turn_dir * math.pi / 2)
        
        # Outgoing: Tangent is psi_out. Radius angle = psi_out - turn_dir * 90
        rad_ang_out = psi_out - (turn_dir * math.pi / 2)
        
        # Note: "Our Method" usually produces a bulge because the circle is further out
        arc = generate_arc_points(center, r_min, rad_ang_in, rad_ang_out, turn_dir)
        
        if arc:
            path.append(tuple(arc[0]))
            path.extend(arc)
        
    path.append(points_m[-1])
    return path

def method_overfly_robust(points_m, r_min):
    """
    Robust Overfly S-Turn with exact tangential re-entry calculation.
    """
    path = []
    path.append(points_m[0])
    
    for i in range(1, len(points_m) - 1):
        p_prev = np.array(points_m[i-1])
        p_curr = np.array(points_m[i])
        p_next = np.array(points_m[i+1])
        
        v_in = p_curr - p_prev
        v_out = p_next - p_curr
        psi_in = math.atan2(v_in[1], v_in[0])
        psi_out = math.atan2(v_out[1], v_out[0])
        
        angle_diff = (psi_out - psi_in + math.pi) % (2 * math.pi) - math.pi
        turn_dir = 1 if angle_diff > 0 else -1
        
        # 1. Overshoot Heading (45 deg)
        intercept_offset = math.radians(45)
        psi_int = psi_out + (turn_dir * intercept_offset)
        
        # 2. Arc 1: P_curr -> P1 (Overshoot)
        # Center C1 is perpendicular to incoming path at P_curr
        c1_angle = psi_in + (turn_dir * math.pi / 2)
        c1 = p_curr + r_min * np.array([math.cos(c1_angle), math.sin(c1_angle)])
        
        ang_start_1 = c1_angle - math.pi
        ang_end_1 = psi_int - (turn_dir * math.pi / 2)
        
        arc1 = generate_arc_points(c1, r_min, ang_start_1, ang_end_1, turn_dir)
        if not arc1: 
            path.append(tuple(p_curr))
            continue
            
        p1_end = np.array(arc1[-1])
        
        # 3. Calculate Straight Leg Length (L)
        # We need to fly vector u for length L to reach point P2, where Arc 2 starts.
        # Condition: Circle C2 (radius R, -turn_dir) starting at P2 is tangent to output line.
        u_vec = np.array([math.cos(psi_int), math.sin(psi_int)])
        
        # Normal to the outgoing line (pointing to the side we are approaching from)
        # We overshot in 'turn_dir', so we are on that side.
        n_angle = psi_out + (turn_dir * math.pi / 2)
        n_vec = np.array([math.cos(n_angle), math.sin(n_angle)])
        
        # Vector from P2 to C2 (perpendicular to heading psi_int, dir -turn_dir)
        # Angle = psi_int - (-turn_dir * 90) = psi_int + turn_dir*90
        to_c2_angle = psi_int + (turn_dir * math.pi / 2) # ERROR IN CHAT, FIX: Opposite turn is -turn_dir
        # Wait. Arc 2 turns Opposite. If Arc 1 was Left (+1), Arc 2 is Right (-1).
        # Center of Right turn is -90 relative to heading.
        # psi_int + (-1 * 90) = psi_int - 90.
        # General: psi_int + (-turn_dir * pi/2).
        to_c2_angle = psi_int - (turn_dir * math.pi / 2) 
        to_c2_vec = np.array([math.cos(to_c2_angle), math.sin(to_c2_angle)])

        # Solve: (P1 + L*u + R*to_c2 - P_curr) dot n_vec = -R (since C2 is on 'inside' of return turn)
        # Wait, if we turn Right to merge from Left, the center C2 is to the Right of the path.
        # If 'n_vec' points Left, C2 is at distance -R along n_vec relative to the line.
        # Equation: dot(C2 - P_curr, n_vec) = -r_min ?
        # Let's verify signs.
        # Left overshoot (turn_dir=1). n_vec points Left.
        # We return by turning Right. C2 is to the Right of the tangent/line.
        # So C2 is on the "Right" side of the output line.
        # n_vec points "Left". So dot product should be negative. Correct.
        
        numerator = -r_min - np.dot((p1_end - p_curr), n_vec) - r_min * np.dot(to_c2_vec, n_vec)
        denominator = np.dot(u_vec, n_vec)
        
        if abs(denominator) < 1e-6:
            L = 0
        else:
            L = numerator / denominator
            if L < 0: L = 0 # Geometry too tight
            
        p2_start = p1_end + L * u_vec
        
        # 4. Arc 2: P2 -> P_tangent (Recovery)
        c2 = p2_start + r_min * to_c2_vec
        ang_start_2 = to_c2_angle + math.pi # From Center to P2
        
        # End at tangent to outgoing.
        # Tangent is psi_out. Turn is -turn_dir.
        # Radius angle = psi_out - (-turn_dir * 90) = psi_out + turn_dir*90
        ang_end_2 = psi_out + (turn_dir * math.pi / 2)
        
        arc2 = generate_arc_points(c2, r_min, ang_start_2, ang_end_2, -turn_dir)
        
        # Assemble
        path.extend(arc1)
        path.append(tuple(p2_start))
        path.extend(arc2)
        
    path.append(points_m[-1])
    return path

# =============================================================================
# 3. MAIN EXECUTION & PLOTTING
# =============================================================================

# 1. Define coordinates (Lat, Lon) - Creating a Zig-Zag path
waypoints_ll = [
    (37.3933610, -6.0272694), # P1
    (37.3929637, -6.0502296), # P2 (Corner)
    (37.4162931, -6.0641661), # P3 (Corner)
    (37.3996759, -6.0583849), # P4 (Corner)
    (37.4043908, -6.0769638),  # P5
    (37.4050127, -6.0974208),
    (37.42, -6.0974208) 
]

# 2. Project to Meters
ref_lat, ref_lon = waypoints_ll[0]
points_m = [latlon_to_meters(lat, lon, ref_lat, ref_lon) for lat, lon in waypoints_ll]
print(points_m)
# 3. Run Methods
R_MIN = 87.0 # Meters

# A. Straight
path_straight = points_m

# B. Spline
path_spline = method_spline(points_m)

# C. Dubins (Fillet)
path_dubins = method_dubins_fillet(points_m, R_MIN)

# D. Our Method
path_our = method_our_nfz_bubble(points_m, R_MIN)

# E. Overfly
path_overfly = method_overfly_robust(points_m, R_MIN)

# 4. Plot
plt.figure(figsize=(12, 8))

def get_xy(path):
    return [p[0] for p in path], [p[1] for p in path]

# Plot Straight
x, y = get_xy(path_straight)
plt.plot(x, y, 'k--', label='Original (VRP)', linewidth=1.5, alpha=0.4, marker='x')

# Plot Spline
#x, y = get_xy(path_spline)
#plt.plot(x, y, 'm-', label='Spline', linewidth=2, alpha=0.7)

# Plot Dubins
#x, y = get_xy(path_dubins)
#plt.plot(x, y, 'b-', label="Dubins (Fillet)", linewidth=2)

# Plot Our Method
#x, y = get_xy(path_our)
#plt.plot(x, y, 'g-', label='Our Method (Artificial NFZ)', linewidth=2.5)

# Plot Overfly
x, y = get_xy(path_overfly)
plt.plot(x, y, 'r-', label='Overfly (S-Turn)', linewidth=2)

# Styling
plt.title("Comparison of Path Smoothing Methods", fontsize=16)
plt.xlabel("Local X (meters)", fontsize=12)
plt.ylabel("Local Y (meters)", fontsize=12)
plt.legend(fontsize=12, loc='best')
plt.grid(True, linestyle=':', alpha=0.6)
plt.axis('equal')

# Add labels to vertices
for i, p in enumerate(points_m):
    plt.text(p[0], p[1]+5, f"P{i+1}", fontsize=12, fontweight='bold')

plt.tight_layout()
plt.show()