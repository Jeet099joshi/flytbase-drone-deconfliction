import math 
import matplotlib.pyplot as plt 
 
# ============================================================ 
# UAV STRATEGIC DECONFLICTION SYSTEM (2D) 
# ============================================================ 
# This module performs pre-flight strategic deconfliction 
# by checking spatial and temporal conflicts between 
# multiple UAV waypoint trajectories in 2D space. 
# ============================================================ 
 
# ------------------------------------------------------------ 
# Compute Euclidean distance between two 2D points 
# ------------------------------------------------------------ 
def distance_2d(p1, p2): 
    """ 
    Returns the Euclidean distance between two points (x, y). 
    """ 
    return math.sqrt( 
        (p1[0] - p2[0]) ** 2 + 
        (p1[1] - p2[1]) ** 2 
    ) 
 
# ------------------------------------------------------------ 
# Sort waypoints in ascending order of time 
# ------------------------------------------------------------ 
def sort_path_by_time(path): 
    """ 
    Ensures the waypoint sequence is temporally ordered. 
    """ 
    return sorted(path, key=lambda p: p[2]) 
 
# ------------------------------------------------------------ 
# Linear interpolation between two waypoints in 2D 
# ------------------------------------------------------------ 
def interpolate(p_start, p_end, t): 
    """ 
    Computes interpolated (x, y) position at time t 
    between two waypoints. 
    """ 
    t1, t2 = p_start[2], p_end[2] 
 
    # Prevent division by zero for identical timestamps 
    if t2 == t1: 
        return (p_start[0], p_start[1]) 
 
    ratio = (t - t1) / (t2 - t1) 
 
    x = p_start[0] + ratio * (p_end[0] - p_start[0]) 
    y = p_start[1] + ratio * (p_end[1] - p_start[1]) 
 
    return (x, y) 
 
# ------------------------------------------------------------ 
# Determine drone position at a specific time 
# ------------------------------------------------------------ 
def get_position_at_time(path, t): 
    """ 
    Returns the interpolated drone position at time t. 
    """ 
    for i in range(len(path) - 1): 
        if path[i][2] <= t <= path[i + 1][2]: 
            return interpolate(path[i], path[i + 1], t) 
    return None 
 
# ------------------------------------------------------------ 
# Perform pairwise deconfliction across all drones 
# ------------------------------------------------------------ 
def check_all_paths_conflict(all_drones_paths, safety_distance, time_step): 
    """ 
    Checks spatial and temporal conflicts between all drone pairs. 
    """ 
    drone_names = list(all_drones_paths.keys()) 
    conflicts = [] 
 
    # Pairwise comparison of drone trajectories 
    for i in range(len(drone_names)): 
        for j in range(i + 1, len(drone_names)): 
 
            d1, d2 = drone_names[i], drone_names[j] 
            path1 = sort_path_by_time(all_drones_paths[d1]) 
            path2 = sort_path_by_time(all_drones_paths[d2]) 
 
            # Ignore drones with insufficient trajectory data 
            if len(path1) < 2 or len(path2) < 2: 
                continue 
 
            # Determine overlapping mission window 
            start_time = max(path1[0][2], path2[0][2]) 
            end_time = min(path1[-1][2], path2[-1][2]) 
 
            mission_duration = end_time - start_time 
            if mission_duration <= 0: 
                continue 
 
            # Adaptive temporal sampling 
            effective_step = time_step 
            if effective_step >= mission_duration: 
                effective_step = mission_duration / 20 
 
            if effective_step <= 0: 
                continue 
 
            t = start_time + effective_step 
 
            # Sample positions over time window 
            while t <= end_time: 
                pos1 = get_position_at_time(path1, t) 
                pos2 = get_position_at_time(path2, t) 
 
                if pos1 and pos2: 
                    dist = distance_2d(pos1, pos2) 
 
                    # Conflict condition based on safety buffer 
                    if dist <= safety_distance: 
                        conflicts.append({ 
                            "time": round(t, 2), 
                            "location": (round(pos1[0], 2), round(pos1[1], 
2)), 
                            "distance": round(dist, 2), 
                            "between": (d1, d2) 
                        }) 
 
                t += effective_step 
 
    if conflicts: 
        return {"status": "CONFLICT", "conflicts": conflicts} 
    return {"status": "CLEAR"} 
 
# ------------------------------------------------------------ 
# Collect waypoint input for a drone 
# ------------------------------------------------------------ 
def get_drone_path(drone_name): 
    """ 
    Reads waypoint input for a single drone. 
    """ 
    path = [] 
    points = int(input(f"\nEnter number of waypoints for {drone_name}: ")) 
 
    if points < 2: 
        print(f"Warning: Drone '{drone_name}' has insufficient waypoints.") 
 
    for i in range(points): 
        print(f"Waypoint {i + 1}:") 
        x = float(input("  x: ")) 
        y = float(input("  y: ")) 
        t = float(input("  time: ")) 
        path.append((x, y, t)) 
 
    return path 
 
# ------------------------------------------------------------ 
# 2D Visualization of drone trajectories and conflicts 
# ------------------------------------------------------------ 
def plot_paths_2d(all_drones_paths, result): 
    """ 
    Plots all drone paths and highlights conflict locations. 
    """ 
    plt.figure(figsize=(8, 7)) 
 
    for drone_name, path in all_drones_paths.items(): 
        if len(path) < 2: 
            continue 
 
        path = sort_path_by_time(path) 
        x = [p[0] for p in path] 
        y = [p[1] for p in path] 
 
        plt.plot(x, y, marker='o', label=drone_name) 
 
    # Highlight conflict points 
    if result["status"] == "CONFLICT": 
        for c in result["conflicts"]: 
            cx, cy = c["location"] 
            plt.scatter(cx, cy, color='red', s=90) 
 
    plt.xlabel("X Position") 
    plt.ylabel("Y Position") 
    plt.title("UAV Strategic Deconfliction (2D)") 
    plt.legend() 
    plt.grid(True) 
    plt.show() 
 
# ------------------------------------------------------------ 
# Main execution flow 
# ------------------------------------------------------------ 
if __name__ == "__main__": 
 
    all_drones_paths = {} 
 
    num_drones = int(input("Enter number of drones: ")) 
 
    for i in range(num_drones): 
        name = input(f"\nEnter name for Drone {i + 1}: ") 
        all_drones_paths[name] = get_drone_path(name) 
 
    SAFETY_DISTANCE = float(input("\nEnter safety distance (meters): ")) 
    TIME_STEP = float(input("Enter time step for checking (seconds): ")) 
 
    result = check_all_paths_conflict( 
        all_drones_paths, 
        SAFETY_DISTANCE, 
        TIME_STEP 
    ) 
 
    print("\n--- RESULT ---") 
    if result["status"] == "CONFLICT": 
        print(f"{len(result['conflicts'])} CONFLICT(S) DETECTED\n") 
        for idx, c in enumerate(result["conflicts"], 1): 
            print( 
                f"{idx}. Time: {c['time']} | " 
                f"Location: {c['location']} | " 
                f"Distance: {c['distance']} m | " 
                f"Between: {c['between']}" 
            ) 
    else: 
        print("NO CONFLICT ALL PATHS SAFE") 
 
    plot_paths_2d(all_drones_paths, result) 