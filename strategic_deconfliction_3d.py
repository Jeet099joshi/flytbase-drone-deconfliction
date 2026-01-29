import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ============================================================
# UAV STRATEGIC DECONFLICTION SYSTEM (3D / 4D)
# ============================================================
# This module performs pre-flight strategic deconfliction
# by checking spatial and temporal conflicts between
# multiple UAV trajectories in 3D space with time.
# Each trajectory is defined as (x, y, z, time).
# ============================================================


# ------------------------------------------------------------
# Compute Euclidean distance between two 3D points
# ------------------------------------------------------------
def distance_3d(p1, p2):
    """
    Returns the Euclidean distance between two points (x, y, z).
    """
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


# ------------------------------------------------------------
# Sort waypoints in ascending order of time
# ------------------------------------------------------------
def sort_path_by_time(path):
    """
    Ensures that the waypoint sequence is temporally ordered.
    """
    return sorted(path, key=lambda p: p[3])


# ------------------------------------------------------------
# Linear interpolation between two 3D waypoints
# ------------------------------------------------------------
def interpolate(p_start, p_end, t):
    """
    Computes the interpolated (x, y, z) position at time t
    between two consecutive waypoints.
    """
    t1, t2 = p_start[3], p_end[3]

    # Prevent division by zero for identical timestamps
    if t2 == t1:
        return (p_start[0], p_start[1], p_start[2])

    ratio = (t - t1) / (t2 - t1)

    x = p_start[0] + ratio * (p_end[0] - p_start[0])
    y = p_start[1] + ratio * (p_end[1] - p_start[1])
    z = p_start[2] + ratio * (p_end[2] - p_start[2])

    return (x, y, z)


# ------------------------------------------------------------
# Determine drone position at a specific time
# ------------------------------------------------------------
def get_position_at_time(path, t):
    """
    Returns the interpolated drone position at time t
    along the given trajectory.
    """
    for i in range(len(path) - 1):
        if path[i][3] <= t <= path[i + 1][3]:
            return interpolate(path[i], path[i + 1], t)
    return None


# ------------------------------------------------------------
# Perform pairwise deconfliction across all drones
# ------------------------------------------------------------
def check_all_paths_conflict(all_drones_paths, safety_distance, time_step):
    """
    Checks spatial and temporal conflicts between all pairs
    of drone trajectories.
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
            start_time = max(path1[0][3], path2[0][3])
            end_time = min(path1[-1][3], path2[-1][3])

            mission_duration = end_time - start_time
            if mission_duration <= 0:
                continue

            # Adaptive temporal sampling to avoid missed conflicts
            effective_step = time_step
            if effective_step >= mission_duration:
                effective_step = mission_duration / 20

            if effective_step <= 0:
                continue

            t = start_time + effective_step

            # Sample positions over overlapping time window
            while t <= end_time:
                pos1 = get_position_at_time(path1, t)
                pos2 = get_position_at_time(path2, t)

                if pos1 and pos2:
                    dist = distance_3d(pos1, pos2)

                    # Conflict condition based on safety buffer
                    if dist <= safety_distance:
                        conflicts.append({
                            "time": round(t, 2),
                            "location": tuple(round(v, 2) for v in pos1),
                            "distance": round(dist, 2),
                            "between": (d1, d2)
                        })

                t += effective_step

    if conflicts:
        return {"status": "CONFLICT", "conflicts": conflicts}
    return {"status": "CLEAR"}


# ------------------------------------------------------------
# Collect waypoint input for a drone (3D)
# ------------------------------------------------------------
def get_drone_path(drone_name):
    """
    Reads waypoint input for a single drone trajectory.
    """
    path = []
    points = int(input(f"\nEnter number of waypoints for {drone_name}: "))

    if points < 2:
        print(f"Drone '{drone_name}' has insufficient waypoints.")

    for i in range(points):
        print(f"Waypoint {i + 1}:")
        x = float(input("  x: "))
        y = float(input("  y: "))
        z = float(input("  z (altitude): "))
        t = float(input("  time: "))
        path.append((x, y, z, t))

    return path


# ------------------------------------------------------------
# 3D visualization of drone trajectories and conflicts
# ------------------------------------------------------------
def plot_paths_3d(all_drones_paths, result):
    """
    Plots all drone trajectories in 3D space and highlights
    detected conflict points.
    """
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    for drone_name, path in all_drones_paths.items():
        if len(path) < 2:
            continue

        path = sort_path_by_time(path)
        x = [p[0] for p in path]
        y = [p[1] for p in path]
        z = [p[2] for p in path]

        ax.plot(x, y, z, marker='o', label=drone_name)

    # Highlight conflict locations
    if result["status"] == "CONFLICT":
        for c in result["conflicts"]:
            cx, cy, cz = c["location"]
            ax.scatter(cx, cy, cz, color='red', s=90)

    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Altitude (Z)")
    ax.set_title("UAV Strategic Deconfliction (3D)")
    ax.legend()
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
        print("NO CONFLICT – ALL PATHS SAFE")

    plot_paths_3d(all_drones_paths, result)
