import numpy as np
import matplotlib.pyplot as plt

waypoints = None
reached_goal = False
travelled_path = []
goal_state = None


def calculate_avoidance_curve(current_position, obstacle_position, lidar_radius):
    obstacle_vector = obstacle_position - current_position
    theta = np.linspace(0, np.pi, 10)
    semi_circle_radius = lidar_radius * 1.5
    avoidance_curve = np.zeros((len(theta), 3))

    for i in range(len(theta)):
        avoidance_curve[i, :] = (
            current_position
            + semi_circle_radius * np.array([np.cos(theta[i]), np.sin(theta[i]), 0])
            + obstacle_vector * 0.5
        )
    return avoidance_curve


def is_curve_safe(curve, obstacles, safe_distance):
    for point in curve:
        for obstacle in obstacles:
            if np.linalg.norm(point - obstacle) < safe_distance:
                return False
    return True


def advanced_obstacle_avoidance(
    current_position,
    target_position,
    obstacles,
    lidar_radius,
    drone_radius,
    safety_margin,
):
    global waypoints, goal
    distances_to_obstacles = np.sqrt(
        np.sum((obstacles - current_position) ** 2, axis=1)
    )
    potential_collisions = np.where(distances_to_obstacles < lidar_radius)[0]
    adjusted = False
    new_position = current_position

    if potential_collisions.size > 0:
        for collision_index in potential_collisions:
            obstacle_pos = obstacles[collision_index, :]
            avoidance_curve = calculate_avoidance_curve(
                current_position, obstacle_pos, lidar_radius
            )

            if is_curve_safe(avoidance_curve, obstacles, drone_radius + safety_margin):
                new_position = avoidance_curve[-1, :]
                adjusted = True
                break
            
        
        # recalculate and update path to goal from new avoided position
        num_points = 100
        x = np.linspace(new_position[0], goal_state[0], num_points)
        y = np.linspace(new_position[1], goal_state[1], num_points)
        z = np.linspace(new_position[2], goal_state[2], num_points)
        waypoints = np.column_stack((x, y, z))

    if not adjusted:
        new_position = target_position

    return new_position, adjusted


# Main function
def uav_smart():
    global waypoints, reached_goal, goal_state
    start_state = np.array([0, 0, 0, np.deg2rad(0)])
    goal_state = np.array([50, 50, 10, np.deg2rad(45)])

    num_obstacles = 70
    obstacle_radius = 1
    lidar_radius = 5
    drone_radius = 1
    safety_margin = 1

    obstacle_points = np.random.rand(num_obstacles, 3) * 50
    deterministic_obstacles = np.array([[25, 25, 5], [30, 30, 7]])
    obstacle_points = np.vstack((obstacle_points, deterministic_obstacles))

    num_points = 100
    x = np.linspace(start_state[0], goal_state[0], num_points)
    y = np.linspace(start_state[1], goal_state[1], num_points)
    z = np.linspace(start_state[2], goal_state[2], num_points)
    waypoints = np.column_stack((x, y, z))

    # Visualization setup
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    (drone_plot,) = ax.plot(
        [start_state[0]], [start_state[1]], [start_state[2]], "ro", markersize=10
    )
    (traveled_plot,) = ax.plot(
        [start_state[0]], [start_state[1]], [start_state[2]], "r-"
    )

    ax.scatter(
        obstacle_points[:, 0],
        obstacle_points[:, 1],
        obstacle_points[:, 2],
        c="k",
        marker="x",
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Drone Path Animation")
    plt.grid(True)
    plt.axis("equal")

    # Main loop for path animation
    drone_position = start_state[:3]
    xdata, ydata, zdata = [start_state[0]], [start_state[1]], [start_state[2]]

    while not reached_goal and waypoints.size > 0:
        waypoint = waypoints[0]  # Current waypoint
        drone_position, _ = advanced_obstacle_avoidance(
            drone_position,
            waypoint,
            obstacle_points,
            lidar_radius,
            drone_radius,
            safety_margin,
        )

        # Update drone plot
        drone_plot.set_data(drone_position[0], drone_position[1])
        drone_plot.set_3d_properties(drone_position[2])

        # Update traveled path
        xdata.append(drone_position[0])
        ydata.append(drone_position[1])
        zdata.append(drone_position[2])
        traveled_plot.set_data(xdata, ydata)
        traveled_plot.set_3d_properties(zdata)

        # Check if current waypoint is reached
        if np.linalg.norm(drone_position - waypoint) < 0.5:
            waypoints = waypoints[1:]  # Remove the reached waypoint

        # Check if goal is reached
        if waypoints.size == 0:
            reached_goal = True

        plt.pause(0.05)

    plt.show()


if __name__ == "__main__":
    uav_smart()
