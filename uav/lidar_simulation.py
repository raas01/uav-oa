import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def simulate_3d_lidar_data(num_readings=360, max_distance=10.0, max_elevation=90):
    """
    Simulates 3D Lidar data with random distances and angles.
    """
    distances = np.random.uniform(low=0.0, high=max_distance, size=num_readings)
    azimuth_angles = np.random.uniform(low=0.0, high=360, size=num_readings)
    elevation_angles = np.random.uniform(low=-max_elevation, high=max_elevation, size=num_readings)

    return distances, azimuth_angles, elevation_angles

def plot_3d_lidar_data(distances, azimuth_angles, elevation_angles, max_distance=10.0):
    """
    Plots the simulated 3D Lidar data.
    """
    azimuth_radians = np.radians(azimuth_angles)
    elevation_radians = np.radians(elevation_angles)

    x = distances * np.sin(elevation_radians) * np.cos(azimuth_radians)
    y = distances * np.sin(elevation_radians) * np.sin(azimuth_radians)
    z = distances * np.cos(elevation_radians)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)
    ax.set_title("Simulated 3D Lidar Data")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([-max_distance, max_distance])
    ax.set_ylim([-max_distance, max_distance])
    ax.set_zlim([-max_distance, max_distance])
    plt.show()

# Simulate 3D Lidar data
distances, azimuth_angles, elevation_angles = simulate_3d_lidar_data()

# Plot the simulated 3D Lidar data
plot_3d_lidar_data(distances, azimuth_angles, elevation_angles)
