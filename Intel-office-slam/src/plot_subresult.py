import numpy as np
import matplotlib.pyplot as plt

def create_file(odometry, means, laser_data, odometry_filename = "odometry.txt", estpose_filename = "pose.txt", laser_data_filename = "laser_data.txt"):
        # Save odometry data
        odom_pose = np.zeros(3)
        with open(odometry_filename, "w") as odom_file:
            for odom_data in odometry:
                # Assuming odom is a numpy array or tuple with numeric values
                odom_pose[0] = odom_pose[0] + odom_data[0] * np.cos(odom_pose[2]) - odom_data[1] * np.sin(odom_pose[2])
                odom_pose[1] = odom_pose[1] + odom_data[1] * np.cos(odom_pose[2]) + odom_data[0] * np.sin(odom_pose[2])
                odom_pose[2] = odom_pose[2] + odom_data[2]
                odom_pose[2] = (odom_pose[2] + np.pi) % (2 * np.pi) - np.pi
                odom_file.write(f"{odom_pose[0]:.6f} {odom_pose[1]:.6f} {odom_pose[2]:.6f}\n")

        # Save pose data (x, y, theta)
        with open(estpose_filename, "w") as pose_file:
            for pose in means:
                # Assuming pose is a numpy array or tuple with at least 3 elements
                pose[2] = (pose[2] + np.pi) % (2 * np.pi) - np.pi
                pose_file.write(f"{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f}\n")

        with open(laser_data_filename, "w") as laser_file:
            for laser_data in laser_data:
                laser_file.write(f"{laser_data[0]:.6f} {laser_data[1]:.6f} {laser_data[2]:.6f}\n")

def plot_results_from_file(odom_file="odometry.txt", pose_file="pose.txt"):
    """
    Reads odometry and pose data from files and plots the trajectories.

    Parameters:
    - odom_file: Path to the file containing odometry data (default: "odometry.txt").
    - pose_file: Path to the file containing pose data (default: "pose.txt").
    """
    # Load odometry data
    try:
        odometry_data = np.loadtxt(odom_file)
        odom_x, odom_y = odometry_data[:, 0], odometry_data[:, 1]
    except Exception as e:
        print(f"Error reading odometry data from {odom_file}: {e}")
        return

    # Load pose data
    try:
        pose_data = np.loadtxt(pose_file)
        pose_x, pose_y = pose_data[:, 0], pose_data[:, 1]
    except Exception as e:
        print(f"Error reading pose data from {pose_file}: {e}")
        return
    plt.ioff()
    # Plot trajectories
    plt.figure(figsize=(10, 6))
    plt.plot(odom_x, odom_y, label="Odometry Trajectory", marker='o')
    plt.plot(pose_x, pose_y, label="Pose Data", marker='x', linestyle='--')

    # Add labels, legend, and grid
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Odometry and Pose Data Trajectory")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def plot_map_from_file(pose_file, laser_data_file):
    laser_points = [] 