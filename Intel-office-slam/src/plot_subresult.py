import numpy as np
import matplotlib.pyplot as plt
from mapping import Mapper

def create_file(odometry, means, pointclouds, odometry_filename = "odometry.txt", estpose_filename = "pose.txt", laser_data_filename = "laser_data.txt"):
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
            for pc in pointclouds:
                for point in pc:
                    laser_file.write(f"{point[0]:.6f} {point[1]:.6f}, ")
                laser_file.write("\n")

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

def plot_map_from_file(pose_file, laser_data_file, start_index=0, end_index=-1):
    laser_data = []
    length = 0
    
    pose_data = np.loadtxt(pose_file)
    pose_x, pose_y, pose_theta = pose_data[:, 0], pose_data[:, 1], pose_data[:, 2]

    with open(laser_data_file, "r") as laser_file: 

        for i, line in enumerate(laser_file):
            if i > len(pose_x)-1:
                break
            points = line.split(", ")[:-1]
            length += len(points) 

            px, py, ptheta = pose_x[i], pose_y[i], pose_theta[i]
            x_lst = np.zeros(len(points))
            y_lst = np.zeros(len(points))
            pointcloud = np.zeros((2, len(points)))
            for j, point in enumerate(points):
                x, y = float(point.split(" ")[0]), float(point.split(" ")[1])
                # Transform using pose_frame
                x_lst[j] = x  # Store original points
                y_lst[j] = y
                
                # Correct transformation from robot frame to global frame
                x_global = px + x * np.cos(ptheta) - y * np.sin(ptheta)
                y_global = py + y * np.cos(ptheta) + x * np.sin(ptheta)  # Fixed: was using x instead of y in first term

                pointcloud[0, j] = x_global
                pointcloud[1, j] = y_global
            if i == -1:
                print(px, py, ptheta)
                pointcloud_max = np.max(np.max(pointcloud, axis=1))

                plt.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
                plt.scatter(x_lst, y_lst, marker='.')
                plt.show()

                plt.scatter(pointcloud[0, :], pointcloud[1, :], marker='.')
                plt.arrow(px, py, np.cos(ptheta), np.sin(ptheta), head_width=0.1, head_length=0.1, fc='blue', ec='blue')
                plt.show()
            laser_data.append(pointcloud)

    lidar_points = np.zeros((2, length))
    last = 0
    
    cum_length = [0]

    for i, pointcloud in enumerate(laser_data):
        _, num_points_in_frame = pointcloud.shape
        start = last
        end = last + num_points_in_frame
        lidar_points[:, start:end] = pointcloud
        last = end
        cum_length.append(last)

    start_data = cum_length[start_index] 
    end_data = cum_length[end_index]
    
    print(lidar_points[:, start_data:end_data])

    plt.figure(figsize=(10, 6))
    # x_diam = np.max(lidar_points[0, start_data:end_data]) - np.min(lidar_points[0, start_data:end_data])
    # y_diam = np.max(lidar_points[1, start_data:end_data]) - np.min(lidar_points[1, start_data:end_data])
    # # Set axes so that the scale is the same in both directions
    # plt.xlim(np.min(lidar_points[0, start_data:end_data]) - x_diam * 0.1, np.max(lidar_points[0, start_data:end_data]) + x_diam * 0.1)
    # plt.ylim(np.min(lidar_points[1, start_data:end_data]) - y_diam * 0.1, np.max(lidar_points[1, start_data:end_data]) + y_diam * 0.1)

    plt.scatter(lidar_points[0, start_data:end_data], lidar_points[1, start_data:end_data], s=5, marker='.')
    
    plt.plot(pose_x[start_index:end_index], pose_y[start_index:end_index], label="Pose Data", marker='x', linestyle='', c='red')
    # Draw bearing as arrow from each pose  
    for i in range(start_index, end_index):
        plt.arrow(pose_x[i], pose_y[i], np.cos(pose_theta[i]) * 0.2, np.sin(pose_theta[i]) * 0.2, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
    plt.show()

def plot_map_from_file_using_mapper(pose_file, laser_data_file):
    mapper = Mapper(map_size=150, map_resolution=0.1, lidar_range=5, lidar_spatial_tolerance=0.1, lidar_angular_tolerance=np.pi/180*1)
    measurement_frames = []
    length = 0
    
    pose_data = np.loadtxt(pose_file)
    pose_x, pose_y, pose_theta = pose_data[:, 0], pose_data[:, 1], pose_data[:, 2]
    mapper.interactive_off()
    # mapper.origin_index = (0, mapper.map_size-1)

    with open(laser_data_file, "r") as laser_file: 

        for i, line in enumerate(laser_file):
            if i > len(pose_x)-1:
                break
            points = line.split(", ")[:-1]
            length += len(points) 

            px, py, ptheta = pose_x[i], pose_y[i], pose_theta[i]

            pointcloud = np.zeros((2, len(points)))
            for j, point in enumerate(points):
                x, y = float(point.split(" ")[0]), float(point.split(" ")[1])
                # Transform using pose_frame

                x_global = px + x * np.cos(ptheta) - y * np.sin(ptheta)
                y_global = py + y * np.cos(ptheta) + x * np.sin(ptheta)

                pointcloud[0, j] = x_global
                pointcloud[1, j] = y_global

            measurement_frames.append(point_cloud_to_measurements(pointcloud))
    
    start = 0
    end = start +2
    for i in range(start, end):
        print(f"Frame {i}")
        mapper.update_map(pose_data[i], measurement_frames[i])

    mapper.draw_map(pose_data[start:end])
    plt.show()

def point_cloud_to_measurements(pointcloud):
    measurements = np.zeros(180)
    # create a list with 180 elements, where each element is the range measurement, and the index corresponds to the angle
    points_x = pointcloud[0, :] 
    points_y = pointcloud[1, :]
    for i in range(len(points_x)):
        # Wrap into range [-pi/2, pi/2] 
        angle = np.arctan2(points_y[i], points_x[i]) 
        angle = (angle + np.pi/2) % (np.pi)
        print(angle)
        index = int(angle / (np.pi) * 180)
        print(index)
        measurements[index] = np.linalg.norm(pointcloud[:, i])

    # plt.scatter(points_x, points_y, marker='.')
    # plt.show()
    # Transform back from sine, distance to points frame to check 
    x_from_meas = np.array([np.cos(i * np.pi/180 - np.pi/2) * measurements[i] for i in range(180)])
    y_from_meas = np.array([np.sin(i * np.pi/180 - np.pi/2) * measurements[i] for i in range(180)])
    # plt.scatter(x_from_meas, y_from_meas, marker='.')
    # plt.show()
    return measurements

def plot_using_pose_from_file_and_original_laser_data_using_mapper(pose_file, laser_data_file, start_index=0, end_index=-1):
    mapper = Mapper(map_size=150, map_resolution=0.1, lidar_range=5, lidar_spatial_tolerance=0.1, lidar_angular_tolerance=np.pi/180*1)
    mapper.interactive_off()

    pose_data = np.loadtxt(pose_file) 
    laser_data = np.loadtxt(laser_data_file)

    for i in range(start_index, end_index):
        print(f"Frame {i}")
        mapper.update_map(pose_data[i], laser_data[i])

    mapper.draw_map(pose_data[start_index:end_index])
    plt.show()

if __name__ == "__main__":
    # plot_map_from_file("odometry.txt", "laser_data.txt")
    start = 12
    length = 1
    # plot_map_from_file("odometry.txt", "laser_data.txt", start, start+length)

    # plot_using_pose_from_file_and_original_laser_data_using_mapper("pose.txt", "Intel-office-slam/dataset_intel/intel_LASER_.txt", start, start+length)
    plot_map_from_file("pose.txt", "laser_data.txt", start, start+length)
    plot_map_from_file("pose.txt", "laser_data.txt", start+1, start+length+1)
    plot_map_from_file("pose.txt", "laser_data.txt", start, start+length+1)