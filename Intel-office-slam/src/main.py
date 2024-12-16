from icp import icp, create_cloud
from simpleicp import PointCloud
from play_data import DataPlayer
from matplotlib import pyplot as plt
import numpy as np

class SLAM:
    def __init__(self, laser_data_path: str, odometry_data_path: str):
        self.data_player = DataPlayer(laser_data_path, odometry_data_path)
        self.poses = []
        self.pointclouds = []

    def run(self):
        plt.ion()  # Turn on interactive mode
        fig, (ax1, ax2) = plt.subplots(1, 2)  # Create 1 row, 2 columns of subplots
        
        for i in range(len(self.data_player.laser_data_list)):
            laser_frame, odometry_frame = self.data_player.new_frame()
            pointcloud = create_cloud(laser_frame)
            self.pointclouds.append(pointcloud)
            self.poses.append(self.new_pose(odometry_frame))
            self.plot_pointcloud_2d((ax1, ax2))
            plt.pause(0.1)  # Add small delay between frames
            
        plt.ioff()
        plt.show()

    def new_pose(self, odometry_frame: list[float]) -> list[float]:
        if len(self.poses) == 0:
            return [float(odometry_frame[0]), float(odometry_frame[1]), float(odometry_frame[2])]
        last_pose = self.poses[-1]
        delta_x_abs = float(odometry_frame[0]) * np.cos(last_pose[2]) + float(odometry_frame[1]) * np.sin(last_pose[2]) 
        delta_y_abs = float(odometry_frame[0]) * np.sin(last_pose[2]) - float(odometry_frame[1]) * np.cos(last_pose[2])
        delta_theta = float(odometry_frame[2])
        
        return [last_pose[0] + delta_x_abs, last_pose[1] + delta_y_abs, last_pose[2] + delta_theta]

    def plot_pointcloud_2d(self, axes):
        ax1, ax2 = axes
        pointcloud = self.pointclouds[-1]
        ax1.clear()
        ax1.scatter(pointcloud['x'], -1 * pointcloud['y'])
        ax1.set_title(f'Point Cloud - Frame {self.data_player.data_frame}')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        
        ax2.clear()
        ax2.scatter([p[0] for p in self.poses], [p[1] for p in self.poses])
        ax2.set_title('Robot Trajectory')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        
        fig = plt.gcf()
        fig.canvas.draw()

if __name__ == "__main__":
    slam = SLAM("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    slam.run()
