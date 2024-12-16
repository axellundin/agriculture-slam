from icp import icp, create_cloud
from simpleicp import PointCloud
from play_data import DataPlayer
from matplotlib import pyplot as plt
import numpy as np
from plot import Plotter

class SLAM:
    def __init__(self, laser_data_path: str, odometry_data_path: str):
        self.data_player = DataPlayer(laser_data_path, odometry_data_path)
        self.poses = []
        self.pointclouds = []
        self.plotter = Plotter(self)

    def run(self): 
        self.plotter.interactive_on()
        
        for i in range(len(self.data_player.laser_data_list)):
            laser_frame, odometry_frame = self.data_player.new_frame()
            pointcloud = create_cloud(laser_frame)
            self.pointclouds.append(pointcloud)
            self.integrate_new_pose(odometry_frame)
            self.plotter.plot_pointcloud_2d()

            plt.pause(0.1)  # Add small delay between frames
            
        self.plotter.interactive_off()
    
    def integrate_new_pose(self, odometry_frame: list[float]) -> list[float]:
        """ 
        Update state from odometry using dynamic model
        """
        if len(self.poses) == 0:
            return [float(odometry_frame[0]), float(odometry_frame[1]), float(odometry_frame[2])]
        last_pose = self.poses[-1]
        delta_x_abs = float(odometry_frame[0]) * np.cos(last_pose[2]) - float(odometry_frame[1]) * np.sin(last_pose[2]) 
        delta_y_abs = float(odometry_frame[0]) * np.sin(last_pose[2]) + float(odometry_frame[1]) * np.cos(last_pose[2])
        delta_theta = float(odometry_frame[2])

        self.poses.append([last_pose[0] + delta_x_abs, last_pose[1] + delta_y_abs, last_pose[2] + delta_theta])

if __name__ == "__main__":
    slam = SLAM("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    slam.run()
