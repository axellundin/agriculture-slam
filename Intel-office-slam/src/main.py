from icp_wrapper import lidar_to_points, get_transform
from play_data import DataPlayer
from matplotlib import pyplot as plt
import numpy as np
from plot import Plotter
from feature_detection import find_corners_in_frame

class SLAM:
    def __init__(self, laser_data_path: str, odometry_data_path: str):
        self.data_player = DataPlayer(laser_data_path, odometry_data_path)
        self.plotter = Plotter(self)
        
        self.poses = []
        self.pointclouds = []
        self.icp_transforms = []

    def run(self): 
        self.plotter.interactive_on()
        
        for i in range(len(self.data_player.laser_data_list)):
            laser_frame, odometry_frame = self.data_player.new_frame()
            pointcloud = lidar_to_points(laser_frame)
            self.pointclouds.append(pointcloud)
            self.integrate_new_pose(odometry_frame)
            self.plotter.plot_pointcloud_2d()
            # if self.do_icp():
            #     pass
                # self.plotter.plot_icp_transform(self.pointclouds[-2], self.pointclouds[-1], self.icp_transforms[-1])
            plt.pause(0.1)  # Add small delay between frames
            
        self.plotter.interactive_off()

    def do_icp(self):
        if len(self.pointclouds) < 3:
            return False
        icp_transform, points = get_transform(self.pointclouds[-1], self.pointclouds[-2])
        self.icp_transforms.append(icp_transform[-1])
        return True
    
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
