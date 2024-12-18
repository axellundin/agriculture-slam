from icp_wrapper import lidar_to_points
from play_data import DataPlayer
from matplotlib import pyplot as plt
import numpy as np
from plot import Plotter
from filtering import filter_points
from mapping import Mapper

class SLAM:
    def __init__(self, laser_data_path: str, odometry_data_path: str):
        self.data_player = DataPlayer(laser_data_path, odometry_data_path)
        self.plotter = Plotter(self)
        self.mapper = Mapper(map_size=400, map_resolution=0.07, lidar_range=8, lidar_spatial_tolerance=0.1, lidar_angular_tolerance=np.pi/180*1)
        self.poses = []
        self.pointclouds = []
        self.icp_transforms = []

    def run(self): 
        # self.plotter.interactive_on()
        self.mapper.interactive_on()
        for i in range(len(self.data_player.laser_data_list)):
            print(f"Frame {i}")
            laser_frame, odometry_frame = self.data_player.new_frame()
            # make laser frame into np.array of floats
            laser_frame = np.array(laser_frame, dtype=float)
            pointcloud = lidar_to_points(laser_frame)
            pointcloud = filter_points(pointcloud)
            self.pointclouds.append(pointcloud)
            self.integrate_new_pose(odometry_frame)
            # self.plotter.plot_pointcloud_2d()
            # Update map
            if len(self.poses) > 0:
                self.mapper.update_map(self.poses[-1], laser_frame)
                self.mapper.draw_map(self.poses)
            plt.pause(0.001)  # Add small delay between frames
        self.mapper.interactive_off()
        # self.plotter.interactive_off()

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
            self.poses.append(np.array([float(odometry_frame[0]), float(odometry_frame[1]), float(odometry_frame[2])]))
            return
        last_pose = self.poses[-1]
        delta_x_abs = float(odometry_frame[0]) * np.cos(last_pose[2]) - float(odometry_frame[1]) * np.sin(last_pose[2]) 
        delta_y_abs = float(odometry_frame[0]) * np.sin(last_pose[2]) + float(odometry_frame[1]) * np.cos(last_pose[2])
        delta_theta = float(odometry_frame[2])

        # Wrap angle to [-pi, pi]
        new_theta = last_pose[2] + delta_theta
        new_theta = ((new_theta + np.pi) % (2 * np.pi)) - np.pi
        self.poses.append(np.array([last_pose[0] + delta_x_abs, last_pose[1] + delta_y_abs, new_theta]))

if __name__ == "__main__":
    slam = SLAM("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    slam.run()
