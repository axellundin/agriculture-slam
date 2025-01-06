from icp_wrapper import lidar_to_points
from play_data import DataPlayer
from matplotlib import pyplot as plt
import numpy as np
from plot import Plotter
from filtering import filter_points
from mapping import Mapper
from ekfslam import EKFSLAM

class SLAM:
    def __init__(self, laser_data_path: str, odometry_data_path: str):
        self.data_player = DataPlayer(laser_data_path, odometry_data_path)
        self.plotter = Plotter(self)
        self.mapper = Mapper(map_size=400, map_resolution=0.07, lidar_range=8, lidar_spatial_tolerance=0.1, lidar_angular_tolerance=np.pi/180*1)
        self.pointclouds = []
        self.last_odometry = None
        self.slam = EKFSLAM()

    def run(self): 
        self.mapper.interactive_on()
        for i in range(len(self.data_player.laser_data_list)):
            print(f"Frame {i}")
            laser_frame, odometry_frame = self.data_player.new_frame()
            # make laser frame into np.array of floats
            laser_frame = np.array(laser_frame, dtype=float)
            pointcloud = lidar_to_points(laser_frame)
            pointcloud = filter_points(pointcloud)
            if len(self.slam.means) == 0: 
                self.slam.means.append(np.array([0, 0, 0, 0, 0, 0]))
                self.slam.covariances.append(np.zeros((6, 6)))
                self.pointclouds.append(pointcloud)
                continue
            if self.last_odometry is not None:
                self.slam.iteration(self.last_odometry, self.pointclouds[-1], pointcloud, perform_icp_update=True)
            self.last_odometry = odometry_frame
            self.pointclouds.append(pointcloud)

            # Update map
            if len(self.slam.means) > 0:
                self.mapper.update_map(self.slam.means[-1], laser_frame)
                self.mapper.draw_map(self.slam.means)
            plt.pause(0.001)  # Add small delay between frames
        self.mapper.interactive_off()

if __name__ == "__main__":
    slam = SLAM("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    slam.run()
