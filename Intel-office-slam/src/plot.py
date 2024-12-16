from simpleicp import PointCloud
from matplotlib import pyplot as plt
import numpy as np

class Plotter: 
    def __init__(self, SLAM):
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2)
        self.SLAM = SLAM

    def interactive_on(self):
        plt.ion()

    def interactive_off(self):
        plt.ioff()

    def plot_pointcloud_2d(self):
        pointcloud = self.SLAM.pointclouds[-1]
        self.ax1.clear()
        self.ax1.scatter(pointcloud['x'], -1 * pointcloud['y'])
        self.ax1.set_title(f'Point Cloud - Frame {self.SLAM.data_player.data_frame}')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_ylim(-10, 10)
        self.ax1.set_xlim(0, 10)

        self.ax2.clear()
        self.ax2.scatter([p[0] for p in self.SLAM.poses], [p[1] for p in self.SLAM.poses])
        self.ax2.set_title('Robot Trajectory')
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.set_ylim(-50, 50)
        self.ax2.set_xlim(-50, 50)

        fig = plt.gcf()
        fig.canvas.draw()

    def plot_icp_transform(self, last_pointcloud_, new_pointcloud_, H):
        if len(self.SLAM.icp_transforms) < 2:
            return
        # Plot the pointclouds, but the new one is transformed by H and the 
        # points that correspond to X_mov_transformed are green 
        # Shift new pointcloud by H
        homogeneous_coords = np.hstack([last_pointcloud_, np.ones((last_pointcloud_.shape[0], 1))])
        last_pointcloud = (H @ homogeneous_coords.T).T
        new_pointcloud = new_pointcloud_

        self.ax1.clear()
        self.ax1.scatter(last_pointcloud[:,0], -1 * last_pointcloud[:,1], color='red')
        self.ax1.scatter(new_pointcloud[:,0], -1 * new_pointcloud[:,1], color='green')
        self.ax1.set_title(f'Point Cloud - Frame {self.SLAM.data_player.data_frame}')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_ylim(-10, 10)
        self.ax1.set_xlim(-10, 10)

        fig = plt.gcf()
        fig.canvas.draw()
