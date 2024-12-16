from simpleicp import PointCloud
from matplotlib import pyplot as plt

class Plotter: 
    def __init__(self, SLAM):
        self.fig, self.ax1, self.ax2 = plt.subplots(1, 2)
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
