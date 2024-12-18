import numpy as np
import matplotlib.pyplot as plt

class Mapper:
    FREE = 1
    UNKNOWN = 0
    OCCUPIED = -1

    def __init__(self, map_size: int=1000, map_resolution: float=0.05, lidar_range: float=10, lidar_spatial_tolerance: float=0.1, lidar_angular_tolerance: float=np.pi/180*10):
        """
            Mapper class that stores the map of the environment.
            Parameters:
                map_size: int, the size of the map
                map_resolution: float, the resolution of the map
                lidar_range: float, the range of the lidar
                lidar_spatial_tolerance: float, the spatial tolerance of the lidar
                lidar_angular_tolerance: float, the angular tolerance of the lidar
        """
        # Initialize as unknown
        self.map_size = map_size
        self.probabilistic_occupancy_grid = np.zeros((self.map_size, self.map_size))  
        self.map_resolution = map_resolution
        self.origin_index = np.array([self.map_size // 2, self.map_size // 2])
        self.lidar_range = lidar_range # meters 
        self.lidar_spatial_tolerance = lidar_spatial_tolerance # meters
        self.lidar_angular_tolerance = lidar_angular_tolerance # radians
        
        # Initialize figure and axes for animation
        self.fig, self.ax = plt.subplots()
        self.im = None  # Will store the image for updating
        
        # Turn on interactive mode
        plt.ion()

        # Add update counter
        self.update_counter = 0

    def interactive_on(self):
        plt.ion()

    def interactive_off(self):
        plt.ioff()

    def update_map(self, pose: np.ndarray, measurement: np.ndarray):
        """
            Update the map from a point cloud.
        """
        pose_index = self.world_to_map(pose) 
        lidar_index_range = int(self.lidar_range / self.map_resolution)
        
        # Original update code
        for x_idx in range(int(pose_index[0] - lidar_index_range), int(pose_index[0] + lidar_index_range)):
            for y_idx in range(int(pose_index[1] - lidar_index_range), int(pose_index[1] + lidar_index_range)):
                if 0 <= x_idx < self.map_size and 0 <= y_idx < self.map_size:  # Check bounds
                    pixel = np.array([x_idx, y_idx])
                    old_value = self.probabilistic_occupancy_grid[x_idx, y_idx]
                    new_value = old_value + self.inverse_sensor_model(pixel, pose, measurement) - self.UNKNOWN
                    self.probabilistic_occupancy_grid[x_idx, y_idx] = new_value

        # Increment counter and draw map if needed
        self.update_counter += 1
        if self.update_counter >= 20:
            self.draw_map([pose])  # Note: This assumes you want to draw only the current pose
            self.update_counter = 0  # Reset counter

    def inverse_sensor_model(self, pixel: np.ndarray, pose: np.ndarray, measurement: np.ndarray):
        """
            Inverse sensor model to update the map.
        """
        com = (pixel - self.origin_index) * self.map_resolution 
        r = np.linalg.norm(com - pose[:2])
        if r < 0.1:
            return self.FREE
        phi = np.arctan2(com[1] - pose[1], com[0] - pose[0]) - pose[2]
        # self.interactive_off()
        # if com[0] > 0:
        #     plt.figure()
        #     plt.plot(com[0], com[1], 'ro')
        #     plt.plot(pose[0], pose[1], 'bo')
        #     plt.show()

        # wrap phi to -pi/2 to pi/2
        phi = np.arctan2(np.sin(phi), np.cos(phi))
        if np.abs(phi) > np.pi/2:
            return self.UNKNOWN
        
        k = int(np.round((phi + np.pi/2) * 180/np.pi))
        if k < 0 or k >= len(measurement):  # Add bounds check
            return self.UNKNOWN
        
        theta_k = k * np.pi / 180 - np.pi/2
        z_k = measurement[k]
        # if com[0] > 0:
        #     print("phi: ", phi, "r: ", r, "theta_k: ", theta_k, "z_k: ", z_k)

        if r > min(self.lidar_range, z_k + self.lidar_spatial_tolerance/2) or np.abs(phi-theta_k) > self.lidar_angular_tolerance:
            # print("Third condition")
            return self.UNKNOWN    
        if z_k < self.lidar_range and np.abs(z_k - r) < self.lidar_spatial_tolerance/2:
            # print("Fourth condition")
            return self.OCCUPIED
        if r <= z_k: 
            # print("Fifth condition")
            return self.FREE
        
        # print("Sixth condition")
        return self.UNKNOWN

    def world_to_map(self, pose: np.ndarray):
        """
            Convert world coordinates to map coordinates.
        """
        return np.hstack([pose[:2] / self.map_resolution + self.origin_index, pose[2]])
    
    def draw_map(self, poses):
        """
        Draw the occupancy grid map
        """
        # Clear the current plot
        self.ax.clear()
        
        # Plot the occupancy grid
        self.ax.imshow(self.probabilistic_occupancy_grid.T,
                      cmap='gray',
                      vmin=min(self.FREE, self.OCCUPIED),
                      vmax=max(self.FREE, self.OCCUPIED))
        
        # draw lines between consecutive poses
        for i in range(len(poses) - 1):
            # rescale and shift to origin
            self.ax.plot([poses[i][0] / self.map_resolution + self.origin_index[0], poses[i+1][0] / self.map_resolution + self.origin_index[0]], [poses[i][1] / self.map_resolution + self.origin_index[1], poses[i+1][1] / self.map_resolution + self.origin_index[1]], 'r-')

        self.ax.set_title('Occupancy Grid Map')
        plt.draw()