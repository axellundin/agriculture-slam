import numpy as np
import matplotlib.pyplot as plt
from play_data import DataPlayer
from icp_wrapper import lidar_to_points
from scipy.spatial import cKDTree

def remove_close_and_distant_points(points, threshold_far=7, threshold_close=0.5):
    """
    Removes points that are too far away from the origin.
    """
    close_points = points[np.linalg.norm(points, axis=1) < threshold_far]
    return close_points[np.linalg.norm(close_points, axis=1) > threshold_close]

def remove_points_with_few_neighbors(points, threshold_neighbors=5):
    """
    Removes points that have less than threshold_neighbors neighbors.
    """
    tree = cKDTree(points)
    indices = tree.query_ball_point(points, r=0.5)
    neighbor_counts = np.array([len(idx_list) for idx_list in indices])
    return points[neighbor_counts >= threshold_neighbors]

def filter_points(points:np.ndarray)->np.ndarray:
    points = remove_close_and_distant_points(points)
    points = remove_points_with_few_neighbors(points)
    return np.array(points)

if __name__ == "__main__":
    data_player = DataPlayer("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    
    for frame in range(50,100):
        laser_data, _ = data_player.get_frame(frame)
        points = lidar_to_points(laser_data)
        points = filter_points(points)
        plt.scatter(points[:, 0], points[:, 1])
        plt.show()