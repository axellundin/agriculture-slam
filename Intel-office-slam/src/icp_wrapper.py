from icp.icp import * 
import numpy as np

def lidar_to_points(measurements: list[float])->np.ndarray:
    # Each measurement consists of 180 range measurements from a lidar 
    points = np.zeros((180, 2))
    angle_increment = np.pi / 180
    for i in range(180):
        if measurements[i] == 0:
            continue
        angle = i * angle_increment - np.pi / 2
        points[i, 0] = float(measurements[i]) * np.cos(angle)
        points[i, 1] = float(measurements[i]) * np.sin(angle)

    return points

def get_transform(last_points: np.ndarray, new_points: np.ndarray)->np.ndarray:
    icp_transform = icp(last_points, new_points, point_pairs_threshold=20, max_iterations=1000, )
    return icp_transform