from simpleicp import PointCloud, SimpleICP
import numpy as np

def icp(source_points, target_points):
    source_cloud = PointCloud(source_points)
    target_cloud = PointCloud(target_points)
    icp = SimpleICP(source_cloud, target_cloud)
    icp.align()
    return icp.transformation

def create_cloud(measurements: list[float]) -> PointCloud:
    # Each measurement consists of 180 range measurements from a lidar 
    points = np.zeros((180, 3))
    angle_increment = np.pi / 180
    for i in range(180):
        if measurements[i] == 0:
            continue
        angle = i * angle_increment - np.pi / 2
        points[i, 0] = float(measurements[i]) * np.cos(angle)
        points[i, 1] = float(measurements[i]) * np.sin(angle)

    return PointCloud(points, columns=['x', 'y', 'z'])
