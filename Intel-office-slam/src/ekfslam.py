import numpy as np
from icp_implementation import ICP

class EKFSLAM:
    def __init__(self):
        self.poses = []
        self.covariances = [np.zeros((3, 3))]
        self.odometry = []
        self.icp_transforms = []
        self.predicted_measurements = []
        self.Q = np.eye(3)
        self.R = np.eye(3)
    
    def iteration(self, mu, sigma, pc1, pc2):
        self.dynamic_model()
        mu = self.poses[-1]
        Sigma = self.covariances[-1]
        G = self.odometry_transforms[-1]
        mu = G @ mu
        Sigma = G @ Sigma @ G.T + self.R    
        
    def dynamic_model(self, odometry_data):
        """ Performs the dynamic model of the EKFSLAM.
        Args:
            odometry_data (np.array): The odometry data.
        """
        
        old_pose = self.poses[-1]
        old_covariance = self.covariances[-1]
        
        th = old_pose[2]
        
        c = np.cos(th)
        s = np.sin(th)
        transform = np.array([[c, -s, 0],
                              [s, c, 0],
                              [0, 0, 1],
                              [0, 0, 0],
                              [0, 0, 0],
                              [0, 0, 0]])
        
        G = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        
        new_pose = G @ old_pose + transform @ odometry_data
        new_covariance = G @ old_covariance @ G.T + self.R
        
        self.odometry.append(odometry_data)
        self.poses.append(new_pose)
        self.covariances.append(new_covariance)
        
    def find_icp_transform(self, points_cloud1, points_cloud2):
        """ Performs icp to get the rotation matrix and translation vector. Returns the homogeneus transform matrix
        Args:
            points_cloud1 (np.array) a Nx2 vector containing the lidar scans at time k-1
            points_cloud2 (np.array) a Nx2 vector containing the lidar scans at time k
        """
        icp = ICP.run(points_cloud1, points_cloud2)
        R, t = icp()
        #delta_theta = np.atan2(R[0,1], R[0,0])
        M = np.array([[R[0,0], R[0,1], t[0]],
                      [R[1,0], R[1,1], t[1]],
                      [0, 0, 1]])
        self.icp_transforms.append(M)

    def measurement_model(self):
        """ Performs the measurement model of the EKFSLAM.
        Args:
            transform (np.array): The transform found by ICP.
        """
        if len(self.poses) < 2:
            return

        odometry_data = self.odometry[-1]
        delta_x, delta_y, delta_th = odometry_data
        c = np.cos(delta_th)
        s = np.sin(delta_th)
        
        H = np.array([[c, -1, delta_x],
                      [s, c, delta_y],
                      [0, 0, 1]])
        
        self.predicted_measurements.append(H)

    def compute_inovation(self):
        pass
    
    def compute_kalman_gain(self):
        pass
    
    def 