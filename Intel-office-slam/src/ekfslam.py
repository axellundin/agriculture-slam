import numpy as np
from icp_implementation import ICP

class EKFSLAM:
    def __init__(self):
        self.poses = []
        self.covariances = [np.zeros((3, 3))]
        self.odometry = []
        self.icp_transforms = []
        self.measurements = []
        self.predicted_measurements = []
        self.innovations = []
        self.kalman_gains = []
        self.H = np.hstack([np.eye(3), -np.eye(3)])
        self.Q = np.eye(3)
        self.R = np.eye(6)
    
    def iteration(self, odometry_data, points_cloud1, points_cloud2, perform_update = True):
        self.prediction_step(odometry_data)
        if perform_update:
            self.measurement(points_cloud1, points_cloud2)
            self.update_step() 
        
    def prediction_step(self, odometry_data):
        """ Performs the dynamic model of the EKFSLAM.
        Args:
            odometry_data (np.array): The odometry data.
        """
        state = self.poses[-1]
        covariance = self.covariances[-1]
        
        th = state[2]
        
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
        print(state)
        new_pose = G @ state + transform @ np.array(odometry_data, dtype=float)
        new_covariance = G @ covariance @ G.T + self.R
        
        self.odometry.append(np.array(odometry_data, dtype=float))
        self.poses.append(new_pose)
        self.covariances.append(new_covariance)
        
    def measurement(self, points_cloud1, points_cloud2):
        """ Performs icp to get the rotation matrix and translation vector. Returns the homogeneus transform matrix
        Args:
            points_cloud1 (np.array) a Nx2 vector containing the lidar scans at time k-1
            points_cloud2 (np.array) a Nx2 vector containing the lidar scans at time k
        """
        icp = ICP(points_cloud1, points_cloud2)
        R, t = icp.run()
    
        M = np.array([[R[0,0], R[0,1], t[0]],
                      [R[1,0], R[1,1], t[1]],
                      [0, 0, 1]])
        self.icp_transforms.append(M)
        z = np.array([t[0], t[1], np.atan2(R[1,0], R[0,0])])
        self.measurements.append(z)
  
    def update_step(self):
        Sigma = self.covariances[-1]
        mu = self.poses[-1]
        H = self.H
        Q = self.Q
        K = Sigma @ H.T @ np.linalg.inv(H @ Sigma @ H.T + Q)
        self.kalman_gains.append(K)
        self.predicted_measurements.append(self.odometry[-1])
        eta = self.measurements[-1] - self.predicted_measurements[-1]
        self.innovations.append(eta)
        
        mu = mu + K @ eta
        Sigma = (np.eye(6) - K @ H) @ Sigma
        
        self.poses.append(mu)
        self.covariances.append(Sigma)
