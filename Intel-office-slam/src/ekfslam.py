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
        self.H = np.array([np.eye(3), -np.eye(3)])
        self.Q = np.eye(3)
        self.R = np.eye(3)
    
    def iteration(self, odometry_data, points_cloud1, points_cloud2, perform_update = True):
        self.prediction_step()
        if perform_update:
            pass
        
        
    def prediction_step(self, odometry_data):
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
                              [0, 0, 0]])S
        
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
        
    def measurement(self, points_cloud1, points_cloud2):
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
        z = np.array([t[0], t[1], np.atan2(R[1,0], R[0,0])])
        self.measurements.append(z)
  
    def update(self):
        Sigma = self.covariances[-1]
        mu = self.poses[-1]
        H = self.H
        Q = self.Q
        K = np.divide(Sigma @ H.T, H @ Sigma @ H .T + Q)
        self.kalman_gains.append(K)
        eta = self.measurements[-1] - self.predicted_measurements[-1]
        self.innovations.append(eta)
        
        mu = mu + K @ eta
        Sigma = (np.eye(6) - K @ H) @ Sigma
        
        self.poses.append(mu)
        self.covariances(Sigma)
