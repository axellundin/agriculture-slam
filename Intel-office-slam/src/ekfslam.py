import numpy as np

class EKFSLAM:
    def __init__(self):
        self.poses = []
        self.covariances = [np.zeros((3, 3))]
        self.odometry_transforms = []
        self.Q = np.eye(3)
        self.R = np.eye(3)


    def dynamic_model(self, odometry_data):
        """ Performs the dynamic model of the EKFSLAM.
        Args:
            odometry_data (np.array): The odometry data.
        """
        transform = np.array([[np.cos(odometry_data[2]), -np.sin(odometry_data[2]), odometry_data[0]],
                        [np.sin(odometry_data[2]), np.cos(odometry_data[2]), odometry_data[1]],
                        [0, 0, 1]])
        self.odometry_transforms.append(transform)
        new_pose = self.poses[-1] + transform @ odometry_data
        self.poses.append(new_pose)
        new_covariance = self.covariances[-1] + transform @ self.covariances[-1] @ transform.T
        self.covariances.append(self.covariances[-1])

    def measurement_model(self, transform):
        """ Performs the measurement model of the EKFSLAM.
        Args:
            transform (np.array): The transform found by ICP.
        """
        if len(self.poses) < 2:
            return
        predicted_measurement = self.poses[-1] - self.poses[-2]
        # Extract rotation in radians and translation from transform
        delta_rotation = np.arctan2(transform[1, 0], transform[0, 0])
        delta_translation = transform[:2, 2]
        measurement = np.hstack((delta_translation, delta_rotation))
        # Kalman gain
        H = np.hstack((np.eye(3), -np.eye(3)))
        K = self.covariances[-1] @ H.T @ np.linalg.inv(H @ self.covariances[-1] @ H.T + self.R)  
        # Update the state
        self.poses[-1] = self.poses[-1] + K @ (measurement - predicted_measurement)
        self.covariances[-1] = (np.eye(3) - K @ H) @ self.covariances[-1]
        # Update the covariance
        self.covariances[-1] = self.covariances[-1] + K @ self.R @ K.T
