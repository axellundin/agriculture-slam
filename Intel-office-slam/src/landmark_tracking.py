import numpy as np
import matplotlib.pyplot as plt
from feature_detection import get_features

class ProvisionedLandmark:
    def __init__(self, position, bearing):
        self.position = position
        self.bearing = bearing
        self.likelihood = 0.5
        self.sigma = 0.1

    def compute_likelihood(self, robot_pose, observed_corner): 
       # Shift into robots frame of reference (in x, y)
       predicted_measurement = self.position - robot_pose[0:2]
       # rotate into robots frame of reference (in x, y)
       predicted_measurement = np.array([predicted_measurement[0] * np.cos(robot_pose[2]) + predicted_measurement[1] * np.sin(robot_pose[2]),
                                        -predicted_measurement[0] * np.sin(robot_pose[2]) + predicted_measurement[1] * np.cos(robot_pose[2])])
       # Compute likelihood from mahalanobis distance using comparison of bearings.
       mahalanobis_distance = np.linalg.norm(predicted_measurement - observed_corner)
       likelihood = 1 / (2 * np.pi * self.sigma) * np.exp(-0.5 * mahalanobis_distance**2 / self.sigma**2)
       return likelihood
    
    def update_position(self, new_position, new_bearing, new_likelihood):
        self.position = (self.position + new_position) / 2
        self.bearing = (self.bearing + new_bearing) / 2
        self.likelihood = (self.likelihood + new_likelihood) / 2

class LandmarkTracker: 
    def __init__(self):
        self.provisioned_landmarks = []

    def update_landmarks(self, laser_data):
        features = get_features(laser_data)
        self.provisioned_landmarks.append(features)
        self.landmark_map[features] = len(self.provisioned_landmarks) - 1

    def get_landmarks(self):
        return self.provisioned_landmarks

