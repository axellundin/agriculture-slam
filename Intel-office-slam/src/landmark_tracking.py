import numpy as np
import matplotlib.pyplot as plt

class Landmark: 
    def __init__(self):
        self.likelihood = 0.5

class CornerLandmark(Landmark):
    def __init__(self, position):
        super().__init__()
        self.type = "CORNER"
        self.position = position
    
    def calculate_likelihood(self, observed_corner):
        # Calculate the likelihood of the observed corner being the landmark
        # Use the distance between the observed corner and the landmark position
        # Use the likelihood threshold to determine if the landmark is visible
        pass

    def update_likelihood(self, likelihood):
        pass 

class LineLandmark(Landmark):
    def __init__(self, start_point, end_point):
        super().__init__()
        self.type = "LINE_SEGMENT"
        self.start_point = start_point
        self.end_point = end_point
    
    def update_likelihood(self, likelihood):
        pass 

class LandmarkTracker:
    def __init__(self, likelihood_threshold=0.9, corner_likelihood_threshold=0.9):
        self.landmarks = []
        self.landmark_provisioning_list = []
        self.likelihood_threshold = likelihood_threshold
        self.corner_likelihood_threshold = corner_likelihood_threshold

    # def correlate_landmarks(self, line_segments):
    #     # Correlate endpoints with existing landmarks and provisioned landmarks.
    #     # If a landmark has high enough likelihood, update the landmark position
    #     # If a observation does not correlate to any landmark, provision a new landmark

    #     observed_corners = []
    #     for line_segment in line_segments:
    #         endpoints = line_segment[3]
    #         observed_corners.append(endpoints[0])
    #         observed_corners.append(endpoints[1])

    #     for landmark in self.landmarks:
    #         if landmark.type == "CORNER":
    #             likelihood = landmark.calculate_likelihood(observed_corners)
    #             if likelihood > self.corner_likelihood_threshold:
    #                 landmark.update_likelihood(likelihood)

    def add_landmarks(self, robot_pose, line_segments):
        for line_segment in line_segments:
            endpoints = line_segment[3] 
            start_point = endpoints[0]
            end_point = endpoints[1]
            shifted_start_point = self.shift_position(start_point, robot_pose)
            shifted_end_point = self.shift_position(end_point, robot_pose)
            self.landmarks.append(CornerLandmark(shifted_start_point))
            self.landmarks.append(CornerLandmark(shifted_end_point))
            self.landmarks.append(LineLandmark(shifted_start_point, shifted_end_point))

    def shift_position(self, position, robot_pose):
        x = position[0]
        y = position[1]
        theta = robot_pose[2]
        x_new = robot_pose[0] + x * np.cos(theta) + y * np.sin(theta)
        y_new = robot_pose[1] - x * np.sin(theta) + y * np.cos(theta)
        return np.array([x_new, y_new])

    def update_landmarks(self, line_segments):
        pass

    def update_landmark_provisioning_list(self, landmark_provisioning_list):
        self.landmark_provisioning_list = landmark_provisioning_list

    def draw_landmarks(self):
        print(f"Landmarks: {self.landmarks}")
        for landmark in self.landmarks: 
            if landmark.type == "LINE_SEGMENT":
                plt.plot([landmark.start_point[0], landmark.end_point[0]], [landmark.start_point[1], landmark.end_point[1]], color='red')
            elif landmark.type == "CORNER":
                plt.scatter(landmark.position[0], landmark.position[1], color='blue')