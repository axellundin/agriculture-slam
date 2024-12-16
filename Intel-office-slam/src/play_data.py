import os 
import numpy as np

class DataPlayer:
    """
    A class to play the data from the given paths.
    """
    def __init__(self, laser_data_path, odometry_data_path):
        self.laser_data_path = laser_data_path
        self.odometry_data_path = odometry_data_path
        self.laser_data_list = []
        self.odometry_data_list = []
        self.data_frame = 0
        self.load_data()

    def load_data(self)-> None:
        """
        Load the laser and odometry data from the given paths.
        """
        current_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(current_dir, self.laser_data_path), 'r') as f:
            for line in f:
                self.laser_data_list.append(line.strip().split())
        with open(os.path.join(current_dir, self.odometry_data_path), 'r') as f:
            for line in f:
                self.odometry_data_list.append(line.strip().split())

    def new_frame(self)-> tuple[np.array, np.array]:
        """
        Get the next frame of data and return the laser and odometry data.
        """
        if self.data_frame >= len(self.laser_data_list):
            return [], []
        self.data_frame += 1
        return np.array(self.laser_data_list[self.data_frame - 1]), np.array(self.odometry_data_list[self.data_frame - 1])

    def reset(self)-> None:
        """
        Reset the data frame to the first frame.
        """
        self.data_frame = 0

    def get_frame(self, frame_index)-> tuple[np.array, np.array]:
        """
        Get the data of a specific frame.
        """
        return np.array(self.laser_data_list[frame_index - 1]), np.array(self.odometry_data_list[frame_index - 1])

    def get_frame_count(self)-> tuple[int, int]:
        """
        Get the number of frames in the laser and odometry data.
        """
        return len(self.laser_data_list), len(self.odometry_data_list)

if __name__ == "__main__":
    data_dir = "../dataset_intel"
    laser_data_path = os.path.join(data_dir, "intel_LASER_.txt")
    odometry_data_path = os.path.join(data_dir, "intel_ODO.txt")
    data_player = DataPlayer(laser_data_path, odometry_data_path)
    for i in range(10):
        print("Frame", i+1, ":", data_player.new_frame())

