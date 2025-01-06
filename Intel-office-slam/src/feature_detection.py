import numpy as np
from play_data import DataPlayer
from icp_wrapper import lidar_to_points
from filtering import filter_points
import matplotlib.pyplot as plt

def seed_segment_detection(laser_data: np.ndarray, epsilon: float, delta: float, num_points_per_seed: int, num_points_per_segment: int, start_index: int):
    """
    Detect segments of points that satisfy the following conditions:
    - The distance from every point in the seed-segment to the fitting straight line should
    be less than a given threshold epsilon.
    - The distance from every point in the seed-segment to the current point should
    be less than a given threshold delta.
    """
    
    num_points = len(laser_data)
    for i in range(start_index, num_points-num_points_per_segment):
        flag = True 
        j = i + num_points_per_seed
        seed_pos = np.array([i, j])
        seed_data = laser_data[i:j]
        seed_fitted_line = fit_line(seed_data)
        # plot_line_segment(laser_data, seed_data, seed_fitted_line)
        for k in range(i, j): 
            d1 = distance_to_predicted_point(seed_fitted_line, laser_data[k])
            print(f"d1: {d1}")
            if d1 > delta:
                flag = False 
                break
            d2 = distance_to_line(seed_fitted_line, laser_data[k])
            print(f"d2: {d2}")
            if d2 > epsilon:
                flag = False
                break 
            if k < num_points-1:
                d3 = np.linalg.norm(laser_data[k] - laser_data[k+1])
                print(f"d3: {d3}")
                if d3 > 0.5:
                    flag = False
                    break
        if flag: 
            return seed_fitted_line, seed_data, seed_pos
    return None, None, None

def region_growing(laser_data: np.ndarray, seed_pos: np.ndarray, seed_data: np.ndarray, seed_fitted_line: np.ndarray, epsilon: float, min_len_per_line: int, min_num_points_per_line: int):
    """
    Region growing algorithm to detect line segments.
    """
    num_points = len(laser_data)
    line_data = seed_data
    line_parameters = seed_fitted_line
    line_length = 0 
    number_of_points_in_line = 0 
    i, j = seed_pos
    P_f = j
    P_b = i - 1 

    while distance_to_line(line_parameters, laser_data[P_f]) < epsilon:
        if P_f >= num_points-1:
            break
        line_data = np.vstack((line_data, laser_data[P_f]))
        line_parameters = fit_line(line_data)
        P_f += 1

    P_f -= 1 

    while distance_to_line(line_parameters, laser_data[P_b]) < epsilon:
        if P_b < 0:
            break
        line_data = np.vstack((line_data, laser_data[P_b]))
        line_parameters = fit_line(line_data)
        P_b -= 1

    P_b += 1 

    line_length = np.linalg.norm(laser_data[P_f] - laser_data[P_b])
    number_of_points_in_line = len(line_data)

    print(f"line_length: {line_length}, number_of_points_in_line: {number_of_points_in_line}")

    if line_length >= min_len_per_line and number_of_points_in_line >= min_num_points_per_line:
        return line_data, line_parameters, np.array([P_b, P_f-1])
    else:
        return None, None, None

def find_all_line_segments(_laser_data: np.ndarray, epsilon: float, delta: float, num_points_per_seed: int, num_points_per_segment: int, min_len_per_line: int, min_num_points_per_line: int):
    """
    Find all line segments in the laser data.
    """
    laser_data = _laser_data.copy()
    line_segments = list()
    start_index = 0
    while len(laser_data) > 0:
        seed_parameters, seed_data, seed_pos = seed_segment_detection(laser_data, epsilon, delta, num_points_per_seed, num_points_per_segment, start_index)
        if seed_parameters is not None:
            # plot_line_segment(laser_data, seed_data, seed_parameters)
            line_data, line_parameters, final_line_pos = region_growing(laser_data, seed_pos, seed_data, seed_parameters, epsilon, min_len_per_line, min_num_points_per_line)
            if line_data is not None:
                line_segments.append([line_data, line_parameters, final_line_pos])
                # plot_line_segment(laser_data, line_data, line_parameters)
            else:
                i,j = seed_pos[0], seed_pos[1]
                start_index = j
                continue
            # Remove the the line points from the laser data
            start_index = final_line_pos[1]
        else:
            break
    # Construct the unmatched points by forming the array with the points not contained in any line segment 
    unmatched_points = np.array(laser_data)
    indices_to_remove = set()
    for line_segment in line_segments:
        start, end = line_segment[2]
        indices_to_remove.update(range(int(start), int(end)))
    unmatched_points = np.delete(unmatched_points, list(indices_to_remove), axis=0)
    return line_segments, unmatched_points

def overlap_region_processing(laser_data, line_segments):
    """
    Process the overlap region between two line segments.
    """
    num_lines = len(line_segments)
    for i in range(num_lines-1):
        j = i + 1
        m1, n1 = line_segments[i][2]
        m2, n2 = line_segments[j][2]
        if m2 <= n1: 
            k = 0
            for k in range(m2, n1):
                P_k = laser_data[k]
                line_params_i = line_segments[i][1]
                d_i_k = distance_to_line(line_params_i, P_k)
                line_params_j = line_segments[j][1]
                d_j_k = distance_to_line(line_params_j, P_k)
                if d_i_k < d_j_k:
                   continue
                break
            n1 = k - 1
            m2 = k
        else:
            break
        line_segments[i][0] = laser_data[m1:n1]
        line_segments[i][1] = fit_line(line_segments[i][0])
        line_segments[i][2] = np.array([m1, n1])

        line_segments[j][0] = laser_data[m2:n2]
        line_segments[j][1] = fit_line(line_segments[j][0])
        line_segments[j][2] = np.array([m2, n2])
    return line_segments

def endpoint_coordinates(line_segments):
    """
    Get the endpoints of all line segments.
    """

    for line in line_segments:
        a, b, c = line[1]
        if len(line[0]) == 0:
            continue

        line_dir =  np.array([-b, a]) / np.sqrt(a**2 + b**2)
        # Project all point onto the line direction 
        projections = line[0] @ line_dir
        min_idx = np.argmin(projections) 
        max_idx = np.argmax(projections)

        x_0, y_0 = line[0][min_idx]
        x_f, y_f = line[0][max_idx]  

        x_start = (b**2 * x_0 - a * b * y_0 - a * c) / (a**2 + b**2)
        y_start = (a**2 * y_0 - a * b * x_0 - b * c) / (a**2 + b**2)

        x_end = (b**2 * x_f - a * b * y_f - a * c) / (a**2 + b**2)
        y_end = (a**2 * y_f - a * b * x_f - b * c) / (a**2 + b**2)
        line.append([np.array([x_start, y_start]), np.array([x_end, y_end])])
    return line_segments

def plot_line_segment(data, seed_data, seed_fitted_line):
    plt.scatter(data[:,0], data[:,1], c='blue')
    plt.scatter(seed_data[:,0], seed_data[:,1], c='red')
    # seed_fitted line is on the form (a, b, c), where ax + by + c = 0
    
    a, b, c = seed_fitted_line
    # Get min and max x coordinates of seed data
    x_min, x_max = np.min(seed_data[:,0]), np.max(seed_data[:,0])
    x = np.linspace(x_min, x_max, 100)
    y = -(a*x + c)/b
    plt.plot(x, y, c='green')
    plt.show()

def plot_all_line_segments(line_segments, unmatched_points):
    # For each line segment, plot its points, its fitted line segment and mark its points with a unique color. The points that are not included in any line should me marked with a cross. 

    unique_colors = plt.cm.rainbow(np.linspace(0, 1, len(line_segments)))
    for i, line_segment in enumerate(line_segments):
        if len(line_segment[0]) == 0:
            continue
        line_data, line_parameters, final_line_pos, endpoints = line_segment
        a, b, c = line_parameters
        x_min, x_max = np.min(line_data[:,0]), np.max(line_data[:,0])
        x = np.linspace(x_min, x_max, 100)
        y = -(a*x + c)/b
        plt.plot(x, y, c=unique_colors[i])
        plt.scatter(line_data[:,0], line_data[:,1], c=unique_colors[i], marker='v')

        start_point, end_point = endpoints
        plt.scatter(start_point[0], start_point[1], c='black', marker='o', s=100, facecolors='none')
        plt.scatter(end_point[0], end_point[1], c='black', marker='o', s=100, facecolors='none')
    plt.scatter(unmatched_points[:,0], unmatched_points[:,1], c='red', marker='x')
    plt.show()

def fit_line(data: np.ndarray):
    # construct A matrix
    A = np.hstack((data, np.ones((len(data), 1))))
    # Find least square solution to Ax = 0 as the parameters (a, b, c)
    # enforce norm of one for the solution, using SVD 
    U, S, Vt = np.linalg.svd(A.T @ A)
    return Vt[-1]

def get_predicted_point(line_params, current_point):
    a, b, c = line_params
    theta = np.arctan2(current_point[1], current_point[0])
    sin = np.sin(theta)
    cos = np.cos(theta)
    x = -c * cos  / (a * cos + b * sin)
    y = -c * sin / (a * cos + b * sin)
    return np.array([x, y])

def distance_to_predicted_point(line_params, current_point):
    predicted_point = get_predicted_point(line_params, current_point)
    return np.linalg.norm(predicted_point - current_point)

def distance_to_line(line_params, point):
    a, b, c = line_params
    x,y = point
    return np.abs(a*x + b*y + c) / np.sqrt(a**2 + b**2)

def feature_detection(laser_data):
    line_segments, unmatched_points = find_all_line_segments(laser_data, 0.05, 0.05, 4, 4, 0.2, 4)
    line_segments = overlap_region_processing(laser_data, line_segments)
    line_segments = endpoint_coordinates(line_segments)
    return line_segments, unmatched_points


def calculate_convexity(line_segments): 
    # Assert that there are endpoints for the lines.
    pass



if __name__ == "__main__":
    # Testing 
    # Get pointcloud from the data player 
    data_player = DataPlayer("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    for frame in range(145,200):
        laser_data, _ = data_player.get_frame(frame)
        pointcloud = lidar_to_points(laser_data)
        pointcloud = filter_points(pointcloud)

        line_segments, unmatched_points = find_all_line_segments(pointcloud, 0.05, 0.05, 4, 4, 0.2, 4)
        print(len(line_segments))
        # line_segments = overlap_region_processing(pointcloud, line_segments)
        line_segments = endpoint_coordinates(line_segments)
        plot_all_line_segments(line_segments, unmatched_points)