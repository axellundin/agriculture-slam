import numpy as np
import matplotlib.pyplot as plt
from play_data import DataPlayer
from icp_wrapper import lidar_to_points

def detect_corner(line_params: np.ndarray)-> list:
    """
    Detect all lines in the pointcloud using hough transform
    """
    # for each pair of lines, calculate the intersection point  
    corners = []
    for i in range(len(line_params)):
        for j in range(i+1, len(line_params)):
            theta1, rho1 = line_params[i]
            theta2, rho2 = line_params[j]
            intersection = calculate_intersection(theta1, rho1, theta2, rho2)
            if intersection: 
                corners.append(intersection)
    return corners

def calculate_intersection(theta1, rho1, theta2, rho2):
    """
    Calculate the intersection point of two lines
    """
    # check if lines parallel
    if np.abs(np.cos(theta1) * np.sin(theta2) - np.cos(theta2) * np.sin(theta1)) < 1e-6:
        return None
    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c2 = np.cos(theta2)
    s2 = np.sin(theta2)
    
    x = (rho2 * s1 - rho1 * s2) / (s1 * c2 - s2 * c1)
    y = (rho1 * c2 - rho2 * c1) / (s1 * c2 - s2 * c1)
    if np.abs(x) > 10 or np.abs(y) > 10:
        return None
    return [x, y]

def hough_transform(pointcloud: np.ndarray, n_theta: int=300, n_rho: int=180, nlines: int=3, plot: bool=False):
    """
    Hough transform for line detection
    """
    # find maximum distance between points in the point cloud
    max_distance = np.max(np.linalg.norm(pointcloud, axis=1))

    theta = np.linspace(-np.pi/2 + 1e-6, np.pi/2 - 1e-6, n_theta)
    rho = np.linspace(-max_distance, max_distance, n_rho)
    theta_rho_map = np.zeros((n_theta, n_rho))
    rho_step = 2 * max_distance / n_rho
    
    for point_idx in range(len(pointcloud)):
        x, y = pointcloud[point_idx,0], pointcloud[point_idx,1]
        if x == 0 and y == 0:
            continue
        for theta_idx, theta_val in enumerate(theta):
            rho_val = x * np.cos(theta_val) + y * np.sin(theta_val)
            rho_idx = int((rho_val + max_distance) / rho_step)
            if 0 <= rho_idx < n_rho:
                theta_rho_map[theta_idx, rho_idx] += 1
   
    # Find the local maxima in the theta_rho_map
    local_maxima = np.array(find_local_maxima(theta_rho_map, n_maxima=nlines))
    # Construct list of line parameter pairs corresponding to the local maxima
    # plot the theta_rho_map and mark out the local maxima
    if plot:
        plt.imshow(theta_rho_map, cmap='gray')
        plt.colorbar()
        plt.scatter(local_maxima[:, 1], local_maxima[:, 0], color='red', marker='x')
        plt.show()
    
    line_parameters = []
    for theta_i, rho_i in local_maxima:
        line_parameters.append((theta[theta_i], rho[rho_i]))

    return line_parameters

def plot_points_and_hough_lines(pointcloud: np.ndarray, line_parameters: list, corners: list=[]):
    """
    Plot the points and the hough lines
    """
    plt.scatter(pointcloud[:, 0], pointcloud[:, 1], s=10, marker='x')
    # plot the corners
    if len(corners) > 0:
        corners = np.array(corners)
        plt.scatter(corners[:, 0], corners[:, 1], s=10, marker='o', color='red')
    for theta, rho in line_parameters:
        # Create points along the line using parametric form
        t = np.linspace(-10, 10, 100)  # Parameter for line equation
        x = -t * np.sin(theta) + rho * np.cos(theta)  # Parametric x
        y = t * np.cos(theta) + rho * np.sin(theta)   # Parametric y
        plt.plot(x, y, 'r-', linewidth=0.5)
    plt.show()

def find_local_maxima(theta_rho_map: np.ndarray, n_maxima: int=10):
    """
    Find the local maxima in the theta_rho_map
    """
    kernel_size = 3
    local_maxima = []
    n_theta, n_rho = theta_rho_map.shape
    
    # Add padding to handle boundaries
    padded_map = np.pad(theta_rho_map, kernel_size, mode='constant', constant_values=0)
    
    # Check each point against its neighborhood
    for i in range(kernel_size, n_theta + kernel_size):
        for j in range(kernel_size, n_rho + kernel_size):
            # Extract neighborhood
            neighborhood = padded_map[i-kernel_size:i+kernel_size+1, 
                                   j-kernel_size:j+kernel_size+1]
            center_value = neighborhood[kernel_size, kernel_size]
            
            # Remove center value for comparison
            neighborhood[kernel_size, kernel_size] = -np.inf
            
            # Check if center is greater than all neighbors
            if center_value > np.max(neighborhood):
                # Convert back to original coordinates
                local_maxima.append((i-kernel_size, j-kernel_size))
    
    # Only keep the n_maxima local maxima
    local_maxima = sorted(local_maxima, 
                         key=lambda x: theta_rho_map[x[0], x[1]], 
                         reverse=True)[:n_maxima]
    
    return local_maxima

def iterative_line_detection(pointcloud: np.ndarray, nlines: int=3):
    """
    Iteratively detect one line at a time and remove the points that are on the line before the next iteration
    """
    line_params = []
    for i in range(nlines):
        params, pointcloud = _point_line_detection_iteration(pointcloud, line_params)
        line_params.append(params)
    return line_params

def _point_line_detection_iteration(pointcloud: np.ndarray, line_params: list):
    """
    Detect the line parameters for the pointcloud
    """
    old_pointcloud = pointcloud
    params = hough_transform(pointcloud, nlines=1)[0]
    pointcloud = remove_points_on_line(pointcloud, params)
   # plot_points_and_hough_lines(old_pointcloud, [params])
    return params, pointcloud

def remove_points_on_line(pointcloud: np.ndarray, line_params: list):
    """
    Remove the points that are close enough to the line
    """
    theta, rho = line_params[0], line_params[1]
    normal = np.array([np.cos(theta), np.sin(theta)])
    # One point that lies on the line
    px = rho * np.cos(theta)
    py = rho * np.sin(theta)
    p = np.array([px, py])
    # Remove the points that are close enough to the line
    dist = np.abs(np.dot(pointcloud - p, normal))
    pointcloud = pointcloud[dist > 0.5]
    return pointcloud

def find_corners_in_frame(pointcloud: np.ndarray, nlines: int=3):
    """
    Find the corners in the pointcloud
    """
    line_parameters = iterative_line_detection(pointcloud, nlines=nlines)
    corners = detect_corner(line_parameters) 
    return corners, line_parameters

if __name__ == "__main__":
    # Get pointcloud from the data player 
    data_player = DataPlayer("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    for frame in range(50,100):
        laser_data, _ = data_player.get_frame(frame)
        pointcloud = lidar_to_points(laser_data)
        line_parameters = iterative_line_detection(pointcloud, nlines=3)
        corners = detect_corner(line_parameters) 
        plot_points_and_hough_lines(pointcloud, line_parameters, corners)

        # plot_points_and_hough_lines(pointcloud, [])
