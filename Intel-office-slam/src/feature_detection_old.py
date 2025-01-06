import numpy as np
import matplotlib.pyplot as plt
from play_data import DataPlayer
from icp_wrapper import lidar_to_points
from filtering import filter_points
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans

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
    if len(pointcloud) == 0:
        return []
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

    return np.array(line_parameters)

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
        if params is None:  # Stop if no more points to process
            break
        line_params.append(params)
    return np.array(line_params)  # Convert to numpy array at the end

def _point_line_detection_iteration(pointcloud: np.ndarray, line_params: list):
    """
    Detect the line parameters for the pointcloud
    """
    if len(pointcloud) == 0:
        return None, pointcloud
        
    old_pointcloud = pointcloud
    params = hough_transform(pointcloud, nlines=1)
    if len(params) == 0:  # No lines found
        return None, pointcloud
    pointcloud = remove_points_on_line(pointcloud, params[0])  # Use params[0] directly
    return params[0], pointcloud  # Return the first (and only) line parameters

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

def find_normal_vectors_for_points(pointcloud: np.ndarray):
    """
    Find the normal vectors for the points
    """
    normal_vectors = []
    # Take the k nearest neighbors and perform a PCA to find the normal vector
    k = 4
    for point in pointcloud:
        distances = np.linalg.norm(pointcloud - point, axis=1)
        indices = np.argsort(distances)[:k]
        neighbors = pointcloud[indices]
        pca = PCA(n_components=2)
        pca.fit(neighbors)
        normal_vectors.append(pca.components_[1] / np.linalg.norm(pca.components_[1]))

    return np.array(normal_vectors)

def calculate_flatness(cluster: np.ndarray):
    """
    Calculate the flatness of the explained variances
    """
    if len(cluster) <= 2:
        return 10
    pca = PCA(n_components=2)
    pca.fit(cluster)

    # Compute the eigenvalues of the clusters.T @ clusters
    eigenvalues = np.linalg.eigvals(cluster.T @ cluster)
    # Return the ratio of the second largest eigenvalue to the largest eigenvalue
    return (np.min(eigenvalues) / np.max(eigenvalues)) ** 2

def calculate_explained_variance(data: np.ndarray, labels: np.ndarray, centers: np.ndarray) -> float:
    """
    Calculate the explained variance ratio for clustering
    
    Args:
        data: Input data array of shape (n_samples, n_features)
        labels: Cluster labels for each point
        centers: Cluster centroids of shape (n_clusters, n_features)
    
    Returns:
        float: Explained variance ratio between 0 and 1
    """
    # Calculate total variance (sum of squared distances to overall mean)
    print(data.shape)
    overall_mean = np.mean(data[:, :], axis=0)
    total_variance = np.sum((data[:, :] - overall_mean) ** 2)
    
    # Calculate within-cluster variance
    within_variance = np.sum([np.sum((data[labels == i] - centers[i])**2) 
                            for i in range(len(centers))])
    
    # Calculate explained variance ratio
    explained_variance = 1 - (within_variance / total_variance)
    return explained_variance

def choose_optimal_k(errors: list, tol: float=0.15):
    """
    Choose the optimal k using the elbow method
    """
    # Choose the value corresponding to the number coming after the largest drop in flatness    
    changes = np.diff(errors)
    optimal_k = np.argmin(changes)
    return optimal_k + 2

def cluster_normal_vectors(pointcloud: np.ndarray, normal_vectors: np.ndarray):
    """
    Cluster the normal vectors using k-means
    """
    augmented_vectors = np.hstack((pointcloud,  normal_vectors))
    explained_variances = []
    errors = []
    for k in range(1, 10):
        kmeans = KMeans(n_clusters=k)
        kmeans.fit(augmented_vectors)
        explained_variances.append(calculate_explained_variance(augmented_vectors, 
                                                             kmeans.labels_, 
                                                             kmeans.cluster_centers_))
    
        error = sum([calculate_flatness(augmented_vectors[kmeans.labels_ == j, :2]) for j in range(k)])
        errors.append(error)
    # Plot explained variance vs number of clusters
    plt.figure()
    print(np.array(errors).shape)
    plt.plot(range(1,10), errors, 'bo-')
    plt.xlabel('Number of Clusters (k)')
    plt.ylabel('Flatness Error')
    plt.title('Flatness Error vs Number of Clusters')
    plt.grid(True)
    plt.show()
    # Choose best k and fit final model
    best_k = choose_optimal_k(errors)
    kmeans = KMeans(n_clusters=best_k)
    kmeans.fit(augmented_vectors)
    return kmeans.labels_, kmeans.cluster_centers_

def plot_normal_vectors(pointcloud: np.ndarray, normal_vectors: np.ndarray, labels: np.ndarray, centers: np.ndarray):
    """
    Plot the pointcloud and the normal vectors
    """
    plt.scatter(pointcloud[:, 0], pointcloud[:, 1], c=labels)
    plt.quiver(pointcloud[:, 0], pointcloud[:, 1], normal_vectors[:, 0], normal_vectors[:, 1], angles='xy', scale_units='xy', scale=5, width=0.0007, headwidth=0.3)
    # Mark the cluster centers with a large cross in its color and draw the average normal vector of that cluster from that point
    for i in range(len(centers)):
        plt.scatter(centers[i, 0], centers[i, 1], marker='x', s=100, c=i)
        plt.quiver(centers[i, 0], centers[i, 1], centers[i, 2], centers[i, 3], angles='xy', scale_units='xy', scale=1, width=0.007, headwidth=0.3)
    plt.show()

if __name__ == "__main__":
    # Get pointcloud from the data player 
    data_player = DataPlayer("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    for frame in range(50,100):
        laser_data, _ = data_player.get_frame(frame)
        pointcloud = lidar_to_points(laser_data)
        pointcloud = filter_points(pointcloud)
        # line_parameters = iterative_line_detection(pointcloud, nlines=3)
        # corners = detect_corner(line_parameters) 
        # plot_points_and_hough_lines(pointcloud, line_parameters, corners)
        normal_vectors = find_normal_vectors_for_points(pointcloud)
        labels, centers = cluster_normal_vectors(pointcloud, normal_vectors)
        plot_normal_vectors(pointcloud, normal_vectors, labels, centers)

        # plot_points_and_hough_lines(pointcloud, [])
