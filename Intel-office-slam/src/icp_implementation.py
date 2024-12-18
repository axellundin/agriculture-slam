import numpy as np
from play_data import DataPlayer
from icp_wrapper import lidar_to_points
from plot import plot_icp_transform
from scipy.spatial import cKDTree

class ICP:
    """
    Implements the Iterative Closest Point algorithm for point cloud registration.
    Finds the optimal rigid transformation (rotation and translation) between two point sets.
    """
    def __init__(self, reference_points: np.ndarray, target_points: np.ndarray, 
                 tol: float = 1e-1, niter_max: int = 100) -> None:
        """
        Initialize ICP with two point clouds to be aligned.
        
        Args:
            reference_points: Source point cloud to be transformed
            target_points: Target point cloud to align with
            tol: Convergence tolerance
            niter_max: Maximum number of iterations
        """
        N = target_points.shape[0]
        self.cores = np.zeros((3, N))
        self.Rot = np.eye(2)
        self.Trans = np.zeros(2)
        self.reference_points = reference_points
        self.current_points = reference_points
        self.target_points = target_points
        self.tol = tol
        self.niter_max = niter_max
    
    def run(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Main ICP loop that iteratively finds the best transformation.
        Returns the final rotation matrix and translation vector.
        """
        prev_error = float('inf')
        niter = 0
        while niter < self.niter_max:
            self.find_correspondences()
            self.find_closest_index()
            self.find_mean_and_covariance_matrix()
            self.find_rotation_and_translation()
            
            # Calculate error based on corresponding points, not all points
            valid_mask = (self.cores[0] != -1) & (self.cores[1] != -1)
            valid_source_indices = self.cores[0, valid_mask].astype(int)
            valid_target_indices = self.cores[1, valid_mask].astype(int)
            
            current_error = np.mean(np.linalg.norm(
                self.current_points[valid_source_indices] - 
                self.target_points[valid_target_indices], axis=1))
            
            # Check for convergence
            if abs(prev_error - current_error) < self.tol:
                break
            
            prev_error = current_error
            niter += 1
        
        return self.Rot, self.Trans
        
    def find_correspondences(self) -> None:
        """
        Finds correspondences between current and target point clouds 
        using KD-tree for nearest neighbor search.
        """
        tree = cKDTree(self.target_points)
        distances, indices = tree.query(self.current_points, k=1)
        
        # Filter out correspondences that are too far (outliers)
        max_distance = np.mean(distances) + 2 * np.std(distances)
        valid_mask = distances <= max_distance
        
        N = self.current_points.shape[0]
        self.cores = np.zeros((3, N))
        self.cores[0, :] = np.arange(N)
        self.cores[1, :] = np.where(valid_mask, indices, -1)
        self.cores[2, :] = np.where(valid_mask, distances, -1)

    def find_closest_index(self) -> None:
        """
        Finds the closest correspondences by removing duplicates.
        If multiple points map to the same target point, only the closest one is kept.
        """
        # Get unique target indices and their first occurrences
        unique_targets, unique_indices = np.unique(self.cores[1], return_index=True)
        
        # Create a mask for valid entries (not -1)
        valid_mask = self.cores[1] != -1
        
        # For each unique target, find the correspondence with minimum distance
        for target in unique_targets:
            if target == -1:
                continue
            
            # Find all correspondences to this target
            target_mask = (self.cores[1] == target) & valid_mask
            if np.sum(target_mask) > 1:
                # Get distances for these correspondences
                distances = self.cores[2, target_mask]
                # Find the index of minimum distance
                min_idx = np.where(target_mask)[0][np.argmin(distances)]
                
                # Mark all other correspondences as invalid
                invalid_mask = target_mask.copy()
                invalid_mask[min_idx] = False
                self.cores[1:, invalid_mask] = -1
    
    def find_mean_and_covariance_matrix(self) -> None:
        """
        Computes the mean points and covariance matrix of corresponding points.
        These are used to find the optimal rotation and translation.
        """
        # Get valid correspondences
        valid_mask = (self.cores[0] != -1) & (self.cores[1] != -1)
        valid_source_indices = self.cores[0, valid_mask].astype(int)
        valid_target_indices = self.cores[1, valid_mask].astype(int)
        
        # Compute means
        self.curr_mean = np.mean(self.current_points[valid_source_indices], axis=0)
        self.tar_mean = np.mean(self.target_points[valid_target_indices], axis=0)
        
        # Center the points
        curr_centered = self.current_points[valid_source_indices] - self.curr_mean
        tar_centered = self.target_points[valid_target_indices] - self.tar_mean
        
        # Compute covariance matrix
        self.Sigma = tar_centered.T @ curr_centered

    def find_rotation_and_translation(self) -> None:
        """
        Computes optimal rotation and translation using SVD of covariance matrix.
        Updates the current points using the accumulated transformation.
        """
        U, S, V = np.linalg.svd(self.Sigma)
        
        # Ensure proper rotation matrix (handle reflection case)
        det = np.linalg.det(U @ V)
        D = np.eye(2)
        if det < 0:
            D[-1, -1] = -1
        
        Rot_temp = U @ D @ V
        self.Rot = Rot_temp @ self.Rot
        
        Trans_temp = self.tar_mean - Rot_temp @ self.curr_mean
        self.Trans = Trans_temp + self.Trans
        
        # Update current points
        self.current_points = (self.reference_points @ self.Rot.T) + self.Trans
        
if __name__=='__main__':
    data_player = DataPlayer("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    frame = 80
    laser_data1, _ = data_player.get_frame(frame)
    laser_data2, _ = data_player.get_frame(frame+1)
    pc1 = lidar_to_points(laser_data1)
    pc2 = lidar_to_points(laser_data2)
    
    # Do icp and transform ...
    icp = ICP(pc1, pc2, tol=1e-1, niter_max=100)
    R, t = icp.run()
    
    print(R)
    print(t)
    
    pc1_transformed = np.zeros((pc1.shape[0], 2))
    for i in range(len(pc1)):
        pc1_transformed[i,:] = R @ pc1[i,:] + t
    
    plot_icp_transform(pc1, pc2)
    plot_icp_transform(pc1_transformed, pc2)
    
    
    
    