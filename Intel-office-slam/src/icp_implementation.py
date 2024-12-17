import numpy as np
from play_data import DataPlayer
from icp_wrapper import lidar_to_points
from plot import plot_icp_transform

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
        err = self.tol + 1
        niter = 0
        while err > self.tol and niter < self.niter_max:
            self.find_correspondences()
            self.find_closest_index()
            self.find_mean_and_covariance_matrix()
            self.find_rotation_and_translation()
            err = np.linalg.norm( self.current_points - self.target_points)
            niter += 1
        
        return self.Rot, self.Trans
        
    def find_correspondences(self) -> None:
        """
        Finds corresponding points between current and target point clouds
        using nearest neighbor search. Stores results in self.cores matrix:
        - cores[0,i]: index in current points
        - cores[1,i]: index in target points
        - cores[2,i]: distance between corresponding points
        """
        N = self.current_points.shape[0]
        
        for i in range(N):
            min_dist = np.inf
            idx = -1
            for j in range(N):
                dist = np.linalg.norm( self.target_points[i,:] - self.current_points[j,:] )
                if dist < min_dist:
                    min_dist = dist
                    idx = j
        
            self.cores[0, i] = i
            self.cores[1, i] = idx
            self.cores[2, i] = min_dist
        
        
    def find_closest_index(self) -> None:
        """
        Removes duplicate correspondences by keeping only the closest matches.
        If multiple points map to the same target point, only the closest one is kept.
        """
        N = self.cores.shape[1]
        for i in range(N):
            for j in range(N):
                if i != j  and self.cores[0, i] != -1 and self.cores[1, i] != -1:
                    if self.cores[1, i] == self.cores[1, j]:
                        if self.cores[2, i] <= self.cores[2,j]:
                            self.cores[1,j] = -1
                            self.cores[2,j] = -1
                        else:
                            self.cores[1,i] = -1
                            self.cores[2,i] = -1
                            
    def find_mean_and_covariance_matrix(self) -> None:
        """
        Computes the mean points and covariance matrix of corresponding points.
        These are used to find the optimal rotation and translation.
        """
        cnt_cores = 0        
        self.Sigma = np.zeros((2,2))
        self.curr_mean = np.zeros(2)
        self.tar_mean = np.zeros(2)
        
        N = len(self.cores[1])
        
        for i in range(N):
            if not self.cores[0,i] == -1 and not self.cores[1,i] == -1:
                Sigma_new = ( self.target_points[int(self.cores[1,i]),:].T @ self.current_points[int(self.cores[0,i]),:] )
                self.Sigma = self.Sigma + Sigma_new
                self.curr_mean = self.curr_mean + self.current_points[int(self.cores[0,i]),:]
                self.tar_mean = self.tar_mean + self.target_points[int(self.cores[1,i]),:]
                cnt_cores += 1
        
        self.curr_mean = self.curr_mean / cnt_cores
        self.tar_mean = self.tar_mean / cnt_cores
    
    def find_rotation_and_translation(self) -> None:
        """
        Computes optimal rotation and translation using SVD of covariance matrix.
        Updates the current points using the accumulated transformation.
        """
        U, _, V = np.linalg.svd(self.Sigma)
        D = np.eye(2)
        if np.linalg.det(U @ V) < 0:
            D[-1, -1] = -1
        Rot_temp = U @ D @ V
        #Rot_temp = U @ V
        self.Rot = Rot_temp @ self.Rot
        
        Trans_temp = self.tar_mean - Rot_temp @ self.curr_mean
        self.Trans = self.Trans + Trans_temp
        
        # for i in range(self.target_points.shape[0]):
        #     #self.target_points[i,:] = self.Rot @ self.target_points[i,:] + self.Trans
        # self.current_points = (self.Rot @ self.current_points.T)
        # self.current_points = self.current_points.T + self.Trans
        self.current_points = (self.reference_points @ self.Rot.T) + self.Trans
        
        
if __name__=='__main__':
    data_player = DataPlayer("../dataset_intel/intel_LASER_.txt", "../dataset_intel/intel_ODO.txt")
    frame = 150
    laser_data1, _ = data_player.get_frame(frame)
    laser_data2, _ = data_player.get_frame(frame+1)
    pc1 = lidar_to_points(laser_data1)
    pc2 = lidar_to_points(laser_data2)
    
    # Do icp and transform ...
    icp = ICP(pc1, pc2)
    R, t = icp.run()
    
    print(R)
    print(t)
    
    pc1_transformed = np.zeros((pc1.shape[0], 2))
    for i in range(len(pc1)):
        pc1_transformed[i,:] = R @ pc1[i,:] + t
    
    plot_icp_transform(pc1, pc2)
    plot_icp_transform(pc1, pc1_transformed)
    
    
    
    