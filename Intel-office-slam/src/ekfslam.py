import numpy as np
import matplotlib.pyplot as plt
from icp_implementation import ICP

class EKFSLAM:
    def __init__(self):
        self.N = int(0)
        self.alpha = 1
        
        self.means = []
        self.covariances = [np.zeros((3, 3))]
        self.odometry = []
        self.icp_transforms = []
        self.icp_measurements = []
        self.predicted_measurements_icp = []
        self.innovations = []
        self.kalman_gains_icp = []
        self.H_icp = np.hstack([np.eye(3), -np.eye(3)])
        self.Q_icp = np.diag([10000, 10000, 1])
        self.Q_landmarks = np.eye(3)
        self.Mahalanobis_threshold = 1 #for outlier detection
        self.R = np.eye(6)     
        self.map = []
    
    def iteration(self, odometry_data, points_cloud1, points_cloud2, perform_icp_update = True, detected_landmarks = None):
        self.prediction_step(odometry_data)
        if perform_icp_update:
            self.icp_measurement(points_cloud1, points_cloud2)
            self.icp_update_step() 
        if detected_landmarks is not None:
            self.incremental_maximum_likelihood(detected_landmarks)
        
    def prediction_step(self, odometry_data):
        """ Performs the dynamic model of the EKFSLAM.
        Args:
            odometry_data (np.array): The odometry data.
        """
        mean = self.means[-1]
        covariance = self.covariances[-1]
        
        th = mean[2]
        
        c = np.cos(th)
        s = np.sin(th)
        delta_x = float(odometry_data[0])
        delta_y = float(odometry_data[1])
        delta_th = float(odometry_data[2])
        print("Odometry")
        print(delta_x, delta_y, delta_th)
        A = - (delta_x * s + delta_y * c)
        B = delta_x * c - delta_y * s
        
        G_small = np.array([[1, 0, A, 0, 0, 0],
                            [0, 1, B, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        #print(mean)
        G = np.zeros((6+2*self.N, 6+2*self.N)) #covariances to zero?
        G[0:6, 0:6] = G_small
        new_pose = np.zeros(6)
        new_pose[0:3] = mean[0:3]
        new_pose[3:6] = mean[0:3]
        new_pose[0] = new_pose[0] + delta_x * c - delta_y * s
        new_pose[1] = new_pose[1] + delta_y * c + delta_x * s
        new_pose[2] = new_pose[2] + delta_th
        new_covariance = G @ covariance @ G.T + self.R
        
        self.odometry.append(np.array(odometry_data, dtype=float))
        self.means.append(new_pose) #wrong
        self.covariances.append(new_covariance)
        
    def icp_measurement(self, points_cloud1, points_cloud2):
        """ Performs icp to get the rotation matrix and translation vector. Returns the homogeneus transform matrix
        Args:
            points_cloud1 (np.array) a Nx2 vector containing the lidar scans at time k-1
            points_cloud2 (np.array) a Nx2 vector containing the lidar scans at time k
        """
        icp = ICP(points_cloud2, points_cloud1, tol=1e-7, niter_max=1000)
        R, t = icp.run()
    
        M = np.array([[R[0,0], R[0,1], t[0]],
                      [R[1,0], R[1,1], t[1]],
                      [0, 0, 1]])
        self.icp_transforms.append(M)
        z = np.array([t[0], t[1], np.atan2(R[1,0], R[0,0])])
        print("ICP:")
        print(z)
        self.icp_measurements.append(z)
  
    def icp_update_step(self):
        Sigma = self.covariances[-1]
        mu = self.means[-1]
        H_small = self.H_icp
        H = np.hstack((H_small, np.zeros((3, 2*self.N))))
        Q = self.Q_icp
        K = Sigma @ H.T @ np.linalg.inv(H @ Sigma @ H.T + Q)
        self.kalman_gains_icp.append(K)
        self.predicted_measurements_icp.append(self.odometry[-1])
        eta = self.icp_measurements[-1] - self.predicted_measurements_icp[-1]
        self.innovations.append(eta)
        print(f"Innovation: {eta}")
        print(f"")
        
        mu = mu + K @ eta
        Sigma = (np.eye(6) - K @ H) @ Sigma
        
        self.means.append(mu)
        self.covariances.append(Sigma)
    
    def incremental_maximum_likelihood(self, detected_landmarks):
        print(f"Processing {len(detected_landmarks)} landmarks")
        mu = self.means[-1]
        Sigma = self.covariances[-1]
        Q = self.Q_landmarks
        j = -1 * np.ones(len(detected_landmarks))
        K_i = []
        zhat_i = []
        H_i = []
        for i in range(len(detected_landmarks)):
            z = detected_landmarks[i]
            map = self.map
            x_lm = mu[0] + z[0] * np.cos(z[1] + mu[2])
            y_lm = mu[1] + z[0] * np.sin(z[1] + mu[2])
            s = z[2]
            lm = np.array([x_lm, y_lm, s])
            map.append(lm)
            #delta = np.zeros(2, len(map))
            H_list = []
            Psi_list = []
            Mahalanobis_distances = np.zeros(len(map))
            zhat = []
            for k in range(len(map)):
                delta_x = map[0,k] - mu[0]
                delta_y = map[1,k] - mu[1]
                # delta[0,k] = delta_x
                # delta[1,k] = delta_y
                # q = delta[:,k].T @ delta[:,k]
                delta = np.array([delta_x, delta_y])
                q = np.dot(delta, delta)
                zhat.append(np.array(np.sqrt(q), np.atan2(delta_y, delta_x) - mu[2], map[2,k]))
                A = np.vstack(np.eye(3), np.zeros(3, 3))
                B = np.zeros(6, 2*k - 2 + 3)
                C = np.vstack(np.zeros(3,3), np.eye(3))
                D = np.zeros(6, 2*(self.N-k))
                Fx = np.hstack(A,B,C,D)
                H = np.array([np.sqrt(q)*delta_x, -np.sqrt(q)*delta_y, 0, -np.sqrt(q)*delta_x, np.sqrt(q)*delta_y, 0],
                             [delta_y, delta_x, -1, -delta_y, -delta_x, 0],
                             [0, 0, 0, 0, 0, 1])
                H = 1/q * H @ Fx
                H_list.append(H)
                Psi_list.append(H @ Sigma @ H.T + Q)
                Mahalanobis_distances[k] = (z - zhat[k]).T @ np.linalg.inv(Psi_list[k]) @ (z - zhat[-1])
            Mahalanobis_distances[-1] = self.alpha
            min_Mdist = 100000
            for k in range(len(map)):
                if not Mahalanobis_distances[k] > self.Mahalanobis_threshold: #Outlier detection
                    if Mahalanobis_distances[k] < min_Mdist:
                        min_Mdist = Mahalanobis_distances[k]
                        j[i] = k
            self.N = max(self.N, j[i])
            if self.N == j[i]:
                print(f"Adding landmark")
                self.map.append(lm)
            zhat_i.append(zhat[j])
            H_i.append(H_list[j[i]][:, 0:self.N])
            Psi_i = Psi_list[j[i]]
            K_i.append(Sigma @ H_i[i].T @ np.inv(Psi_i))

        update_mean = np.zeros(len(mu))
        update_cov = np.zeros(np.size(Sigma))
        for i in range(len(detected_landmarks)):
            delta_mean = K_i[i] @ (zhat_i[i] - detected_landmarks[i])
            delta_cov = K_i[i]@H_i[i]
            if len(delta_mean) > len(update_mean):
                np.append(update_mean, 0)
                np.append(mu, 0)
            if np.size(delta_cov) > np.size(update_cov):
                # Append a column of zeros
                update_cov = np.hstack(( update_cov, np.zeros((np.size(update_cov, 0), 1)) ))
                Sigma = np.hstack(( Sigma, np.zeros((np.size(Sigma, 0), 1)) ))
                # Append a row of zeros
                update_cov = np.vstack(( update_cov, np.zeros((1, np.size(update_cov, 1))) ))
                Sigma = np.vstack(( Sigma, np.zeros((1, np.size(Sigma, 1))) ))
            update_mean = update_mean + delta_mean
            update_cov = update_cov + delta_cov
        
        self.means[-1] = mu + update_mean
        self.covariances[-1] = ( np.eye(np.size(Sigma, 0)) - update_cov ) @ Sigma
        
    def create_file(self, odometry_filename = "odometry.txt", estpose_filename = "pose.txt"):
        # Save odometry data
        odom_pose = np.zeros(3)
        with open(odometry_filename, "w") as odom_file:
            for odom_data in self.odometry:
                # Assuming odom is a numpy array or tuple with numeric values
                odom_pose[0] = odom_pose[0] + odom_data[0] * np.cos(odom_pose[2]) - odom_data[1] * np.sin(odom_pose[2])
                odom_pose[1] = odom_pose[1] + odom_data[1] * np.cos(odom_pose[2]) + odom_data[0] * np.sin(odom_pose[2])
                odom_pose[2] = odom_pose[2] + odom_data[2]
                odom_pose[2] = (odom_pose[2] + np.pi) % (2 * np.pi) - np.pi
                odom_file.write(f"{odom_pose[0]:.6f} {odom_pose[1]:.6f} {odom_pose[2]:.6f}\n")

        # Save pose data (x, y, theta)
        with open(estpose_filename, "w") as pose_file:
            for pose in self.means:
                # Assuming pose is a numpy array or tuple with at least 3 elements
                pose[2] = (pose[2] + np.pi) % (2 * np.pi) - np.pi
                pose_file.write(f"{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f}\n")
    
    def plot_results(self):
        # Process odometry data to compute the trajectory
        odom_pose = np.zeros(3)
        odom_trajectory = [odom_pose[:2].copy()]
        for odom_data in self.odometry:
            odom_pose[0] += odom_data[0] * np.cos(odom_pose[2]) - odom_data[1] * np.sin(odom_pose[2])
            odom_pose[1] += odom_data[1] * np.cos(odom_pose[2]) + odom_data[0] * np.sin(odom_pose[2])
            odom_pose[2] = (odom_pose[2] + odom_data[2] + np.pi) % (2 * np.pi) - np.pi
            odom_trajectory.append(odom_pose[:2].copy())
        odom_trajectory = np.array(odom_trajectory)

        # Extract poses for plotting
        pose_x = np.zeros(len(self.means))
        pose_y = np.zeros(len(self.means))
        for i in range(len(self.means)):
            mean = self.means[i]
            pose_x[i], pose_y[i] = mean[0], mean[1]
        plt.ioff()
        # Plot trajectories
        plt.figure()
        plt.plot(odom_trajectory[:, 0], odom_trajectory[:, 1], label="Odometry Trajectory", marker='o')
        plt.plot(pose_x, pose_y, label="Pose Data", marker='x', linestyle='--')

        # Add labels, legend, and grid
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Odometry and Pose Data Trajectory")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
        

    def plot_results_from_file(self, odom_file="odometry.txt", pose_file="pose.txt"):
        """
        Reads odometry and pose data from files and plots the trajectories.

        Parameters:
        - odom_file: Path to the file containing odometry data (default: "odometry.txt").
        - pose_file: Path to the file containing pose data (default: "pose.txt").
        """
        # Load odometry data
        try:
            odometry_data = np.loadtxt(odom_file)
            odom_x, odom_y = odometry_data[:, 0], odometry_data[:, 1]
        except Exception as e:
            print(f"Error reading odometry data from {odom_file}: {e}")
            return

        # Load pose data
        try:
            pose_data = np.loadtxt(pose_file)
            pose_x, pose_y = pose_data[:, 0], pose_data[:, 1]
        except Exception as e:
            print(f"Error reading pose data from {pose_file}: {e}")
            return
        plt.ioff()
        # Plot trajectories
        plt.figure(figsize=(10, 6))
        plt.plot(odom_x, odom_y, label="Odometry Trajectory", marker='o')
        plt.plot(pose_x, pose_y, label="Pose Data", marker='x', linestyle='--')

        # Add labels, legend, and grid
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Odometry and Pose Data Trajectory")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
