import numpy as np
import matplotlib.pyplot as plt
from icp_implementation import ICP

class EKFSLAM:
    def __init__(self):
        self.N = int(0)
        self.alpha = 0.2
        
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
    
    def iteration(self, odometry_data, points_cloud1, points_cloud2, perform_icp_update = True, detected_landmarks = None):
        self.prediction_step(odometry_data)
        if perform_icp_update:
            self.icp_measurement(points_cloud1, points_cloud2)
            self.icp_update_step() 
        if detected_landmarks is not None:
            if len(detected_landmarks) != 0:
                print(f"detected landmarks: {detected_landmarks}")
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
        #print(f"Odometry: {delta_x, delta_y, delta_th}")
        #print(delta_x, delta_y, delta_th)
        A = - (delta_x * s + delta_y * c)
        B = delta_x * c - delta_y * s
        Fx = np.hstack([np.eye(6), np.zeros((6, 3 * self.N))])
        
        G_small = np.array([[1, 0, A, 0, 0, 0],
                            [0, 1, B, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        
        #print(f"N = {self.N}")
        #G = np.zeros((6+3*self.N, 6+3*self.N))
        G = np.eye(6+3*self.N)
        G[0:6, 0:6] = G_small
        new_pose = np.zeros(6)
        new_pose[0:3] = mean[0:3]
        new_pose[3:6] = mean[0:3]
        new_pose[0] = new_pose[0] + delta_x * c - delta_y * s
        new_pose[1] = new_pose[1] + delta_y * c + delta_x * s
        new_pose[2] = new_pose[2] + delta_th
        # print(f"Sigma = {covariance}]")
        # print(f"G = {G}]")
        new_covariance = G @ covariance @ G.T + Fx.T @ self.R @ Fx
        
        self.odometry.append(np.array(odometry_data, dtype=float))
        self.means.append(self.means[-1])
        self.means[-1][0:6] = new_pose
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
        #print(f"ICP: {z}")
        self.icp_measurements.append(z)
  
    def icp_update_step(self):
        Sigma = self.covariances[-1]
        mu = self.means[-1]
        H_small = self.H_icp
        H = np.hstack((H_small, np.zeros((3, 3*self.N))))
        #Fx = np.hstack([np.eye(3), np.zeros((3, 3 + 3 * self.N))])
        Q = self.Q_icp
        # print(f"H: {H}")
        # print(f"Sigma: {Sigma}")
        # print(f"Q: {Q}")
        K = Sigma @ H.T @ np.linalg.inv(H @ Sigma @ H.T + Q)
        self.kalman_gains_icp.append(K)
        self.predicted_measurements_icp.append(self.odometry[-1])
        eta = self.icp_measurements[-1] - self.predicted_measurements_icp[-1]
        self.innovations.append(eta)
        
        mu = mu + K @ eta
        Sigma = (np.eye(len(mu)) - K @ H) @ Sigma
        
        self.means.append(mu)
        self.covariances.append(Sigma)
    
    def incremental_maximum_likelihood(self, detected_landmarks):
        print(f"Processing {len(detected_landmarks)} landmarks")
        mu = self.means[-1]
        Sigma = self.covariances[-1]
        Q = self.Q_landmarks
        j = -1 * np.ones(len(detected_landmarks), dtype=int)
        K_i = []
        zhat_i = []
        H_i = []
        for i in range(len(detected_landmarks)):
            z = detected_landmarks[i]
            map = self.means[-1][6:]
            cos = np.cos(z[1] + mu[2])
            sin = np.sin(z[1] + mu[2])
            x_lm = mu[0] + z[0] * cos
            y_lm = mu[1] + z[0] * sin
            s = z[2]
            lm = np.array([x_lm, y_lm, s])
            # Provisional extended state
            # print(f"map: {map}")
            # print(f"lm : {lm}")
            map = np.hstack((map, lm))
            N = int(len(map)/3)
            # Provisional extended covariance
            G_z = np.array([[cos, -z[0]*sin, 0],
                            [sin, z[0]*cos, 0],
                            [0, 0, 1]])
            G_R = np.array([[1, 0, -z[0]*sin, 0, 0, 0],
                            [0, 1, z[0]*cos, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0]])
            Sigma_LL = G_R @ Sigma[0:6, 0:6] @ G_R.T + G_z @ Q @ G_z.T
            Sigma_Lmu = G_R @ Sigma[0:6, :]
            Sigma_right = np.vstack((Sigma_Lmu.T, Sigma_LL))
            Sigma_prov = np.vstack((Sigma, Sigma_Lmu))
            Sigma_prov = np.hstack((Sigma_prov, Sigma_right))
            #delta = np.zeros(2, N)
            H_list = []
            Psi_list = []
            Mahalanobis_distances = np.zeros(N)
            zhat = []
            #print(f"Vector: {range(N)}")
            for k in range(N):
                # print(f"k: {k}")
                # print(f"Signature: {map[k+2]}")
                delta_x = map[k] - mu[0]
                delta_y = map[k+1] - mu[1]
                delta = np.array([delta_x, delta_y])
                q = np.dot(delta, delta)
                zhat.append(np.array([np.sqrt(q), np.atan2(delta_y, delta_x) - mu[2], map[k+2]]))
                A = np.vstack([np.eye(3), np.zeros((3, 3))])
                B = np.zeros((6, 3*k + 3))
                C = np.vstack([np.zeros((3,3)), np.eye(3)])
                D = np.zeros((6, 3*(N-k-1)))
                Fx = np.hstack([A,B,C,D])
                #print(Fx)
                H = np.array([[np.sqrt(q)*delta_x, -np.sqrt(q)*delta_y, 0, -np.sqrt(q)*delta_x, np.sqrt(q)*delta_y, 0],
                             [delta_y, delta_x, -1, -delta_y, -delta_x, 0],
                             [0, 0, 0, 0, 0, 1]])
                H = 1/q * H @ Fx
                H_list.append(H)
                # print(f"map: {map}")
                # print(f"H: {H}")
                # print(f"Sigma_prov: {Sigma_prov}")              
                Psi_list.append(H @ Sigma_prov @ H.T + Q)
                Mahalanobis_distances[k] = (z - zhat[k]).T @ np.linalg.inv(Psi_list[k]) @ (z - zhat[k])
            Mahalanobis_distances[-1] = self.alpha
            min_Mdist = 100000
            for k in range(N):
                if not Mahalanobis_distances[k] > self.Mahalanobis_threshold: #Outlier detection
                    if Mahalanobis_distances[k] < min_Mdist:
                        min_Mdist = Mahalanobis_distances[k]
                        j[i] = k
            # print(f"Before assigning: N = {N}, N_global = {self.N}, j = {j[i]}")
            # self.N = max(self.N, j[i]+1)
            # print(f"After assigning: N = {self.N}")
            # if self.N == j[i]+1:
            if j[i] + 1 > self.N:
                print(f"Adding landmark to the state")
                self.N = j[i] + 1
                # extend the state
                mu = np.hstack([mu, lm])             
                # extend the covariance matrix
                Sigma = Sigma_prov
                # update the global variables
                self.means[-1] = mu
                self.covariances[-1] = Sigma
            # print(f"After assigning: N = {self.N}")
            # print(f"map: {self.means[-1][6:]}")
                
            zhat_i.append(zhat[j[i]])
            #print(f"Before appending: {H_list[j[i]]}")
            H_i.append(H_list[j[i]][:, 0:(6 + 3 * self.N)])
            #print(f"After appending: {H_i[i]}")
            Psi_i = H_i[i] @ Sigma @ H_i[i].T + Q
            # print(f"Sigma: {Sigma}")
            # print(f"H_i: {H_i[i]}")
            K_i.append(Sigma @ H_i[i].T @ np.linalg.inv(Psi_i))
            #print(f"K_i = {K_i[i]}")
        
        update_mean = np.zeros((len(mu)))
        update_cov = np.zeros(Sigma.shape)
        
        for i in range(len(detected_landmarks)):
            sz = np.size(K_i[i], 0)
            # print(f"mu: {mu}, {len(mu)=}")
            
            # print(f"sz : {sz}")
            # print(f"Result of te operation: {K_i[i] @ (detected_landmarks[i] - zhat_i[i])}")
            # print(f"Left side: {update_mean[0:sz]}")
            update_mean[0:sz] += K_i[i] @ (detected_landmarks[i] - zhat_i[i])
            #print(f"K_i: {K_i[i]}")
            #print(f"H_i: {H_i[i]}")
            #print(f"Sigma: {Sigma}")
            #print(f"map: {map}")
            #print(f"update_cov: {update_cov}")
            # print(f"i = {i}")
            # print(f"K_i: {K_i[i]}")
            update_cov[0:sz, 0:sz] += K_i[i] @ H_i[i]
        
        self.means.append(mu + update_mean)
        print(f"Update: {self.means[-1][0:3]}")
        print(f"Map: {self.means[-1][3:]}")
        #print(f"Sigma: {Sigma}")
        self.covariances.append( ( np.eye(np.size(Sigma, 0)) - update_cov ) @ Sigma )
        #print(f"Update Cov: {self.covariances[-1]}")
    
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