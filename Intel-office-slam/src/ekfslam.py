import numpy as np
import matplotlib.pyplot as plt
from icp_implementation import ICP

class EKFSLAM:
    def __init__(self):
        self.N = int(0)
        self.alpha = 15
        
        self.means = []
        self.covariances = [np.zeros((3, 3))]
        self.odometry = []
        self.icp_transforms = []
        self.icp_measurements = []
        self.predicted_measurements_icp = []
        self.innovations = []
        self.kalman_gains_icp = []
        self.H_icp = np.hstack([np.eye(3), -np.eye(3)])
        self.Q_icp = np.diag([10000, 10000, 0.0001])
        self.Q_landmarks = 10 * np.diag([0.01, 0.01, 0.05])
        self.Mahalanobis_threshold = 70 #for outlier detection
        self.R = 0.01 * np.diag([0.05, 0.05, 0.01, 0.05, 0.05, 0.01])
    
    def iteration(self, odometry_data, points_cloud1, points_cloud2, perform_icp_update = True, detected_landmarks = None):
        self.prediction_step(odometry_data)
        if perform_icp_update:
            self.icp_measurement(points_cloud1, points_cloud2)
            self.icp_update_step() 
        if detected_landmarks is not None:
            if len(detected_landmarks) != 0:
                self.update_landmarks(detected_landmarks)
        
    def prediction_step(self, odometry_data):
        """ Performs the dynamic model of the EKFSLAM.
        Args:
            odometry_data (np.array): The odometry data.
        """
        mean = self.means[-1].copy()

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


        new_mean = np.ones(len(mean))
        new_mean[0:6] = new_pose
        new_mean[6:] = mean[6:]

        self.means.append(new_mean)
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

    def update_landmarks(self, detected_landmarks):
        """
        Updates the SLAM state with newly detected landmarks using EKF update equations.
        
        Args:
            detected_landmarks: List of landmark measurements, each containing [range, bearing, signature]
        """
        if not detected_landmarks:
            return
            
        mu = self.means[-1]
        Sigma = self.covariances[-1]
        counter = 0
        # Process each detected landmark
        for z in detected_landmarks:
            # 1. Transform measurement to global coordinates
            cos_theta = np.cos(z[1] + mu[2])
            sin_theta = np.sin(z[1] + mu[2])
            landmark_global = np.array([
                mu[0] + z[0] * cos_theta,  # x
                mu[1] + z[0] * sin_theta,  # y
                z[2]                       # signature
            ])
            
            # 2. Data association
            best_match_idx = -1
            min_distance = 100000 #wrong
            
            # For each existing landmark, compute Mahalanobis distance
            for k in range(self.N):
                map_idx = 6 + 3*k  # Skip pose states (6) and index into map
                existing_landmark = mu[map_idx:map_idx+3]
                
                # Compute measurement prediction (h function)
                delta_x = existing_landmark[0] - mu[0]
                delta_y = existing_landmark[1] - mu[1]
                q = delta_x**2 + delta_y**2
                zhat = np.array([
                    np.sqrt(q),
                    np.atan2(delta_y, delta_x) - mu[2],
                    existing_landmark[2]
                ])
                
                # Compute Jacobians
                H = self._compute_measurement_jacobian(delta_x, delta_y, q, k)
                
                # Innovation covariance
                S = H @ Sigma @ H.T + self.Q_landmarks
                
                # Mahalanobis distance
                innovation = z - zhat
                distance = innovation.T @ np.linalg.inv(S) @ innovation
                print(f"Distance: {distance}")
                # Only consider landmarks within the Mahalanobis threshold
                if distance > self.Mahalanobis_threshold:
                    continue
                
                if distance < min_distance:
                    counter += 1 
                    min_distance = distance
                    best_match_idx = k
            print(f"Counter: {counter}")
            # 3. Handle the landmark
            if best_match_idx == -1 and min_distance > self.alpha:
                # New landmark - extend state and covariance
                self.N += 1
                mu = np.hstack([mu, landmark_global])
                
                # Compute Jacobians for new landmark
                G_z = np.array([
                    [cos_theta, -z[0]*sin_theta, 0],
                    [sin_theta, z[0]*cos_theta, 0],
                    [0, 0, 1]
                ])
                G_R = np.array([
                    [1, 0, -z[0]*sin_theta, 0, 0, 0],
                    [0, 1, z[0]*cos_theta, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0]
                ])
                
                # Extend covariance matrix
                Sigma_new = G_R @ Sigma[0:6, 0:6] @ G_R.T + G_z @ self.Q_landmarks @ G_z.T
                Sigma_cross = G_R @ Sigma[0:6, :]
                
                Sigma_temp = np.vstack((Sigma, Sigma_cross))
                Sigma = np.hstack((Sigma_temp, np.vstack((Sigma_cross.T, Sigma_new))))
                    
            else:
                # Update existing landmark
                map_idx = 6 + 3*best_match_idx

                print(f"Updating landmark {best_match_idx}: {mu[map_idx:map_idx+3]}")
                print(f"Landmark was associated with observation: {z}")

                H = self._compute_measurement_jacobian(
                    mu[map_idx] - mu[0],
                    mu[map_idx+1] - mu[1],
                    (mu[map_idx] - mu[0])**2 + (mu[map_idx+1] - mu[1])**2,
                    best_match_idx
                )
                
                # Kalman gain
                S = H @ Sigma @ H.T + self.Q_landmarks
                K = Sigma @ H.T @ np.linalg.inv(S)
                
                # Update state and covariance
                innovation = z - self._predict_measurement(mu, best_match_idx)
                mu = mu + K @ innovation
                Sigma = (np.eye(len(mu)) - K @ H) @ Sigma
        
        # Store updated state and covariance
        self.means.append(mu)
        self.covariances.append(Sigma)

    def _compute_measurement_jacobian(self, delta_x, delta_y, q, landmark_idx):
        """Helper function to compute the measurement Jacobian H."""
        # Create base Jacobian for pose and current landmark
        H_base = np.array([
            [np.sqrt(q)*delta_x, -np.sqrt(q)*delta_y, 0, -np.sqrt(q)*delta_x, np.sqrt(q)*delta_y, 0],
            [delta_y, delta_x, -1, -delta_y, -delta_x, 0],
            [0, 0, 0, 0, 0, 1]
        ]) / q
        
        # Extend H to full state size
        H = np.zeros((3, 6 + 3*self.N))
        H[:, :6] = H_base[:, :6]  # Copy pose Jacobian
        H[:, 6 + 3*landmark_idx:9 + 3*landmark_idx] = H_base[:, 3:6]  # Copy landmark Jacobian
        
        return H

    def _predict_measurement(self, mu, landmark_idx):
        """Helper function to compute the predicted measurement for a landmark."""
        map_idx = 6 + 3*landmark_idx
        delta_x = mu[map_idx] - mu[0]
        delta_y = mu[map_idx+1] - mu[1]
        q = delta_x**2 + delta_y**2
        
        return np.array([
            np.sqrt(q),
            np.atan2(delta_y, delta_x) - mu[2],
            mu[map_idx+2]
        ])
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
            # Compute the global coordinates of the detected landmark
            cos = np.cos(z[1] + mu[2])
            sin = np.sin(z[1] + mu[2])
            x_lm = mu[0] + z[0] * cos
            y_lm = mu[1] + z[0] * sin
            s = z[2]
            lm = np.array([x_lm, y_lm, s])
            # Provisional extended map
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
                delta_x = map[3*k] - mu[0]
                delta_y = map[3*k+1] - mu[1]
                print(f"delta_x: {delta_x}, delta_y: {delta_y}")
                delta = np.array([delta_x, delta_y])
                q = np.dot(delta, delta)
                zhat.append(np.array([np.sqrt(q), np.atan2(delta_y, delta_x) - mu[2], map[3*k+2]]))
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
                Psi = H @ Sigma_prov @ H.T + Q
                Psi_list.append(Psi)
                Mahalanobis_distances[k] = (z - zhat[k]).T @ np.linalg.inv(Psi) @ (z - zhat[k])
            Mahalanobis_distances[-1] = self.alpha
            print(f"Mahalanobis distances: {Mahalanobis_distances}")
            min_Mdist = 100000
            for k in range(N):
                if Mahalanobis_distances[k] > self.Mahalanobis_threshold: #Outlier detection
                    continue
                if Mahalanobis_distances[k] < min_Mdist:
                    min_Mdist = Mahalanobis_distances[k]
                    j[i] = k
            print(f"Detected landmark {i} associated with landmark {j[i]}")
            print(f"Global coordinates of detected landmark: {lm}")
            print(f"Global coordinates of the associated landmark: {map[3*j[i]:3*j[i]+3]}")
            # print(f"Before assigning: N = {N}, N_global = {self.N}, j = {j[i]}")
            # self.N = max(self.N, j[i]+1)
            # print(f"After assigning: N = {self.N}")
            # if self.N == j[i]+1:
            if j[i] + 1 > self.N:
                print(f"Adding landmark to the state: {lm}")
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
            # print(f"Inovation: {(detected_landmarks[i] - zhat_i[i])}, zhat: {zhat_i[i]}")
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
        print(f"Updated position: {self.means[-1][0:3]}")
        print(f"Update delta: {update_mean}")
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