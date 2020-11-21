from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt






class Kalman_Accel():
    def __init__(self,initPos=0, timestep=0.01,AccelerationValue=0, master=None):
        dt = 0.059
        F = [[1, dt, 0.5*(dt**2)], 
            [0,  1,       dt],
            [0,  0,        1]]

# observation_matrix   
        H = [0, 0, 1]

# transition_covariance 
        Q = [[0.1,    0,      0], 
            [  0,  0.1,      0],
            [  0,    0,  0.0001]]

# observation_covariance 
        R = 0.0020

# initial_state_mean
        X0 = [0,
            0,
            AccelerationValue]

# initial_state_covariance
        P0 = [[  0,    0,               0], 
            [  0,    0,               0],
            [  0,    0,   0.02]]

        self.previous_state_means=X0
        self.previous_state_covariances=P0


        self.kf = KalmanFilter(transition_matrices = F, 
                        observation_matrices = H, 
                        transition_covariance = Q, 
                        observation_covariance = R, 
                        initial_state_mean = X0, 
                        initial_state_covariance = P0)

# iterative estimation for each new measurement
    def update(self, acceleration):
        filtered_state_means, filtered_state_covariances = (
        self.kf.filter_update(
            self.previous_state_means,
            self.previous_state_covariances,
            acceleration
            ) )

        self.previous_state_means=filtered_state_means
        self.previous_state_covariances=filtered_state_covariances

        return filtered_state_means
