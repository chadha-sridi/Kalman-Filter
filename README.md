# Kalman Filter
## Overview
A Kalman filter is an optimal recursive data processing algorithm used for estimating the state of a linear dynamic system from noisy observations. It uses a series of measurements over time, along with system dynamics (model), to predict and update the state of the system. The Kalman filter is widely used in various applications such as navigation, control systems, and signal processing.
This code integrates a Gaussian process with a Kalman filter for state estimation. It simulates a system with process and measurement noise and demonstrates how the filter can estimate the system's true states and predict observations.
## Files
* *kalman_filter.m*: Main script to run the simulation and Kalman filter.
* *model_sim.m*: Function to simulate state evolution and observations (according to the State and Observation equations).
* *KalmanFilt.m*: Function to perform Kalman filtering on observations.
## Key Parameters
#### State-Space Matrices:
 * F: State transition matrix.
 * G: Input matrix.
 * H: Observation matrix.
#### Covariance Matrices:
 * Q: Process noise covariance.
 * R: Measurement noise covariance.
 * P1: Initial state covariance for Kalman filter.
#### Initial State:
  * x0: Initial true state.
  * x1: Initial estimated state.
## Requirements
* MATLAB
