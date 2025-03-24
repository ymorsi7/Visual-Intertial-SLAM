# Visual-Inertial SLAM with Extended Kalman Filter

**Yusuf Morsi**  
*Department of Electrical and Computer Engineering*  
*University of California, San Diego*  
La Jolla, CA, USA  
ymorsi@ucsd.edu

## Running

Due to academic integrity policies, the majority of my code for this project (i.e. `code/feature_detection.py`, `code/main_utils.py`, `code/pr3_utils.py`) is in my `.gitignore`, thus not publicly available in this repository.

If you would like to run the code, I've provided compiled Python bytecode (`.pyc`) files in place of the original `.py` files. These have the same functionality of my original code but in a compiled format (again, for academic integrity reasons). 

However, if you would like to see my results after the calling of the functions in my code, navigate to `data/main.ipynb` to see the cells calling the functions, with the outputs.

## Introduction

This project, which focuses on implementing Simultaneous Localization and Mapping (SLAM); is meant to solve the problem of constructing/updating a map of an unknown environment, all the while keeping track of the robot/device's location with respect to it. More specifically, our project implements Visual-Inertial SLAM, where we combine camera data from Clearpath Jackal robots navigating on MIT's campus with inertial measurement units (IMU) data. This is important as it is implemented in many technologies, including rovers on Mars, iRobot vacuum cleaners, AR headsets, and more.

We find this to be important in the context of our project because the cameras fill gaps that IMUs have, and vice versa (i.e. cameras aren't always aware of the environment when things like light, rain, etc are in the way). Knowing how to combine these two is imperative, as it allows us to get results that, on their own, the individual sensors would otherwise not be able to attain.

To reiterate, this project focuses on my implementation of a Visual-Inertial SLAM system with an Extended Kalman Filter (EKF) framework. By combining IMU kinematics for motion prediction with stereo camera data (for landmark mapping and poses), we are able to conduct IMU-only localization, then landmark mapping with EKF update, and finally, combine both elements into a complete SLAM system.

## Problem Formulation

The following is the Visual-Inertial SLAM problem in mathematical terms, quoting from [6]:

### Inputs

- IMU measurements: linear velocity ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bv%7D_t%20%5Cin%20%5Cmathbb%7BR%7D%5E3) and angular velocity ![equation](http://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Comega%7D_t%20%5Cin%20%5Cmathbb%7BR%7D%5E3) of the body with coordinates expressed in the body frame of the IMU.

- Time stamps: ![equation](http://latex.codecogs.com/gif.latex?%5Ctau_t) in UNIX time (seconds since January 1, 1970).

- Camera measurements: grayscale images with pixel length and width (600 × 480) are provided separately in datasetXX_imgs.npy.

- Visual feature measurements: pixel coordinates ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bz%7D_t%20%5Cin%20%5Cmathbb%7BR%7D%5E%7B4%5Ctimes%20M%7D) of detected visual features from M point landmarks with precomputed correspondences between the left and right camera frames. Landmarks i that were not observable at time t have measurement 

![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bz%7D_%7Bt%2Ci%7D%20%3D%20%5B-1%20-1%20-1%20-1%5D%5ET), indicating a missing observation.

- Extrinsic calibration: transformation ![equation](http://latex.codecogs.com/gif.latex?%5E%7BI%7D%5Ctextbf%7BT%7D_C%20%5Cin%20SE%283%29) from the left and right camera to the IMU frame. The IMU frame is oriented as x = forward, y = left, z = up.

- Intrinsic calibration: values for the camera calibration matrix for the left and right camera: 

![equation](http://latex.codecogs.com/gif.latex?K%20%3D%20%5Cbegin%7Bbmatrix%7D%20f_u%20%26%200%20%26%20c_u%20%5C%5C%200%20%26%20f_v%20%26%20c_v%20%5C%5C%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

### Expected Output

- World-frame IMU pose ![equation](http://latex.codecogs.com/gif.latex?%5E%7BW%7D%5Ctextbf%7BT%7D_%7BI%2Ct%7D%20%5Cin%20SE%283%29).
- World-frame coordinates ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bm%7D_j%20%5Cin%20%5Cmathbb%7BR%7D%5E3) of the point landmarks that generated the visual features.

### Goal

As described in [1] and [6], our main focus is to determine not only the the robot trajectory, but also landmark positions through state estimation:

Prior: ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bx%7D_t%20%7C%20%5Ctextbf%7Bz%7D_%7B0%3At%7D%2C%20%5Ctextbf%7Bu%7D_%7B0%3At-1%7D%20%5Csim%20%5Cmathcal%7BN%7D%28%5Cboldsymbol%7B%5Cmu%7D_%7Bt%7Ct%7D%2C%20%5Cboldsymbol%7B%5CSigma%7D_%7Bt%7Ct%7D%29)

where ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bx%7D_t%20%3D%20%5Ctext%7Bstate%20vector%20with%20trajectory%7D%20%5Ctextbf%7BT%7D_t%20%5Cin%20SE%283%29%20%5Ctext%7Band%20landmark%20positions%7D%20%5Ctextbf%7Bm%7D%20%5Cin%20%5Cmathbb%7BR%7D%5E%7B3M%7D), ![equation](http://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Cmu%7D_%7Bt%7Ct%7D) is the est. mean, and ![equation](http://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5CSigma%7D_%7Bt%7Ct%7D) is the state covariance matrix.

## Technical Approach

The primary approach utilizes Extended Kalman Filter (EKF), where we have IMU-based prediction steps and camera-based update steps. We have three main stages in this project, which are, respectively: IMU localization, landmark mapping, and finally, full Visual-Inertial SLAM.

### IMU Localization via EKF Prediction

For IMU localization, we utilize the kinematic equations of motion for pose ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7BT%7D_t%20%5Cin%20SE%283%29) while using IMU measurements as inputs, as it provides us with both linear and angular velocity (![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bv%7D_t), ![equation](http://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Comega%7D_t)).

Utilizing ECE276A lecture notes [1], we find the, the discrete-time kinematics equation used for the pose update as:

![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7BT%7D_%7Bt&plus;1%7D%20%3D%20%5Ctextbf%7BT%7D_t%20%5Cexp%28%5Ctau_t%20%5Chat%7B%5Cboldsymbol%7B%5Czeta%7D%7D_t%29)

where ![equation](http://latex.codecogs.com/gif.latex?%5Ctau_t) is the length of the time step, and ![equation](http://latex.codecogs.com/gif.latex?%5Chat%7B%5Cboldsymbol%7B%5Czeta%7D%7D_t) is the twist matrix.

Said matrix is created at every time step and keeps a covariance matrix for uncertainty, (which is updated per the EKF prediction equations from [1]). This gives us the ability to track the IMU pose over time while also taking measurement noise into consideration.

### Stereo Vision and Feature Processing

An additional task we deeply explored, is feature detection and tracking in stereo camera images over time. The focus of the stereo matching was to first detect features in left camera images, and match said features with corresponding points in the right camera images. The plan was then to track these features across sequential frames, implementing temporal tracking, then storing coordinates into a matrix, and using OpenCV (i.e. goodFeaturesToTrack and calcOpticalFlowPyrLK) [2] [3] for assistance.

For the Stereo-Matching task, our code process the left and right camera images in order to have feature correspondences. We read both of the video streams, convert the frames to grayscale, and detect corner features in the left image with the Shi-Tomasi detector (using goodFeaturesToTrack from OpenCV [2]). We subsequently track these features to the correct image with Lucas-Kanade optical flow (calcOpticalFlowPyrLK from OpenCV [3]) with a 21 by 21 window size and 3 pyramid levels. We ensure that points with bad tracking/outside the image boundaries are filtered before we continue processing the matched coordinates. We used the provided Github code [4] as a model to follow, especially for the stereo-matching.

For Temporal Tracking, we track features across consecutive frames from the left camera. Our pseudocode is written to read the first frame and detect initial features before initializing a feature matrix with -1 values. For frames after that, the system tracks points from the previous frame with OpenCV's calcOpticalFlowPyrLK [3]. Once again, we filter bad points before updating the feature matrix.

### Landmark Mapping via EKF Update

For landmark mapping, we use our determined IMU poses, and building on that, we estimate the landmark positions. Referring back to the lecture slides [1], we want to estimate the landmark coordinates: ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bm%7D%20%3D%20%5B%5Ctextbf%7Bm%7D_1%5ET%2C%20%5Ctextbf%7Bm%7D_2%5ET%2C%20...%2C%20%5Ctextbf%7Bm%7D_M%5ET%5D%5ET%20%5Cin%20%5Cmathbb%7BR%7D%5E%7B3M%7D).

The observation model, taken from the lecture notes once again [1], relates landmark positions to stereo camera measurements:

![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bz%7D_%7Bt%2Ci%7D%20%3D%20h%28%5Ctextbf%7BT%7D_t%2C%20%5Ctextbf%7Bm%7D_j%29%20&plus;%20%5Ctextbf%7Bv%7D_%7Bt%2Ci%7D)

where h is the stereo camera projection function and ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bv%7D_%7Bt%2Ci%7D) is measurement noise.

For the EKF update, we compute the Jacobian of the observation model w.r.t. landmark positions, once again as shown in the lecture slides [1] before performing the update:

![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7BK%7D_%7Bt&plus;1%7D%20%3D%20%5Cboldsymbol%7B%5CSigma%7D_t%20%5Ctextbf%7BH%7D_%7Bt&plus;1%7D%5ET%20%28%5Ctextbf%7BH%7D_%7Bt&plus;1%7D%20%5Cboldsymbol%7B%5CSigma%7D_t%20%5Ctextbf%7BH%7D_%7Bt&plus;1%7D%5ET%20&plus;%20%5Ctextbf%7BR%7D%29%5E%7B-1%7D)  
![equation](http://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5Cmu%7D_%7Bt&plus;1%7D%20%3D%20%5Cboldsymbol%7B%5Cmu%7D_t%20&plus;%20%5Ctextbf%7BK%7D_%7Bt&plus;1%7D%20%28%5Ctextbf%7Bz%7D_%7Bt&plus;1%7D%20-%20%5Ctilde%7B%5Ctextbf%7Bz%7D%7D_%7Bt&plus;1%7D%29)  
![equation](http://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5CSigma%7D_%7Bt&plus;1%7D%20%3D%20%28%5Ctextbf%7BI%7D%20-%20%5Ctextbf%7BK%7D_%7Bt&plus;1%7D%20%5Ctextbf%7BH%7D_%7Bt&plus;1%7D%29%20%5Cboldsymbol%7B%5CSigma%7D_t)  

where ![equation](http://latex.codecogs.com/gif.latex?%5Ctilde%7B%5Ctextbf%7Bz%7D%7D_%7Bt&plus;1%7D) is the predicted observation, ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7BK%7D_%7Bt&plus;1%7D) is the Kalman gain, and ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7BR%7D) is the measurement noise covariance.

### Visual-Inertial SLAM

For the final part of the project, the complete Visual-Inertial SLAM, we combine the IMU prediction step with the camera update step, but now also update the IMU pose based on observations. As we are relayed once again in the lecture slides [1], the state consists of the IMU pose ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7BT%7D_t%20%5Cin%20SE%283%29) and landmark positions ![equation](http://latex.codecogs.com/gif.latex?%5Ctextbf%7Bm%7D%20%5Cin%20%5Cmathbb%7BR%7D%5E%7B3M%7D).

For the update step, we find the Jacobian of the observation model w.r.t. the IMU pose, once again modeling after the methodology presented in the lecture [1]. This allows us to fix the pose based on visual observations that we have of known landmarks. Despite being similar to the landmark update, the EKF update for
