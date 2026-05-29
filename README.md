# Reconstructing human-like movement from experimental end effector timeseries using the ROS framework
*Master's Thesis Project | ROS2 Jazzy | Inverse Kinematics & Time-Series Analysis*

## Project Summary 
In this project I created a biomechanically inspired model of the human arm within ROS2 Jazzy and a Jacobian Inverse Kinematics solver. 
Then recorded timeseries of human reaching movements were taken as input and possible human-like paths are constructed from these.
The resulting paths were evaluated using the Normalized Jerk Score as well as with the elbow angle.
Resulting data was visually prepared. 

## Archicture 
The Repo is devided up into 3 parts:
1. ros2_ws - a ROS2 Jazzy workspace that includes the Robot Model as well as the Solver
2. calibration - standalone code to calculate some calibration data from the input timeseries
3. eval - data visualization & interpretation  
