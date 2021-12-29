# DLL: Direct Lidar Localization

## Summary
This package presents DLL, a direct map-based localization technique using 3D LIDAR for its application to aerial robots. DLL implements a point cloud to map registration based on non-linear optimization of the distance of the points and the map, thus not requiring  features, neither point correspondences. Given an initial pose, the method is able to track the pose of the robot by refining the predicted pose from odometry. The method performs much better than Monte-Carlo localization methods and achieves comparable precision to other optimization-based approaches but running one order of magnitude faster. The method is also robust under odometric errors. 

DLL is fully integarted in Robot Operating System (ROS). It follows the general localization apparoch of ROS, DLL makes use of sensor data to compute the transform that better fits the robot odometry TF into the map. Although an odometry system is recommended for fast and accurate localization, DLL also performs well without odometry information if the robot moves smoothly. 

![DLL experimental results in different setups](dll_video.gif)

## Software dependencies
There are not hard dependencies except for Google Ceres Solver and ROS:
 - ceres: Follow installation instructions for Google Ceres (http://ceres-solver.org/installation.html)
 - ROS: The package has been tested in ROS Melodic under Ubuntu 18.04. Follow installation instruction from ROS (http://wiki.ros.org/melodic/Installation/Ubuntu)

## Hardware requirements
DLL has been tested in a 10th generation Intel i7 processor, with 16GB of RAM. No graphics card is needed. The optimization is currently configured to be single threaded. You can easily reduce the processing time by a 33% just increasing the number of threads used by Ceres Solver.

## Compilation
Download this source code into the src folder of your catkin worksapce:
```
$ cd catkin_ws/src
$ git clone https://github.com/robotics-upo/dll
```
Compile the project:
```
$ cd catkin_ws
$ source devel/setup.bash
$ catkin_make
```

## How to use DLL
You can find several examples into the launch directory. The module needs the following input information:
- A map of the environment. This map is provided as a .bt file
- You need to provide an initial position of the robot into the map. 
- base_link to odom TF. If the sensor is not in base_link frame, the corresponding TF from sensor to base_link must be provided.
- 3D point cloud from the sensor. This information can be provided by a 3D LIDAR or 3D camera.
- IMU information is used to get roll and pitch angles. If you don't have IMU, DLL will take the roll and pitch estimations from odometry as the truth values.

Once launched, DLL will publish a TF between map and odom that alligns the sensor point cloud to the map. 

When a new map is provided, DLL will compute the Distance Field grid. This file will be automatically generated on startup if it does not exist. Once generated, it is stored in the same path of the .bt map, so that it is not needed to be computed in future executions.

As example, you can download 5 datasets from the Service Robotics Laboratory repository (https://robotics.upo.es/datasets/dll/). The example launch files are prepared and configured to work with these bags. You can see the different parameters of the method. Notice that, except for mbzirc.bag, these bags do not include odometry estimation. For this reason, as an easy work around, the lauch files publish a fake odometry that is the identity matrix. DLL is faster and more accurate when a good odometry is available.

## Cite
DLL has been accepted for publication in IROS 2021.

F. Caballero and L. Merino. "DLL: Direct LIDAR Localization. A map-based localization approach for aerial robots". Sumbitted to the International Conference on Intelligent Robots and Systems, IROS 2021.

You can download preliminar version of the the paper from [arXiv](https://arxiv.org/abs/2103.06112) 



