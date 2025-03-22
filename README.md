Robot programming: DMapLocalizer  	

This repository contains my project for the Robot Programming course at Sapienza University of Rome.
The project consists in a distance map based ICP localizer implemented with ROS where the initial guess is obtained iteratively from the odometry estimation.
There are three main ROS nodes, 
-DmapLocalizerNode which by receiving the scan, map and odometry information (odom-> base_link), refines the map -> odom estimation.
-ScanNode which receives the true pose of the robot in the map and computes and publishes the Scan.
-RobotControllerNode which integrates velocities from turtle_bot commands and publishes the drifted odom->base_link pose, where systematic and gaussian noise are injected.

To launch the simulation
-build the project with catkin_make 
-run ./setup.sh
From Rviz add map (displays the Occupancy grid) and display the tf transform hierarchy. set initial pose in the non-occupied grid and through turtle bot terminal, control velocity commands with the appropriate keys.
