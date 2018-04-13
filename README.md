# Capture a flying object using 6 DOF robot arm through visual Servoing

*Mar 2018 – Present*

Project description:

This project is divided in four main sub-tasks:

1.) Using Microsoft Kinect to receive RGBD(red, blue, green, depth) data and process it to get x,y, z coordinates of the flying object(each frame) in World reference through homogeneous transformations

2.) Calibration of the sensor using Extended Kalman filter based trajectory prediction and correction algorithms

3.) Predict the landing point of the flying object in robot arm's workspace

4.) Deploy the PID and other control schemes to perform the inverse kinematics on robot arm so that the robot arm tip reaches the landing point of the flying object


# Setup Instructions and Dependencies:

1.) The simulation environment used is V-Rep in Linux environment: http://www.coppeliarobotics.com/downloads.html
Communication channel used is ROS

2.) Clone this repository using git clone https://github.com/arpitg1304/ObjectCapture.git

3.) Interfacing ROS kinetic with v-rep: http://analuciacruz.me/articles/RosInterface_kinetic/ 


https://github.com/CoppeliaRobotics/v_repExtRosInterface



