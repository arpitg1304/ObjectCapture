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

4.) Start vrep, load scene1.ttt in V-Rep, do not run the simulation from V-Rep start button

5.) Run the python file getData.py (dependencies: PIL, CV2 libraries) in a terminal which will create a folder in the system and start saving the kinect sensor images as soon as the simulation starts(explained in next step)

6.) In a separate terminal, run object_cpture.py: The functonality of this file is controlled using the keyboard. Press 'm' to stop the simulation; press 'n' --> prompt will ask for z,x velocity (input for example 5,5 and hit enter), then enter time of flight(enter example 5 and hit enter). These inputs will start the simulation, if you check the folder created by the getDat.py now, you will find lots of images saved there if everything is working fine



