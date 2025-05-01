Author: Jeremy Chen
Email: pchen189@asu.edu
School: Arizona state university
Term: Spring 2025
Date created: May 1 2025
Link to website: https://ras598-2025-s-team06.github.io/

Short description:
    Goals:
    Our objective was to develop a simple multi-robot system using ROS 2 for inter-device communication. Rather than building a complex framework, our primary focus was to ensure stable connectivity and deliver a user-friendly graphical interface for controlling the system.

    Process:
    We initially implemented CycloneDDS as the discovery server, but due to persistent connection issues, we transitioned to Fast DDS for improved stability. Once the nodes were successfully connected, we developed a custom GUI on the virtual machine to manage all image processing tasks. The GUI provides full control over the robot with features including directional movement, commands for moving to pickup locations, instructing the UR5 arm to place items on the robot, navigating to different offloading zones, docking and undocking, and speed control. Additionally, it features an integrated IMU data visualization to help users monitor and understand the robot’s movements in real time.

    Results:
    All core functionalities were successfully implemented and tested. The robot can undock, navigate to the pickup area, receive an item from the UR5 arm, move to the designated drop-off location, and return to its docking station. However, due to Wi-Fi instability and UR5 driver limitations, we were unable to run all components concurrently. As a result, the robot and UR5 could only be operated individually; otherwise, the UR5 would frequently disconnect.

File breakdown:
    ./full_gui/gui_v1.py --> this is the file that includes all function of the robot
    ./full_gui/mwindow.ui --> a GUI setup file created by PyQt5 designer, that is used in the main .py code

Installation:
    Ensure ros2 humble is installed
    Download the whole package to your ros workspace
    Use Colcon build to build the packages
    Use rosdep to ensure all dependencies are installed
    source the file: install/setup.bash
    execute the following code: ros2 run full_gui fullgui

    The program should automatically show the gui, however, if the GUI is not showing, ensure all the topics are publishing as the program only executes once it sees all the topics.

Uses:
    Please refer to the website for function breakdowns and a short DEMO of each features.



External Sources used:
https://github.com/jaspereb/UR5_With_ROS_Moveit_Tutorial
https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_robot.html
also codes from Professor Daniel Aukes