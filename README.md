Motion Library for UR5

Overview

  This repository contains a Python-based motion library and a Flask-Dash  interface designed for simulating and controlling the UR5 robotic arm. The project provides the following features:

  1. Joint and linear motion planning primitives.

  2. Gazebo integration to visualize and execute motion.

  3. Flask-Dash Copilot interface for code suggestions using open-source LLMs.

Features

  1. Motion Library

  2. Joint Motion: Moves the robot between two points in joint space.

  3. Linear Motion: Moves the robot between two poses in Cartesian space.

  4. Includes an inverse kinematics (IK) solver using PyKDL.

  5. Copilot Integration

  6. Interactive interface for code suggestions.

  7. Suggests motion primitives and APIs via an LLM-powered backend.

  8. Gazebo Integration

  9. Publishes joint motion outputs to the Gazebo simulation environment.

  10. Visualizes robot motion in Gazebo and RViz.

Installation Guide

  Step 1: Install ROS (Robot Operating System)

    Follow the official ROS installation guide for your platform: ROS Installation Guide.

    Verify the installation by running:

      roscore

  Step 2: Create a ROS Workspace

    Create a workspace directory:

    mkdir -p ~/ur5_ws/src
    cd ~/ur5_ws/src
    catkin_init_workspace

  Build the workspace:

    cd ~/ur5_ws
    catkin_make

  Source the workspace setup file:

    source ~/ur5_ws/devel/setup.bash

  Step 3: Install Required Libraries

  Install dependencies:

    sudo apt-get update
    sudo apt-get install ros-<your_ros_distro>-kdl-parser-py python3-pykdl python3-flask python3-dash python3-rosdep

  Install Python packages:

    pip install numpy flask dash plotly openai

Setting Up the Repository

  Clone the repository:

    git clone albingeorge7/Ros_ur5_simulation
    cd clone albingeorge7/Ros_ur5_simulation

  Copy the URDF file of your robot (e.g., UR5) to the appropriate directory.

  Configure the Flask-Dash application:



Running the Application

  Step 1: Launch 

  roslaunch ur5_joint_sine_publisher sine_wave_joint_publisher.launch
    

  Step 2: Run the Motion Library

  Start the motion publisher:

    python motion_library.py

  Step 3: Start the Flask-Dash Copilot

  Run the Flask-Dash application:

    python app.py

Access the interface in your web browser at http://127.0.0.1:5000.

Usage Guide

  
    robot_urdf = "/home/albin/ur5_ws/src/universal_robot/ur_description/urdf/urdf/xacro"  # Provide the URDF path
    api = RobotMotionAPI(robot_urdf)


    point1 = np.array([0, 0, 0, 0, 0, 0])  # Starting joint configuration
    point2 = np.array([1, 1, 1, 1, 1, 1])  # Ending joint configuration
    joint_velocity = 0.1
    joint_acceleration = 0.1

    api.move_robot_joint(point1, point2, joint_velocity, joint_acceleration) #joint move


    pose1 = np.array([[1, 0, 0, 0.5],
                      [0, 1, 0, 0.5],
                      [0, 0, 1, 0.5],
                      [0, 0, 0, 1]])

    pose2 = np.array([[1, 0, 0, 1.0],
                      [0, 1, 0, 1.0],
                      [0, 0, 1, 1.0],
                      [0, 0, 0, 1]])

    linear_velocity = 0.1
    linear_acceleration = 0.1

    api.move_robot_cartesian(pose1, pose2, linear_velocity, linear_acceleration) #cartesian move
    
  Copilot Interface

Enter your desired motion primitive or API call in the provided interface.

Receive code suggestions and explanations powered by the integrated LLM.

Development

  Key Components

  motion_library.py: Implements the motion planning primitives.

  copilot.py: Flask-Dash application for the Copilot interface.

  sine_wave_joint_publisher.py: Publishes sine wave motion to Gazebo.

  Launch Files:

  sine_wave_joint_publisher.launch: Launches the sine wave publisher.

  ur5_sinewave.rviz: Configures RViz for motion visualization.

