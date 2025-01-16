Motion Library for UR5

Overview

This repository contains a Python-based motion library and a Flask-Dash Copilot interface designed for simulating and controlling the UR5 robotic arm. The project provides the following features:

Joint and linear motion planning primitives.

Gazebo integration to visualize and execute motion.

Flask-Dash Copilot interface for code suggestions using open-source LLMs.

Features

Motion Library

Joint Motion: Moves the robot between two points in joint space.

Linear Motion: Moves the robot between two poses in Cartesian space.

Includes an inverse kinematics (IK) solver using PyKDL.

Copilot Integration

Interactive interface for code suggestions.

Suggests motion primitives and APIs via an LLM-powered backend.

Gazebo Integration

Publishes joint motion outputs to the Gazebo simulation environment.

Visualizes robot motion in Gazebo and RViz.

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

git clone <repository-url>
cd <repository-name>

Copy the URDF file of your robot (e.g., UR5) to the appropriate directory.

Configure the Flask-Dash application:

Modify the app.py file to include your OpenAI or LLM API keys if necessary.

Running the Application

Step 1: Launch ROS Core and Gazebo

Start ROS core:

roscore

Launch the Gazebo simulation:

roslaunch ur_gazebo ur5.launch

Step 2: Run the Motion Library

Start the motion publisher:

python motion_library.py

Step 3: Start the Flask-Dash Copilot

Run the Flask-Dash application:

python app.py

Access the interface in your web browser at http://127.0.0.1:5000.

Usage Guide

Motion Library

Import the library and initialize the robot controller:

from motion_library import UR5MotionController

controller = UR5MotionController(robot_urdf='path_to_urdf')

Execute a joint motion:

controller.joint_motion([0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1], 0.5, 0.2)

Execute a linear motion:

controller.linear_motion(pose1, pose2, 0.5, 0.2)

Copilot Interface

Enter your desired motion primitive or API call in the provided interface.

Receive code suggestions and explanations powered by the integrated LLM.

Development

Key Components

motion_library.py: Implements the motion planning primitives.

app.py: Flask-Dash application for the Copilot interface.

ur5_sine_wave_publisher.py: Publishes sine wave motion to Gazebo.

Launch Files:

sine_wave.launch: Launches the sine wave publisher.

ur5_visualization.launch: Configures RViz for motion visualization.

Contributing

Contributions are welcome! Feel free to open issues or submit pull requests for new features, bug fixes, or documentation updates.

License

This project is licensed under the MIT License.

Acknowledgments

ROS and Gazebo for simulation.

PyKDL for the inverse kinematics solver.

Flask and Dash for the Copilot interface.

OpenAI for LLM-based code suggestions.
