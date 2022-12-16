# brief_robotics
This repository is the backbone for the class "A Brief Introduction to Robotics".

This is an experimental set of educational modules communicating the basics of robotic motion, trajectory optimization, and simulation. Use at your own risk :) 

## Getting Started

1. For setting up the simulation environment please follow the guidelines given [here](./src/turtlebot3_ws/Readme.md) in order to setup ROS and run simulations.
Note: The simulation environment is based on the [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) platform 
2. The following modules are independent of the simulation, and can be run on any machine with python3 and Jupyter installed. 
To install Jupyter, please follow the instructions [here](https://jupyter.org/install).
To install this tutorial in your local machine, simply do the following:

```git clone https://github.com/yoraish/brief_robotics.git ```

We recommend using conda environments for installing python packages. To create a conda environment, do the following:

```conda env create -f brief_robotics.yml```

To install the dependencies with pip and without using conda, do the following:

```pip install -r requirements.txt```

## Modules
1. Module 1: [Odometry using Optical Flow](./src/optical_flow/optical_flow.ipynb) - A guide explanation and implementation of optical flow for odometry estimation.
2. Module 2: [Trajectory Optimization](./src/trajectory_optimization/trajectory_optimization.ipynb) - A guide explanation and implementation of trajectory optimization for a simple car-like robot.
3. Module 3: [Landmark Detection](./src/landmark_detection/landmark_detection.ipynb) - A simple implementation of landmark-detection with Aruco markers for creating a pose graph.
