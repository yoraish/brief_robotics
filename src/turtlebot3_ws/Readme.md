### Getting Started
---

1. Install the following dependent ROS packages.
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

2. Install the turtlebot packages.
```
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-gazebo
```

3. Perform `catkin_make` on [turtlebot_ws](./) directory.


### Running the code

1. Run this on **every terminal** first whenver you're running any code from [turtlebot_ws](./) directory. You can also add this to `~/.bashrc` if you prefer.

```
export TURTLEBOT3_MODEL=burger
```

2. Run the gazebo simulation environment.
```
roslaunch turtlebot3_gazebo turtlebot3_chouse_camera.launch
```

3. You can control the robot using keyboard by running:
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

4. Use rviz to perform any visualizations:
```
oslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```



