##  bb_sensors
This ROS package forms part of the BeetleBot platform, constructed
to test models of insect orientation and navigation.

The software is split across two repositories; this one contains all
software which runs on-board, for the Host PC software, see
[beetlebot_software](https://github.com/refmitchell/beetlebot_software).

This software is supposed to be distributed as part of a complete system
image for the BeetleBot so you should not have to set up your catkin
workspace to build this software. Nevertheless, there are some
external dependencies to bear in mind.

###  Compile-time dependencies
- [vision_opencv](https://github.com/ros-perception/vision_opencv) (if you are building
the workspace from scratch, make sure you download the version for ROS **Kinetic**).

###  Run-time dependencies
This code is designed to be run on a TurtleBot3 (Burger) which requires the
following ROS packages. These can be built from source by cloning the repos
into your catkin workspace.

- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

Each of these repos has a number of branches available. Make sure you
use a version compatible with ROS **Kinetic**.

### Documentation
Package documentation is available under doc/html/index.html.

This documentation cannot be re-built on the BeetleBot unless you install
rosdoc. Otherwise, you can clone the repo to a machine with rosdoc (i.e. your
Host PC) and build the docs there.