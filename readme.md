ROS Beginner Tutorials - Introduction to Publisher and Subscriber  
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

This program is a basic introduction to a publisher and a subscriber in ROS. It creates two nodes, a talker (publisher) node and a listener (subscriber) nodes.

## Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## Build Instructions

To run this code in a catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone https://github.com/bhargav-umd/beginner_tutorials
cd ..
catkin_make
```
If you do not have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/bhargav-umd/beginner_tutorials
cd ..
catkin_make
```

## Run Instructions 

After following the build instructions:

Run roscore in a new terminal:
```
source devel/setup.bash
roscore
```

Then run talker node from a new terminal:
```
source devel/setup.bash
rosrun beginner_tutorials talker
```
Finally run listener node from a new terminal:
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
