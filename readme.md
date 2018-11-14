ROS Beginner Tutorials - Introduction to Publisher and Subscriber  
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

This program is a basic introduction to a publisher and a subscriber in ROS. It creates two nodes, a talker (publisher) node and a listener (subscriber) nodes.

In Week10_HW branch, updated service calls and included service , loggers in talker node. Service updates the string as given in argument while running the node. A launch file was created to run both nodes at same time. An argument to change the publisher frequency was also added in the launch file and in the code. The logging levels INFO, DEBUG, WARN, ERROR and FATAL have been used in the package.

In Week11_HW branch Transform broadcaster was added to talker node. It will create a static transform between the world parent frame and a frame called talk. Updated the results directory with echo_tf and rqt_tree output. Implemented unit test in test folder to test if the service is updating the string properly. 
Modified beginner.launch file to include data recording. It saves all the transmission information when any of te nodes are working. Included a sample bag file in results directory. 

## Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install ros, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## Build Instructions

To run this code in a ros workspace:
```
cd ~/ros_ws/
source devel/setup.bash
cd src/
git clone https://github.com/bhargav-umd/beginner_tutorials
cd ..
catkin_make
```
If you do not have a ros workspace:
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
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

Then run talker node in second terminal:
```
source devel/setup.bash
rosrun beginner_tutorials talker
```
Output will look like:
```
[ INFO] [1540934618.726572148]: GO TERPS GO 0

```
Finally run listener node in third terminal:
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
Output will look like
```
[ INFO] [1540934620.527403735]: I heard: [GO TERPS GO 0]
```

## To call the service

* Once both nodes are running in separate terminals as described above, check in another terminal that the service is being detected. `rosservice list` should output a few services and `/update_string` should be listed.

* Now run the following commands to call the service and change the string to be published (`newString` is the message to be published):
```
cd ros_ws
source devel/setup.bash
rosservice call /update_string newString
```
The talker and listener terminals should output now the new message containing newString

## To run using launch file

* To run the package with a launch file, first close every terminal related to this package (including `roscore`).

* Now run the following commands in the terminal:
```
cd ros_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner.launch frequency:=3
```
where `frequency` is the argument that changes the publisher frequency. The value 3 is just an example and it can be changed for whatever other integer value.

* Once the `roslaunch` command is executed, another terminal will be opened with the listener output. The first terminal outputs the talker node's output.

## Verify  TF frames

To verify that the TF frames were correctly created, two different actions can be done:

* First run the talker node in a terminal and then run tf_echo in another terminal:

`rosrun beginner_tutorials talker frequencyvalue` and then `rosrun tf tf_echo world talk`

The output is as included in results directory and as below:

* The second way is to use rqt_tf_tree. With the talker node running in a terminal, run the next command in a another terminal:

```
rosrun rqt_tf_tree rqt_tf_tree 
```
or simply

```
rqt &
```
The output is as shown in results directry:

Run following command to generate pdf using view_frames tool
```
rosrun tf view_frames
```
It will generate frames.pdf 

## Running rostest

To run rostest, we do not need to run any node. In a terminal go to your ros_ws directory and run the next commands:

```
cd ~/ros_ws
source devel/setup.bash
catkin_make run_tests
```

This will compile the tests and the terminal should output the results of the tests as shown:

## Recording with bag files

To record using rosbag, we have to launch the updated `beginner.launch`. A new argument called _rosbagFlag_ is included in the launch file. When called as _true_ the data will be recorded. If called as false or not given in argument of launch while running launch file  the data will not be recorded.

* To record data:

```
cd ~/ros_ws
source devel/setup.bash
roslaunch beginner_tutorials tutorial.launch rosbagFlag:=true
```
This will record the data in the `~/.ros` folder. To check information saved in bag file:

```
cd ~/.ros
rosbag info bagfile.bag
```

* If the data recording is not needed, just launch:

`roslaunch beginner_tutorials beginner.launch` or `roslaunch beginner_tutorials tutorial.launch rosbagFlag:=false`

## Bag file demonstration with ONLY listener Node

Once the bag file is generated, it can be used as data to be sent to the listener. To try this, in a first terminal run the listener node (with roscore running already):

```
cd ~/ros_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

In a second terminal, 

```
cd ~/.ros
rosbag play bagfile.bag`
```
This will run the rosbag and the listener will start receiving the data sent by `bagfile.bag`.

