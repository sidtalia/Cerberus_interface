# Cerberus_interface

This repository provides a ROS driver/interface for the TRAC cerberus INS.
## Author:
Sidharth Talia

## Installation:

Assuming rospy is already installed for your system:
1) Clone the repo:
``` bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/naughtyStark/Cerberus_interface.git 
$ pip install -r requirements.txt
```

2) Catkin make:
``` bash
$ cd ~/catkin_ws
$ catkin_make
```

The catkin make is necessary to make the system recognize this as a ros package.

## Running the ros driver:
``` bash
$ rosrun cerberus cerberus_link.py
```

Note that the driver assumes that the INS is connected as '/dev/ttyUSB0' by default. This will be fixed soon to be parameterizable.

## API:
The driver publishes the following topics:
Topic | Type | Description
------|------|------------
`/odom/cerberus` | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)| Odometry of car relative to the starting point when the ros-node was started
`/gps/cerberus` | [sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)| Raw GPS location (with GPS horizontal and vertical errors and number of satalites represented by the status number)

The driver can also pass external velocity data to the INS. The python side code for this is a WIP and needs to be parameterized. For now, you can publish the body frame velocity to `/car/t265/odom/sample` topic, although it is not recommended without going through the driver once because I made certain transformations to test it on the [MuSHR](https://mushr.io/) car. At the moment everything is at a Proof of concept level so I'll be updating stuff regularly. Besides, you can't really use this code if you don't have the INS so I'm not worried about people complaining...

Topic | Type | Description
------|------|------------
`/car/t265/odom/sample` | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)| external velocity input

