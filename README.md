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

### Note: Give the unit about 60 seconds after the green light starts blinking before running the ros-driver.

## Running the GUI:
Connect the system and run the following command:
```bash
$ python /catkin_ws/src/Cerberus_interface/src/GUI.py
```
Note that right now you can either run the ros-driver or the GUI, but not both at the same time. If there is a need for this, such a feature will be implemented in the future.


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

## Running the bag data for analysis:
Start a new terminal:
1) start roscore:
``` bash
roscore
```
2) In a new tab, run the log_converter.py:
```bash
$ cd ~/catkin_ws/src/Cerberus_interface/src
$ python log_converter.py
```
3) In another new terminal, run:
```bash
$ cd ~/catkin_ws/src/Cerberus_interface/bags/
$ rosbag play -r 200.0 cerberus_pixh409_cmparison.bag
```
Once the bag has finished, go to the tab in which you ran the log converter and press Ctrl+C. If you get a index error, just repeat the process from step 2. On pressing Ctrl+C, you should be shown 4 graphs (one after another, you'll have to close one for the next to open) which show the performance comparison between the pixhawk running ArduPlane 4.0.9 and the cerberus INS.

If you wish to compare against an older version, there is a bag for that too. Replace the 409 in the bag's name with 368 and you should be good to go. There may be spelling mistakes. 