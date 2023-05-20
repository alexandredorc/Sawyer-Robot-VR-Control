# Control a Cobot using a virtual reality controller.

The aim of this project is to connect a VR system to a robot that enables users to control 
the robot arm by moving their own arms and grip items using a gripper end effector. 
The end goal is to be able to perform a variety of tasks using the VR controller and different end effector. 

## Setup ROS environment

First to make this project work you need to setup the ROS Melodic environment on a Ubuntu 18.04 OS. To create the appropriate environment follow the tutorial on the official ROS [website](http://wiki.ros.org/melodic/Installation/Ubuntu) 

## Installing dependecies for the Sawyer robot

This project will use a Sawyer Robot of Rethink Robotics. We need to connect the Sawyer Robot by following the tutorial on the Rethink Robotics [website](https://www.rethinkrobotics.com/)  create the `catkin_ws` folder and the require dependencies indiquated in the website.

## Connection bewteen master computer and robots

To establish a connection between the robot and the master you need a router and connect your device with a static IP address. To setup your ROS environment with the robot you need to:
 
- add to the `~/.bashrc` file the following line 
> export ROS_MASTER_URI=http://Your_Master_IP:11311

> export ROS_IP=Sawyer_IP

- change the `intera.sh` file accordingly to the tutorial

Check the SDK version on the Sawyer robot and your master computer. We advise to update to the version 5.3.0 or up both of them to be sure not to have version collision.

## Initialize the robot

to initialize the Sawyer robot functions execute the following commands.
```bash
cd ~/catkin_ws
./intera.sh
rosrun intera_interface enable_robot.py -e
```

## Run the VR command 

..................
