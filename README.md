# ROSbot 2 driver

This repository contains driver for ROSbot 2. Driver allows for controlling motors, reading odometry and battery state.

## Installation

* Copy code to Husarion Cloud or build it with VSCode plugin
* Upload to CORE2
* Start serial bridge `/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2`

## Subscribed topics

* `/cmd_vel` [geometry_msgs::Twist] - velocity commands for robot
* `/reset_odom` [std_msgs::Bool] - trigger to reset odometry and encoders

## Published topics

* `/pose` [geometry_msgs::PoseStamped] - robot position and orientation based on encoder readings
* `/battery` [sensor_msgs::BatteryState] - robot battery voltage
