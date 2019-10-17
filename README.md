# AMADEE 2020 catkin workspace

## How to run the example

```
# init git submodules
./init
# build from source
catkin_make all run_tests -j 1
# add your packages to the ROS environment
source devel/setup.bash
# start the mission
roslaunch bringup mission1.launch
```

For further detail about the scripts and maintainance of an catkin workspace follow:
https://gitlab.aau.at/aau-nav/development/examples/example_catkin_ws

## Requirements:

1. QGroundControl:
1. Mavlink compatible Flight-Control-Unit (FCU): e.g. Pixhawk 4 flashed with the Ardupilot-Copter or PX4 flight stack.
1. Serial link between FCU (Telemetry 1: 921600 baud, 8N1) and PC:
1. connect the Logitec F710 joy pad via USB dongle with our PC.

## Running the system

Use multiple terminals:
Terminal1:
```
$ ./QGroundControl.AppImage
```

Terminal2:
```
roslaunch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600 gcs_url:=udp://@localhost
```

Terminal3:
```
roslaunch ros_gamepad_teleop twist2pose.launch 
```
