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

## Requirements

1. Install latest [QGroundControl](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html) (QGC):
1. Mavlink compatible Flight-Control-Unit (FCU): e.g. [Pixhawk 4](https://docs.px4.io/v1.9.0/en/flight_controller/pixhawk4.html) flashed with the [Ardupilot-Copter](http://ardupilot.org/) or [PX4 flight stack](https://docs.px4.io/master/en/index.html). [Changing the firmware](https://docs.qgroundcontrol.com/en/SetupView/Firmware.html) can be done with QGC.
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

## Findings

1. FCU: ArduCopter V3.6.9
1.

## Links

### Indoor Fligths

1. [Chobitsfan's Blog](https://discuss.ardupilot.org/t/indoor-flight-with-external-navigation-data/29980/6)
1. [Vicon Example](http://ardupilot.org/copter/docs/common-vicon-for-nongps-navigation.html)

### Others

1. [Arduino + Mavlink](https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566)
1. [Dronekit Python](https://github.com/dronekit/dronekit-python/blob/master/docs/guide/copter/guided_mode.rst)


Specify the date-time on a remote machine that has no access to Internet:
```
ssh pi@10.5.1.190 sudo date -s @`( date -u +"%s" )`
```
