
# Tutorial on How to add new sensors to MaRS

This tutorial explains on how to add new sensors to [MaRS framework](https://github.com/aau-cns/mars_lib) and how to use the new sensor modules in ROS. This guide strictly follows the [online MaRS tutorials](https://github.com/aau-cns/mars_lib#tutorials).

## Installation
This installs the [MaRS ROS framework](https://github.com/aau-cns/mars_ros), which in turn installs the [MaRS library](https://github.com/aau-cns/mars_lib). Please also refer to the [online tutorial](https://github.com/aau-cns/mars_ros#getting-started), which also provides a guide on how to use MaRS in a precompiled docker environment.

1. Create a new worspace root folder and `src` folder in it.
1. Initialize the workspace with `catkin init` (we typically use the `catkin_tools`, you might need to install them)
1. cd into the `src` folder and `git clone` the Mars ROS repository
1. cd into mars_ros and do a `git sumbodule update --recursive --init`. This will download also the corresponding Mars framework library.
1. cd into your workspace root and compile the workspace with `catkin build`


## Adding a new Sensor

### Library
To add a new sensor in the MaRS framework, first the sensor measurement functions and updates have to be implemented. For this example implementation a `pose` sensor is referenced with the [measurement equation](https://github.com/aau-cns/mars_lib#pose-6-dof)


\begin{align}
z_{p} =& \text{\textbf{P}}_{WI} + \text{\textbf{R}}_{WI} ~ \text{\textbf{P}}_{IP} \\
z_{q} =& \text{\textbf{q}}_{WI} \otimes \text{\textbf{q}}_{IP}
\end{align}


<!-- ![](https://latex.codecogs.com/svg.latex?z_{p}=\text{\textbf{p}}_{WI}+\text{\textbf{R}}_{WI}~\text{\textbf{p}}_{IP})
![](https://latex.codecogs.com/svg.latex?z_{q}=\text{\textbf{q}}_{WI}\otimes\text{\textbf{q}}_{IP}) -->


To add the new sensor:

1. Create a new folder in `mars_ros/mars_lib/source/mars/include/mars/<SENSOR_NAME>`
1. Create 3 header files inside this folder (or copy them from e.g. the `pose` sensor)
    - `<SENSOR_NAME>_measurement_type.h`
    - `<SENSOR_NAME>_sensor_state_type.h`
    - `<SENSOR_NAME>_sensor_class.h`
1. (optional) for better usage, we typically also create the corresponding `.cpp` files and only have the definition in the `.h` file. This is up to the user however.
1. `<SENSOR_NAME>_measurement_type.h` defines the data type class of the measurement. E.g. for the `pose` sensor it includes
    - position: `Eigen::Vector3d position_`
    - attitude: `Eigen::Quaternion<double> orientation_;`
1. `<SENSOR_NAME>_sensor_state_type.h` defines additional (auxiliary) states, that a sensor needs to update the core states used in the EKF. In the case of the `pose` sensors these are the calibration states between the inertial frame I and the pose sensor frame P, i.e.
    - `p_ip_` translational calibration state
    - `q_ip_` orientational calibration state
1. `<SENSOR_NAME>_sensor_class.h` defines on how the core states and auxiliary states are initialized, updated, and residuals/Jacobians are calculated. Taking the `pose` sensor as an example:
    1.
    1. Write the initialization of the additional sensor states in `BufferDataType Initialize(...)`
    1. In `bool CalcUpdate(...)` depending on the sensor measurement model implement the
        1. Measurement noise matrix calculation
        1. Implementation of the measurement jacobian H
        1. Residual calculation
    1. In `<SENSOR_NAME>StateType ApplyCorrection(...)` implement the auxiliary sensor state corrections depending on the measurement model.
1. Finally add the new files to the `mars_ros/mars_lib/source/mars/CMakeLists.txt` under the `set(headers ...)` path. Additionally if you have added `.cpp` files put them also into the `set(sources ...)` declaration

If you perform these steps then the sensor can be used as a generalized object in the MaRS framework.

## ROS
To add the newly created sensor in the ROS-based MaRS framework

1. Create (or copy) a new wrapper class in `mars_ros/include/mars_wrapper_<SENSOR_NAME).h` and the corresponding `.cpp` file in `mars_ros/src/mars_wrapper_<SENSOR_NAME).cpp`
1. Copy the `ImuMeasurementCallback` from any of the other sensors
1. Add ROS callback functions for all the sensors used. Each function should perform the following:
    1. Derive the timestamp
    1. Convert the ROS message to the MaRS measruement type
    1. Perform the by calling the `ProcessMeasurement` function of the `core_logic`
1. Copy or add the `RunCoreStatePublisher()` method.

Finally, also add the new files to the `mars_ros/CMakeLists.txt`, e.g. for the pose sensor:

```cmake
add_executable(pose_node src/mars_node.cpp src/mars_wrapper_pose.cpp)
add_dependencies(pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} mars_lib)
target_link_libraries(pose_node
  ${catkin_LIBRARIES} ${MARS_LIBRARIE}
)
set_property(TARGET pose_node PROPERTY COMPILE_DEFINITIONS POSE)
```

Also update the `mars_ros/src/mars_node.cpp` to include the new wrapper.
