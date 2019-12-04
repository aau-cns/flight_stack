rosbag record --split --size=500 -o outdoor /mavros/imu/data_raw /mavros/imu/mag  \
/camera/accel/imu_info /camera/accel/sample /camera/gyro/imu_info /camera/gyro/sample \
/camera/odom/sample /camera/realsense2_camera_manager/bond \
/camera/tracking_module/parameter_descriptions /camera/tracking_module/parameter_updates \
#/camera/fisheye1/camera_info /camera/fisheye1/image_raw /camera/fisheye2/camera_info /camera/fisheye2/image_raw \
/mavros/global_position/compass_hdg \
/mavros/global_position/raw/fix /mavros/global_position/raw/gps_vel /mavros/global_position/raw/satellites \
/mavros/global_position/rel_alt \
/mavros/battery
