rosbag record --split --size=500 -o optitrac_pose \
	/mavros/imu/data_raw /mavros/imu/mag \
	/Twins/vrpn_client/raw_transform \
	/realsense/accel/imu_info /realsense/accel/sample /realsense/gyro/imu_info /realsense/gyro/sample \
	/realsense/odom/sample

