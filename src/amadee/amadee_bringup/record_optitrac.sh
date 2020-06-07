rosbag record --split --size=500 -o optitrac_pose \
	/mavros/imu/data_raw /mavros/imu/mag \
	/Twins/vrpn_client/raw_transform \
	/camera/accel/imu_info /camera/accel/sample /camera/gyro/imu_info /camera/gyro/sample \
	/camera/odom/sample

