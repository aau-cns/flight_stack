rosbag record --split --size=500 -o asctec_thl \
	/asctec_hummingbird/vrpn_client/raw_pose \
        /fcu/imu_custom /fcu/mag \
	/camera/accel/imu_info /camera/accel/sample /camera/gyro/imu_info /camera/gyro/sample \
	/camera/odom/sample

