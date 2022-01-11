rosbag record --split --size=500 -o asctec_thl \
	/asctec_hummingbird/vrpn_client/raw_pose \
        /fcu/imu_custom /fcu/mag \
	/realsense/accel/imu_info /realsense/accel/sample /realsense/gyro/imu_info /realsense/gyro/sample \
	/realsense/odom/sample

