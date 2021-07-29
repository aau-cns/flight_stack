#!/usr/bin/env python

import sys
import rospy
import numpy as np
from rosbag import Bag
from ublox_msgs.msg import NavPVT

# Global timeout, epsilon and rate
timeout_rtks = np.zeros([1,2])
timeout_rtks[0,:] = [2, 2]
eps = 1e-6
rate = 0.01

class gps_mode:
	def __init__(self):

		# Subscribers
		self.sub_rtk_1 = rospy.Subscriber('/rtk_gps_1/navpvt', NavPVT, self.callback, (1), queue_size = 1)
		self.sub_rtk_2 = rospy.Subscriber('/rtk_gps_2/navpvt', NavPVT, self.callback, (2), queue_size = 1)

		# Define bit matrix where each column correspond to a single rtk (init to 255)
		self.__mode_bit_rtks = np.zeros([1,2], dtype=np.dtype('uint8'))
		self.__mode_bit_rtks[0,:] = [-1, -1]

	def callback(self, msg_in, rtk_idx):

		# Subtract 1 since numpy idx start from 0
		rtk_idx -= 1

		# Check if timeout was invalid, this means that the rtk publish again
		if timeout_rtks[0,rtk_idx] == -1:
			print("[WARNING] RTK Gps #{} is publishing again".format(rtk_idx+1))

		# Check the index and reset the timeout
		timeout_rtks[0,rtk_idx] = 2
		
		# Check if rtk fix is ok
		rtk_fix_ok = msg_in.flags & msg_in.FLAGS_GNSS_FIX_OK

		if rtk_fix_ok:

			# Extract the mode_bit, if float 0, if fixed 1
			if msg_in.flags & msg_in.CARRIER_PHASE_FLOAT:
				mode_bit = np.uint8(0)
			elif msg_in.flags & msg_in.CARRIER_PHASE_FIXED:
				mode_bit = np.uint8(1)
			else:
				mode_bit = np.uint8(-1)
			
			# Check if mode_bit changed wrt value at previous message
			if self.__mode_bit_rtks[0,rtk_idx] != mode_bit:

				# Change actual mode bit
				self.__mode_bit_rtks[0,rtk_idx] = mode_bit

				# Print new mode_bit
				if mode_bit == np.uint8(0):
					print("[WARNING] RTK Gps #{} changed mode to: FLOAT".format(rtk_idx+1))
				elif mode_bit == np.uint8(1):
					print("[WARNING] RTK Gps #{} changed mode to: FIXED".format(rtk_idx+1))
				elif mode_bit == np.uint8(-1):
					print("[WARNING] RTK Gps #{} changed mode to: NOTHING (NEITHER FLOAT NOR FIXED)".format(rtk_idx+1))
		else:
			print("[WARNING] RTK Gps #{} GNSS FIX: NOT OK".format(rtk_idx+1))

def timeout_update(event):

	# Update timeout if it is valid
	if timeout_rtks[0,0] > -1:
		timeout_rtks[0,0] = timeout_rtks[0,0] - rate
	if timeout_rtks[0,1] > -1:
		timeout_rtks[0,1] = timeout_rtks[0,1] - rate

	# Print if timeout reach zero and set to invalid (-1)
	if -eps < timeout_rtks[0,0] < eps:
		print("[WARNING] RTK Gps #1 is not publishing for more than 2 seconds")
		timeout_rtks[0,0] = -1
	if -eps < timeout_rtks[0,1] < eps:
		print("[WARNING] RTK Gps #2 is not publishing for more than 2 seconds")
		timeout_rtks[0,1] = -1
				
        
def main(args):

	# Init node
	rospy.init_node('gps_mode', anonymous=True)

	# Instance
	gm = gps_mode()

	# Timer update at 10Hz
	rospy.Timer(rospy.Duration(rate), timeout_update)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':

    main(sys.argv)
