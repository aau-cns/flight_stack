# This message holds the status of an individual component of the robot.
#

# Possible levels of operations
uint8 IMMEDIATE=0     # the action of the trigger should be performed immediately
uint8 SCHEDULE=1      # the action of the trigger should be performed at: t_exec = header.timestamp + action_delay_sec

Header header
string action_description       # description of the action
uint8 action_type               # event of operation enumerated above
float32 action_delay_sec         # delay of the action in [sec]
