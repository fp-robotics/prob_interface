#!/usr/bin/env python

import sys
import rospy

from client_functions import *

if __name__ == "__main__":
    if get_connection_info() == 4 and get_status_info() == 1:
	
        move_joint(5,90,60,90)
	wait_for_robot()
	open_gripper(20,45,60)
	wait_for_robot()
	close_gripper(45,60)
	wait_for_robot()
	test_script('open_gripper()\\\\nclose_gripper()')
	wait_for_robot()
	release([1,2,3,4,5,7])
	rospy.sleep(10)
	hold([1,2,3,4,5,7])
	#filename = '/home/fp_administrator/scripts/test_script.script'
	#execute_script_from_file(filename)
    else:
        print('Robot is not ready to execute commands. Be sure it is initialized and caibrated.')
	sys.exit(1)
        
