#!/usr/bin/env python

import sys
import rospy
import time

from client_functions import *

if __name__ == "__main__":
    if get_connection_info() == 4 and get_status_info() == 1:
        print("App Info: ", get_application_info())
        
        print("Kin Indices: ", get_kinematic_indices())
        
        # TODO: check value
        print("App Output: ", get_application_output())
        print("Release state: ", get_actuator_release_state())

        #print(get_all_status())

        move_to_pose('back_pose')

        """
        move_joint(5,90,60,90)
        wait_for_robot()
        
        move_joint(5,-90,60,90)
        wait_for_robot()
        
        #	move_tool(10,10,10,relative=True,frame="tool")
        #	wait_for_robot()
        
        open_gripper(20,45,60)
        wait_for_robot()
        
        close_gripper(45,60)
        wait_for_robot()
        
        
        
        print("Release")
        release('[6]')
        rospy.sleep(10)
        print("Hold
        hold()

        """
    else:
        print('Robot is not ready to execute commands. Something with initialization and calibration went wrong.')
        print(get_message_info())
    
    sys.exit(0)
        
