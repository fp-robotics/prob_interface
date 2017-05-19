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

        move_tool(-700, 0, 820, [0, 0, 180], relative=False, frame="base")
        wait_for_robot()

        #print(get_all_status())

        move_joint(5, 0)
        wait_for_robot()
        move_to_pose('back_pose')
        wait_for_robot()

        move_joint(5,90,60,90)
        wait_for_robot()
        
        move_joint(5,-90,60,90)
        wait_for_robot()
        

        
        open_gripper(20,45,60)
        wait_for_robot()
        
        close_gripper(45,60)
        wait_for_robot()
        
        
        
        print("Release")
        release()
        rospy.sleep(10)
        print("Hold")
        hold()

    else:
        print('Robot is not ready to execute commands. Something with initialization and calibration went wrong.')
        print(get_message_info())
    
    sys.exit(0)
        
