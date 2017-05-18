#!/usr/bin/env python

import sys
import rospy
from client_functions import *

if __name__ == "__main__":

    while True:
        rospy.sleep(2)
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("Current Robot Status:")
        print("_____________________________________")
        print("For Connection Status:")
        print("0: not Initialized, not Calibrated")
        print("1: Initializing, not Calibrated")
        print("2: Initialized, not Calibrated")
        print("3: Initialized, Calibrating")
        print("4: Initialized, Calibrated")
        print("_____________________________________")
        print("For Status Info:")
        print("0: None")
        print("1: Ready")
        print("2: Stopped")
        print("3: Paused")
        print("4: Running")
        print("5: Reased")
        print("6: Error")
        print("7: Recording")
        print("8: Calibrating")
        print("9: Processing")
        print("_____________________________________")
        print "Connection Status: ", get_connection_info()
        print "Status Info: ", get_status_info()
        #print("Current Joint Angles:", get_joint_angles)
        
        
