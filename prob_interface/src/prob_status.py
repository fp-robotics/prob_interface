#!/usr/bin/env python

import sys
import rospy
from client_functions import *

if __name__ == "__main__":

    while True:
        rospy.sleep(2)
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("Current Robot Status:")
        print("_______________________________________________________")
        print("For Connection Status:               | For Status Info:")
        print("0: not Initialized, not Calibrated   | None")
        print("1: Initializing, not Calibrated      | Ready")
        print("2: Initialized, not Calibrated       | Stopped")
        print("3: Initialized, Calibrating          | Paused")
        print("4: Initialized, Calibrated           | Running")
        print("5:                                   | Released")
        print("6:                                   | Error")
        print "Connection Status: ", get_connection_info()
        print "Status Info: ", get_status_info()
        #print("Current Joint Angles:", get_joint_angles)
