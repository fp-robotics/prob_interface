#!/usr/bin/env python

import sys
import rospy

from client_functions import *


def usage():
    return "%s [script_id=0 robot_model='PRob2R' ]" % sys.argv[0]

if __name__ == "__main__":
    arg_length = len(sys.argv)
    model = 'PRob2R'
    script_id = -1
    channel_name = '/dev/pcanpci0'
    if 1 <= arg_length <= 4:
	if arg_length > 1:
	    script_id = int(sys.argv[1])
	if arg_length > 2:
	    model = str(sys.argv[2])
	if arg_length > 3:
	    channel_name = str(sys.argv[3])
    else:
	print usage()
	sys.exit(1)
    # initialize and calibrate robot
    initialize(model, "real", channel_name)
    wait_for_robot()
    calibrate(script_id)
    wait_for_robot()
    print("Connected to PRob.")
    print("Robot Status:")
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
    print "Connection: ",get_connection_info()
    print "Status: ", get_status_info()

