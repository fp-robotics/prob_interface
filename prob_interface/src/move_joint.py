#!/usr/bin/env python

import sys
import rospy
from prob_interface.srv import *
from client_functions import*



def usage():
    return "%s [axis deg (velocity acceleration block relative)]"%sys.argv[0]


if __name__ == "__main__":
	arg_length = len(sys.argv)
	velocity = None
	acceleration = None
	block = True
	relative = False
	if 3 <= arg_length <= 7:
		axis = int(sys.argv[1])
		deg = int(sys.argv[2])
		if arg_length > 3:
			velocity = int(sys.argv[3])
		if arg_length > 4:
			acceleration = int(sys.argv[4])
		if arg_length > 5:
			block = bool(sys.argv[5])
		if arg_length > 6:
			relative = bool(sys.argv[6])
	else:
		print usage()
		sys.exit(1)
	print "Requesting movement of axis %s to position %s deg"%(axis, deg)
	move_joint(axis, deg, velocity, acceleration, block, relative)
