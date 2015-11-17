#!/usr/bin/env python

import sys
import rospy

from client_functions import *

def usage():
    return "%s [filename]"%sys.argv[0]

if __name__ == "__main__":
	arg_length = len(sys.argv)
	
	if arg_length == 2:
		filename = sys.argv[1]
	else:
		print usage()
		sys.exit(1)
	print "Executing Script from File %s"%(filename)
	execute_script_from_file(filename)
