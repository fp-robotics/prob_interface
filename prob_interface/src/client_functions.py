#!/usr/bin/env python

import sys
import rospy
from prob_msgs.srv import *
import ast

# ROS wrapper for move_joint
def move_joint(axis, deg, velocity=None, acceleration=None, block=True, relative=False):
    rospy.wait_for_service('move_joint')
    try:
        move_joint = rospy.ServiceProxy('move_joint', MoveJoint)
        resp1 = move_joint(axis, deg, velocity, acceleration, block, relative)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# ROS wrapper for move_tool
def move_tool(x, y, z, phi=None, theta=None, psi=None, velocity=None, acceleration=None, velocity_rot=None, acceleration_rot=None, block=True, relative=False):
	rospy.wait_for_service('move_tool')
	try:
		move_tool = rospy.ServiceProxy('move_tool', MoveTool)
		resp1 = move_tool(x, y, z, phi, theta, psi, velocity, acceleration, velocity_rot, acceleration_rot, block, relative)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for move_to_pose
def move_to_pose(name, velocity=None, acceleration=None, block=True):
	rospy.wait_for_service('move_to_pose')
	try:
		move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
		resp1 = move_to_pose(name, velocity, acceleration, block)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	
# ROS wrapper for initialization
def initialize(model='PRob1R', kind='real', channel_name='1', channel_type='PEAK_SYS_PCAN_USB', protocol='TMLCAN', host_id='10', baudrate='500000'):
	rospy.wait_for_service('initialize')
	try:
		initialize = rospy.ServiceProxy('initialize', Initialize)
		resp1 = initialize(model, kind, channel_name, channel_type, protocol, host_id, baudrate)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for calibration
def calibrate(use_existing=True):
	rospy.wait_for_service('calibrate')
	try:
		calibrate = rospy.ServiceProxy('calibrate', Calibrate)
		resp1 = calibrate(use_existing)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for test_script
def test_script(script_code=''):
	rospy.wait_for_service('test_script')
	try:
		test_script = rospy.ServiceProxy('test_script', TestScript)
		resp1 = test_script(script_code)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for wait_for_robot
def wait_for_robot(show = False):
	rospy.wait_for_service('wait_for_robot')
	try:
		wait_for_robot = rospy.ServiceProxy('wait_for_robot', WaitForRobot)
		resp1 = wait_for_robot(show)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


# ROS wrapper for execute_script
def execute_script(script_id=1):
	rospy.wait_for_service('execute_script')
	try:
		execute_script = rospy.ServiceProxy('execute_script', ExecuteScript)
		resp1 = execute_script(script_id)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for release
def release(joint_id=[1]):
	joint_id = str(joint_id)
	joint_id = joint_id.replace('[','')
	joint_id = joint_id.replace(']','')
	joint_id = joint_id.replace(',','')
	joint_id = joint_id.replace(' ','')
	rospy.wait_for_service('release')
	try:
		release = rospy.ServiceProxy('release', Release)
		resp1 = release(joint_id)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for hold
def hold(joint_id = [1]):
	joint_id = str(joint_id)
	joint_id = joint_id.replace('[','')
	joint_id = joint_id.replace(']','')
	joint_id = joint_id.replace(',','')
	joint_id = joint_id.replace(' ','')
	rospy.wait_for_service('hold')
	try:
		hold = rospy.ServiceProxy('hold', Release)
		resp1 = hold(joint_id)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for open_gripper
def open_gripper(position=None, velocity=None, acceleration=None):
	rospy.wait_for_service('open_gripper')
	try:
		open_gripper = rospy.ServiceProxy('open_gripper', OpenGripper)
		resp1 = open_gripper(position, velocity, acceleration)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for close_gripper
def close_gripper(velocity=None, acceleration=None, current=None):
	rospy.wait_for_service('close_gripper')
	try:
		close_gripper = rospy.ServiceProxy('close_gripper', CloseGripper)
		resp1 = close_gripper(velocity, acceleration, current)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for execute_script_from_file
def execute_script_from_file(filename):
	fo = open(filename,"r")
	script = fo.read()
	script = script.replace("\n", "\\\\n")
	rospy.wait_for_service('test_script')
	try:
		test_script = rospy.ServiceProxy('test_script', TestScript)
		resp1 = test_script(script)
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for get_status_info
def get_status_info():
	rospy.wait_for_service('get_status_info')
	try:
		get_status_info = rospy.ServiceProxy('get_status_info', GetInfo)
		resp1 = get_status_info()
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for get_connection_info
def get_connection_info():
	rospy.wait_for_service('get_connection_info')
	try:
		get_connection_info = rospy.ServiceProxy('get_connection_info', GetInfo)
		resp1 = get_connection_info()
		return resp1.res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

# ROS wrapper for get_all_status
def get_all_status():
	rospy.wait_for_service('get_all_status')
	try:
		get_all_status = rospy.ServiceProxy('get_all_status', GetInfo)
		resp1 = get_all_status()
		return ast.literal_eval(resp1.res)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
