#!/usr/bin/env python

import sys
import time
import rospy
from prob_msgs.srv import *
import ast


# ROS wrapper for move_joint
def move_joint(axis, deg, velocity=None, acceleration=None, block=True, relative=False):
    rospy.wait_for_service('move_joint')
    try:
        move_joint_proxy = rospy.ServiceProxy('move_joint', MoveJoint)
        resp1 = move_joint_proxy(axis, deg, velocity, acceleration, block, relative)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for move_tool
def move_tool(x, y, z, orientation=None, velocity=None, acceleration=None, velocity_rot=None, acceleration_rot=None,
              block=True, relative=None, frame="base"):
    rospy.wait_for_service('move_tool')
    try:
        move_tool_proxy = rospy.ServiceProxy('move_tool', MoveTool)
        resp1 = move_tool_proxy(x, y, z, orientation, velocity, acceleration, velocity_rot, acceleration_rot, block, relative,
                          frame)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for move_to_pose
def move_to_pose(name, velocity=None, acceleration=None, block=True):
    rospy.wait_for_service('move_to_pose')
    try:
        move_to_pose_proxy = rospy.ServiceProxy('move_to_pose', MoveToPose)
        resp1 = move_to_pose_proxy(name, velocity, acceleration, block)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for initialization
def initialize(model='PRob2R', kind='real', channel_name='/dev/pcanpci0', channel_type='PEAK_SYS_PCAN_PCI',
               protocol='TMLCAN', host_id='10', baudrate='1000000'):
    rospy.wait_for_service('initialize')
    if channel_name != '/dev/pcanpci0':
        channel_type = 'PEAK_SYS_PCAN_USB'
    try:
        initialize_proxy = rospy.ServiceProxy('initialize', Initialize)
        resp1 = initialize_proxy(model, kind, channel_name, channel_type, protocol, host_id, baudrate)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for calibration
def calibrate(script_id=0):
    rospy.wait_for_service('calibrate')
    try:
        calibrate_proxy = rospy.ServiceProxy('calibrate', Calibrate)
        resp1 = calibrate_proxy(script_id)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for test_script
def test_script(script_code='', script_type='main'):
    rospy.wait_for_service('test_script')
    try:
        test_script_proxy = rospy.ServiceProxy('test_script', TestScript)
        resp1 = test_script_proxy(script_code, script_type)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for wait_for_robot
def wait_for_robot():
    # Wait some time for status to be updated
    time.sleep(0.1)
    rospy.wait_for_service('wait_for_robot')
    try:
        wait_for_robot_proxy = rospy.ServiceProxy('wait_for_robot', Empty)
        resp1 = wait_for_robot_proxy(1)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for wait_for_robot
def finalize():
    rospy.wait_for_service('finalize')
    try:
        finalize_proxy = rospy.ServiceProxy('finalize', Empty)
        resp1 = finalize_proxy(1)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for execute_script
def execute_script(script_id=1):
    rospy.wait_for_service('execute_script')
    try:
        execute_script_proxy = rospy.ServiceProxy('execute_script', ExecuteScript)
        resp1 = execute_script_proxy(script_id)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for release
def release(joint_id=None):
    if joint_id is None:
        joint_id = '123456'
    else:
        # Remove all separators if use added some
        joint_id = str(joint_id)
        joint_id = joint_id.replace('[', '')
        joint_id = joint_id.replace(']', '')
        joint_id = joint_id.replace(',', '')
        joint_id = joint_id.replace(' ', '')

    rospy.wait_for_service('release')
    try:
        release_proxy = rospy.ServiceProxy('release', Release)
        resp1 = release_proxy(joint_id)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for hold
def hold(joint_id=None):
    if joint_id is None:
        joint_id = b'123456'
    else:
        # Remove all separators if use added some
        joint_id = str(joint_id)
        joint_id = joint_id.replace('[', '')
        joint_id = joint_id.replace(']', '')
        joint_id = joint_id.replace(',', '')
        joint_id = joint_id.replace(' ', '')

    rospy.wait_for_service('hold')
    try:
        hold_proxy = rospy.ServiceProxy('hold', Release)
        resp1 = hold_proxy(joint_id)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for open_gripper
def open_gripper(position=None, velocity=None, acceleration=None):
    rospy.wait_for_service('open_gripper')
    try:
        open_gripper_proxy = rospy.ServiceProxy('open_gripper', OpenGripper)
        resp1 = open_gripper_proxy(position, velocity, acceleration)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for close_gripper
def close_gripper(velocity=None, acceleration=None, current=None):
    rospy.wait_for_service('close_gripper')
    try:
        close_gripper_proxy = rospy.ServiceProxy('close_gripper', CloseGripper)
        resp1 = close_gripper_proxy(velocity, acceleration, current)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for execute_script_from_file
def execute_script_from_file(filename):
    fo = open(filename, "r")
    script = fo.read()
    script = script.replace("\n", "\\\\n")
    rospy.wait_for_service('test_script')
    try:
        test_script_proxy = rospy.ServiceProxy('test_script', TestScript)
        resp1 = test_script_proxy(script)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_status_info
def get_status_info():
    rospy.wait_for_service('get_status_info')
    try:
        get_status_info_proxy = rospy.ServiceProxy('get_status_info', GetInfo)
        resp1 = get_status_info_proxy()
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_connection_info
def get_connection_info():
    rospy.wait_for_service('get_connection_info')
    try:
        get_connection_info_proxy = rospy.ServiceProxy('get_connection_info', GetInfo)
        resp1 = get_connection_info_proxy()
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_application_info
def get_application_info():
    rospy.wait_for_service('get_application_info')
    try:
        get_application_info_proxy = rospy.ServiceProxy('get_application_info', GetAppInfo)
        resp1 = get_application_info_proxy()
        return {'script_id': resp1.id, 'script_name':resp1.name}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_print_info
def get_application_output():
    rospy.wait_for_service('get_connection_info')
    try:
        get_application_output_proxy = rospy.ServiceProxy('get_application_output', GetInfoString)
        resp1 = get_application_output_proxy()
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_actuator_release_state
def get_actuator_release_state():
    rospy.wait_for_service('get_actuator_release_state')
    try:
        get_actuator_release_state_proxy = rospy.ServiceProxy('get_actuator_release_state', GetBoolArray)
        resp1 = get_actuator_release_state_proxy()
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_kinematic_indices
def get_kinematic_indices():
    rospy.wait_for_service('get_kinematic_indices')
    try:
        get_kinematic_indices_proxy = rospy.ServiceProxy('get_kinematic_indices', GetStringArray)
        resp1 = get_kinematic_indices_proxy()
        return [int(index) for index in resp1.res ]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_actuator_indices
def get_actuator_indices():
    rospy.wait_for_service('get_actuator_indices')
    try:
        get_actuator_indices_proxy = rospy.ServiceProxy('get_actuator_indices', GetArray)
        resp1 = get_actuator_indices_proxy()
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_message_info
def get_message_info():
    rospy.wait_for_service('get_message_info')
    try:
        get_message_info_proxy = rospy.ServiceProxy('get_message_info', GetInfoString)
        resp1 = get_message_info_proxy()
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ROS wrapper for get_all_status
def get_all_status():
    rospy.wait_for_service('get_all_status')
    try:
        get_all_status_proxy = rospy.ServiceProxy('get_all_status', GetInfoString)
        resp1 = get_all_status_proxy()
        return ast.literal_eval(resp1.res)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
