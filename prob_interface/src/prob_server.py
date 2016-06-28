#!/usr/bin/env python

from prob_msgs.srv import *
from prob_msgs.msg import *
import rospy
import sys
import json
import traceback
import time
from thread import *
from sockjs_client import SockJSClient

robot_arm = 0

# myP function handles
def handle_move_joint(req):
  print("move_joint(%s, %s, %s, %s, %s, %s)"%(req.axis, req.deg, req.vel, req.acc, bool(req.block), bool(req.relative)))
  robot_arm.move_joint( req.axis, req.deg, req.vel, req.acc, bool(req.block), bool(req.relative))
  return 1

def handle_move_tool(req):
  print("move_tool(%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"%( req.x, req.y, req.z, req.orientation, req.velocity, req.acceleration, req.velocity_rot, req.acceleration_rot, bool(req.block), bool(req.relative), req.frame))
  robot_arm.move_tool( req.x, req.y, req.z, req.orientation, req.velocity, req.acceleration, req.velocity_rot, req.acceleration_rot, bool(req.block), bool(req.relative), req.frame)
  return 1
  
def handle_move_to_pose(req):
  print("move_to_pose(%s, %s, %s, %s, %s)"%(req.name, req.vel, req.acc, req.block))
  robot_arm.move_to_pose( req.name, req.vel, req.acc, req.block)
  return 1
  
def handle_wait_for_robot(req):
  print("wait_for_robot()")
  robot_arm.wait_for_robot()
  return 1
  
def handle_finalize(req):
  print("finalize()")
  robot_arm.finalize()
  return 1
  

def handle_wait(req):
  print("wait(%s)"%(req.time))
  robot_arm.wait( req.time)
  return 1

def handle_initialize(req):
  print("initialize(%s, %s, %s, %s, %s, %s, %s)"%(req.model, req.kind, req.channel_name, req.channel_type, req.protocol, req.host_id, req.baudrate))
  robot_arm.initialize(req.model, req.kind, req.channel_name, req.channel_type, req.protocol, req.host_id, req.baudrate)
  return 1

def handle_calibrate(req):
  print("req: ", req)
  print("calibrate(%s)"%(req.use_existing))
  robot_arm.calibrate(req.use_existing)
  return 1

def handle_test_script(req):
  print("test_script(%s)"%(req.script_code))
  robot_arm.test_script( req.script_code)
  return 1

def handle_execute_script(req):
  print("execute_script(%s)"%(req.script_id))
  robot_arm.execute_script(req.script_id)
  return 1

def handle_release(req):
  print("release(%s)"%(req.joint_id))
  robot_arm.release(req.joint_id)
  return 1

def handle_hold(req):
  print("hold(%s)"%(req.joint_id))
  robot_arm.hold(req.joint_id)
  return 1

def handle_open_gripper(req):
  print("open_gripper(%s, %s, %s)"%(req.position, req.velocity, req.acceleration))
  robot_arm.open_gripper(req.position, req.velocity, req.acceleration)
  return 1

def handle_close_gripper(req):
  print("close_gripper(%s, %s, %s)"%(req.velocity, req.acceleration, req.current))
  robot_arm.close_gripper(req.velocity, req.acceleration)
  return 1

def handle_get_status_info(req):
  print("get_status_info()")
  status = robot_arm.get_status_info()
  print status
  return status

def handle_get_connection_info(req):
  print("get_connection_info()")
  connection_info = robot_arm.get_connection_info()
  return connection_info

def handle_get_message_info(req):
  print("get_message_info()")
  status = robot_arm.get_message_info()
  return GetInfoStringResponse(str(status))

def handle_get_application_info(req):
  print("get_application_info()")
  status = robot_arm.get_application_info()
  return status

def handle_get_actuator_release_state(req):
  print("get_actuator_release_state()")
  status = robot_arm.get_actuator_release_state()
  return status

def handle_get_print_info(req):
  print("get_print_info()")
  status = robot_arm.get_print_info()
  return status

def handle_get_kinematic_indices(req):
  print("get_kinematic_indices()")
  array = robot_arm.get_kinematic_indices()
  return array

def handle_get_all_status(req):
  print("get_all_status()")
  status = robot_arm.get_all_status()
  status = str(status)
  return GetInfoStringResponse(status)

def handle_get_actuator_indices(req):
  print("get_actuator_indices()")
  array = robot_arm.get_actuator_indices()
  return array

def handle_get_gripper_angle(req):
  print("get_gripper_angle()")
  robot_arm.get_gripper_angle()

class RobotHandler:

    # 0: 'not Initialized, not Calibrated'
    # 1: 'Initializing, not Calibrated'
    # 2: 'Initialized, not Calibrated'
    # 3: 'Initialized, Calibrating'
    # 4: 'Initialized, Calibrated'
    @staticmethod
    def get_connection_info():
        return myp.get_status("connection_info")
    # 0: 'None'
    # 1: 'Ready'
    # 2: 'Stopped'
    # 3: 'Paused'
    # 4: 'Running'
    # 5: 'Released'
    # 6: 'Error'

    @staticmethod
    def get_status_info():
        return myp.get_status("status_info")

    @staticmethod
    def get_message_info():
        return myp.get_status("message_info")

    @staticmethod
    def get_application_info():
        return myp.get_status("application_info")

    @staticmethod
    def get_actuator_release_state():
        return [not state for state in myp.get_status("actuator_release_state")]

    @staticmethod
    def get_position():
        return myp.get_status("current_pose")

    @staticmethod
    def get_actuator_angles():
        return myp.get_status("current_actuator_angles")

    @staticmethod
    def get_posture():
        return myp.get_status("current_posture")

    @staticmethod
    def get_current():
        return myp.get_status("current_current")

    @staticmethod
    def get_print_info():
        return myp.get_status("print_info")

    @staticmethod
    def get_kinematic_indices():
        return myp.get_status("kinematic_indices")

    @staticmethod
    def get_all_status():
        return myp.status

    def get_actuator_indices(self):
        return myp.get_status("actuator_indices")

    #####################################
    # initialization functions          #
    #####################################
    @staticmethod
    def initialize(model="PRob2R", kind="real", channel_name="/dev/pcanpci0",
                   channel_type="PEAK_SYS_PCAN_PCI", protocol="TMLCAN", host_id="10", baudrate="500000"):
        if RobotHandler.get_connection_info() == 0:
            cmd = "{"
            cmd += "\"action\":\"initialize\","
            cmd += "\"model\":\"" + model + "\","
            cmd += "\"robot_kind\":\"" + kind + "\""
            if kind == 'real':
                cmd += ","
                cmd += "\"channel_name\":\"" + channel_name + "\","
                cmd += "\"channel_type\":\"" + channel_type + "\","
                cmd += "\"protocol\":\"" + protocol + "\","
                cmd += "\"host_id\":" + host_id + ","
                cmd += "\"baudrate\":" + baudrate
            cmd += "}"
            myp.send(cmd)
            return 1
        return 0

    @staticmethod
    def finalize():
        cmd = "{"
        cmd += "\"action\":\"finalize\""
        cmd += "}"
        myp.send(cmd)
        time.sleep(0.5)
        cmd = "{"
        cmd += "\"action\":\"answer_dialog\","
        cmd += "\"dialog_answer\":true"
        cmd += "}"
        myp.send(cmd)
        return 1
    
    #####################################
    # status functions                  #
    #####################################
    @staticmethod
    def pause():
        cmd = "{"
        cmd += "\"action\":\"pause\""
        cmd += "}"
        myp.send(cmd)
        return 1

    @staticmethod
    def resume():
        cmd = "{"
        cmd += "\"action\":\"resume\""
        cmd += "}"
        myp.send(cmd) 
        return 1

    @staticmethod
    def stop():
        cmd = "{"
        cmd += "\"action\":\"stop\""
        cmd += "}"
        myp.send(cmd) 
        return 1

    @staticmethod
    def recover():
        cmd = "{"
        cmd += "\"action\":\"recover\""
        cmd += "}"
        myp.send(cmd)
        return 1
        
    #####################################
    # control functions                 #
    #####################################
    @staticmethod
    def calibrate(use_existing=True):
        # if RobotHandler.get_connection_info() == 2:
        cmd = "{"
        cmd += "\"action\": \"calibrate\","
        cmd += "\"use_existing\": " + str(use_existing).lower()
        cmd += "}"
        myp.send(cmd)
        return 1

    @staticmethod
    def release(joint_id=None):
        if not joint_id:
            joint_id = [int(i) for i in RobotHandler.get_kinematic_indices()]
        length = len(joint_id)
        processed_string = '['
        for i in range(0, length - 1):
            processed_string += "\"" + str(joint_id[i]) + "\","
        processed_string += "\"" + str(joint_id[length - 1]) + "\"]"

        cmd = "{"
        cmd += "\"action\": \"release\","
        cmd += "\"joint_ids\": " + processed_string
        cmd += "}"
        myp.send(cmd)
        return 1

    @staticmethod
    def hold(joint_id=None):
        if not joint_id:
            joint_id = [int(i) for i in RobotHandler.get_kinematic_indices()]
        length = len(joint_id)
        processed_string = '['
        for i in range(0, length - 1):
            processed_string += "\"" + str(joint_id[i]) + "\","
        processed_string += "\"" + str(joint_id[length - 1]) + "\"]"

        cmd = "{"
        cmd += "\"action\": \"hold\","
        cmd += "\"joint_ids\": " + processed_string
        cmd += "}"
        myp.send(cmd)
        return 1

    @staticmethod
    def execute_script(script_name):
        script_id = RobotHandler._get_id_from_name(script_name, "scripts")
        cmd = "{"
        cmd += "\"action\":\"execute_script\","
        cmd += "\"script_id\":" + str(script_id)
        cmd += "}"
        myp.send(cmd)
        return 1

    @staticmethod
    def test_script(script_code=""):
        cmd = "{"
        cmd += "\"action\":\"test_script\","
        cmd += "\"script_id\":0,"
        cmd += "\"script_code\":\"" + script_code + "\""
        cmd += "}"
        myp.send(cmd)
        return 1
    
    #####################################
    # script commands                   #
    #####################################
    @staticmethod
    def move_joint(*arguments):
        return RobotHandler.test_script("move_joint" + str(arguments))

    @staticmethod
    def move_tool(*arguments):
        return RobotHandler.test_script("move_tool" + str(arguments))

    @staticmethod
    def move_to_pose(*arguments):
        return RobotHandler.test_script("move_to_pose" + str(arguments))

    @staticmethod
    def open_gripper(*arguments):
        return RobotHandler.test_script("open_gripper" + str(arguments))

    @staticmethod
    def close_gripper(*arguments):
        return RobotHandler.test_script("close_gripper" + str(arguments))

    @staticmethod
    def move_gripper(*arguments):
        return RobotHandler.test_script("move_gripper" + str(arguments))

    @staticmethod
    def play_path(*arguments):
        return RobotHandler.test_script("play_path" + str(arguments))

    #####################################
    # database functions                #
    #####################################

    @staticmethod
    def get_script(script_name):
        for script in myp.get_status("scripts"):
            if script["name"] == script_name:
                return script["code"]
        return 0

    @staticmethod
    def get_scripts():
        scripts = myp.get_status("scripts")
        return scripts

    @staticmethod
    def save_script(script_name="", script_code=""):
        # edit script
        cmd = "{"
        cmd += "\"action\":\"edit_script\","
        cmd += "\"editor_id\":\"edit_script_0_0\","
        cmd += "\"script_id\":0"
        cmd += "}"
        print(cmd)
        myp.send(cmd)
        # update script
        cmd = "{"
        cmd += "\"action\":\"update_script\","
        cmd += "\"script_name\":\"" + script_name + "\","
        cmd += "\"editor_id\":\"edit_script_0_0\""
        cmd += "}"
        print(cmd)
        myp.send(cmd)
        # save the script
        cmd = "{"
        cmd += "\"action\":\"save_script\","
        cmd += "\"editor_id\":\"edit_script_0_0\","
        cmd += "\"script_code\":\"" + script_code + "\""
        cmd += "}"
        print(cmd)
        myp.send(cmd)
        return 1

    @staticmethod
    def delete_script(script_name):
        script_id = RobotHandler._get_id_from_name(script_name, "scripts")
        cmd = "{"
        cmd += "\"action\":\"delete_script\","
        cmd += "\"script_id\":" + str(script_id)
        cmd += "}"
        print(cmd)
        myp.send(cmd)
        return 1

    @staticmethod
    def get_pose(pose_name):
        poses = myp.get_status("poses")
        for pose in poses:
            if pose["name"] == pose_name:
                return pose
        return 0

    @staticmethod
    def get_poses():
        poses = myp.get_status("poses")
        return poses

    @staticmethod
    def get_path(path_name):
        paths = myp.get_status("paths")
        for path in paths:
            if path["name"] == path_name:
                return path
        return 0

    @staticmethod
    def get_task(task_name):
        tasks = myp.get_status("tasks")
        for task in tasks:
            if task["name"] == task_name:
                return task
        return 0

    @staticmethod
    def get_paths():
        paths = myp.get_status("paths")
        return paths

    @staticmethod
    def wait_for_robot():
        while True:
            status = RobotHandler.get_status_info()
            connection_status = RobotHandler.get_connection_info()
            if status != 4 and connection_status >= 2:
                break
            else:
                time.sleep(0.1)

    @staticmethod
    def _get_id_from_name(name, table_name):
        elements = myp.get_status(table_name)
        for element in elements:
            if element["name"] == name:
                return element["id"]
        return 0


    def publisher(self):
        pub1 = rospy.Publisher('connection', Status, queue_size=10)
        pub2 = rospy.Publisher('status', Status, queue_size=10) 
        pub3 = rospy.Publisher('position', Array, queue_size=10)
        pub4 = rospy.Publisher('current', Array, queue_size=10)
        pub5 = rospy.Publisher('actuator_angles', Array, queue_size=10)
        rate = rospy.Rate(10) #hz
        print("Publisher started to publish Status Infos..")
        while not rospy.is_shutdown():
            connection = self.get_connection_info()
            status = self.get_status_info()
            position = self.get_position()
            current = self.get_current()
            actuator_angles = self.get_actuator_angles()
            pub1.publish(connection)
            pub2.publish(status)
            if position != 0:
                pub3.publish(position)
            if current != 0:
                pub4.publish(current)
            if actuator_angles != 0:
                keys = self.get_kinematic_indices()
                actuator_angles = [actuator_angles[str(key)] for key in keys]
                pub5.publish(actuator_angles)
            rate.sleep()


# initialize server
def start_server():
  global robot_arm

  try:
    rospy.init_node('MyP')

    # ROS service
    srv_move_joint = rospy.Service('move_joint', MoveJoint, handle_move_joint)
    srv_move_tool = rospy.Service('move_tool', MoveTool, handle_move_tool)
    srv_move_to_pose = rospy.Service('move_to_pose', MoveToPose, handle_move_to_pose)
    srv_wait_for_robot = rospy.Service('wait_for_robot', Empty, handle_wait_for_robot)
    srv_finalize = rospy.Service('finalize', Empty, handle_finalize)
    srv_move_wait = rospy.Service('wait', Wait, handle_wait)
    srv_initialize = rospy.Service('initialize', Initialize, handle_initialize)
    srv_calibrate = rospy.Service('calibrate', Calibrate, handle_calibrate)
    srv_test_script = rospy.Service('test_script', TestScript, handle_test_script)
    srv_execute_script = rospy.Service('execute_script', ExecuteScript, handle_execute_script)
    srv_release = rospy.Service('release', Release, handle_release)
    srv_hold = rospy.Service('hold', Release, handle_hold)
    srv_open_gripper = rospy.Service('open_gripper', OpenGripper, handle_open_gripper)
    srv_close_gripper = rospy.Service('close_gripper', CloseGripper, handle_close_gripper)
    srv_get_connection_info = rospy.Service('get_connection_info', GetInfo, handle_get_connection_info)
    srv_get_status_info = rospy.Service('get_status_info', GetInfo, handle_get_status_info)
    srv_get_application_info = rospy.Service('get_application_info', GetInfo, handle_get_application_info)
    srv_get_print_info = rospy.Service('get_print_info', GetInfoString, handle_get_print_info)
    srv_get_message_info = rospy.Service('get_message_info', GetInfoString, handle_get_message_info)
    srv_get_kinematic_indices = rospy.Service('get_kinematic_indices', GetStringArray, handle_get_kinematic_indices)
    srv_get_actuator_indices = rospy.Service('get_actuator_indices', GetArray, handle_get_actuator_indices)
    srv_get_actuator_release_state = rospy.Service('get_actuator_release_state', GetBoolArray, handle_get_actuator_release_state)
    srv_get_all_status = rospy.Service('get_all_status', GetInfoString, handle_get_all_status)
  
    #srv_get_gripper_angle = rospy.Service('get_gripper_angle', Empty, handle_get_gripper_angle)

    # Connection to myP XMLRPC server
    robot_arm = RobotHandler()

    # initialize and calibrate robot
    #robot_arm.initialize("PRob1Uplus", "real")
    #robot_arm.wait_for_robot()
    #robot_arm.calibrate(True)
    #robot_arm.wait_for_robot()
    print("Connected to PRob.")
    print("Robot Status:")
    print("_______________________________________________________")
    print("For Connection Status:               | For Status Info:")
    print("0: not Initialized, not Calibrated   | None")
    print("1: Initializing, not Calibrated      | Ready")
    print("2: Initialized, not Calibrated       | Stopped")
    print("3: Initialized, Calibrating          | Paused")
    print("4: Initialized, Calibrated           | Running")
    print("5:                                   | Released")
    print("6:                                   | Error")
    print "Connection: ",robot_arm.get_connection_info()
    print "Status: ", robot_arm.get_status_info()
    start_new_thread(robot_arm.publisher,())
    rospy.spin()

  except Exception as ex:
    print("ERROR: ", ex)
    sys.exit(1)

def usage():
    return "%s [PRob ip address]"%sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 2:
    ip = sys.argv[1] 
  else:
    print( usage() )
    sys.exit(1)
  try:
    myp = SockJSClient('/socket', ip, 8888)
    myp.connect()
    start_server()

  except:
    print("Could not connect to myP at: ", ip)
    # traceback.print_exc()
    # raise

  finally:
    myp.disconnect()
    print("Server closed")
