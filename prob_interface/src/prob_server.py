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
  print("move_tool(%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"%( req.x, req.y, req.z, req.phi, req.theta, req.psi, req.velocity, req.acceleration, req.velocity_rot, req.acceleration_rot, bool(req.block), bool(req.relative)))
  robot_arm.move_tool( req.x, req.y, req.z, req.phi, req.theta, req.psi, req.velocity, req.acceleration, req.velocity_rot, req.acceleration_rot, bool(req.block), bool(req.relative))
  return 1
  
def handle_move_to_pose(req):
  print("move_to_pose(%s, %s, %s, %s, %s)"%(req.name, req.vel, req.acc, req.block))
  robot_arm.move_to_pose( req.name, req.vel, req.acc, req.block)
  return 1
  
def handle_wait_for_robot(req):
  print("wait_joint(%s)"%(bool(req.show)))
  robot_arm.wait_for_robot( bool(req.show))
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
  return status

def handle_get_connection_info(req):
  print("get_connection_info()")
  connection_info = robot_arm.get_connection_info()
  return connection_info

def handle_get_all_status(req):
  print("get_all_status()")
  status = robot_arm.get_all_status()
  status = str(status)
  return GetInfoStringResponse(status)

#def handle_get_gripper_angle(req):
#  print("get_gripper_angle()")
#  robot_arm.get_gripper_angle()

class RobotHandler:

    # TODO: STATUS_FUNCTIONS are not working correctly yet!

    # 0: 'not Initialized, not Calibrated'
    # 1: 'Initializing, not Calibrated'
    # 2: 'Initialized, not Calibrated'
    # 3: 'Initialized, Calibrating'
    # 4: 'Initialized, Calibrated'
    def get_connection_info(self):
        connection = myp.get_status("connection_info")
        #print("Connection Info: ", connection)
        return connection
    
    # 0: 'None'
    # 1: 'Ready'
    # 2: 'Stopped'
    # 3: 'Paused'
    # 4: 'Running'
    # 5: 'Released'
    # 6: 'Error'
    def get_status_info(self):
        status = myp.get_status("status_info")
        #print("Status Info: ", status)
        return status
      
    def get_message_info(self):
        message = myp.get_status("message_info")
        #print("Message Info: ", message)
        return message
        
    def get_application_info(self):
        message = myp.get_status("application_info")
        #print("Application Info: ", message)
        return message
      
    def get_actuator_release_state(self):
        state = myp.get_status("actuator_release_state")
        #print("Actuator Release State: ", state)
        return state
        
    def get_position(self):
        position = myp.get_status("current_pose")
        #print("Position: ", position)
        return position
        
    def get_euler_position(self):
        position = myp.get_status("euler_position")
        #print("Euler Position: ", position)
        return position
        
    def get_current(self):
        current = myp.get_status("current")
        #print("Current: ", current)
        return current

    #####################################
    # initialization functions          #
    #####################################
    def initialize(self, model="PRob1R", kind="real", channel_name="1",
                   channel_type="PEAK_SYS_PCAN_USB", protocol="TMLCAN", host_id="10", baudrate="500000"):
        if self.get_connection_info() == 0:
            cmd = "\"{"
            cmd += "\\\"action\\\":\\\"initialize\\\","
            cmd += "\\\"model\\\":\\\""+model+"\\\","
            cmd += "\\\"robot_kind\\\":\\\""+kind+"\\\""
            if kind == 'real':
                cmd += ","
                cmd += "\\\"channel_name\\\":\\\""+channel_name+"\\\","
                cmd += "\\\"channel_type\\\":\\\""+channel_type+"\\\","
                cmd += "\\\"protocol\\\":\\\""+protocol+"\\\","
                cmd += "\\\"host_id\\\":"+host_id+","
                cmd += "\\\"baudrate\\\":"+baudrate
            cmd += "}\""
            #print(cmd)
            myp.send(cmd)
            return 1
        return 0
        
    def finalize(self):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"finalize\\\""
        cmd += "}\""
        myp.send(cmd)
        return 1
    
    #####################################
    # status functions                  #
    #####################################
    def pause(self):
        cmd = "\"{"
        cmd += "\\\"action\\\":\\\"pause\\\""
        cmd += "}\""
        myp.send(cmd)
        return 1

    def resume(self):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"resume\\\""
        cmd += "}\""
        myp.send(cmd) 
        return 1
        
    def stop(self):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"stop\\\""
        cmd += "}\""
        myp.send(cmd) 
        return 1
        
    def recover(self):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"recover\\\""
        cmd += "}\""
        myp.send(cmd)
        return 1
        
    #####################################
    # control functions                 #
    #####################################
    def calibrate(self, use_existing=True):
        if self.get_connection_info() == 2:
            cmd = "\"{"
            cmd += "\\\"action\\\": \\\"calibrate\\\","
            cmd += "\\\"use_existing\\\": "+str(use_existing).lower()
            cmd += "}\""
            myp.send(cmd)
            return 1
        return 0
        
    def release(self, joint_id=[1]):
        # TODO: make release all joints to default
        print(joint_id)
        length = len(joint_id)
        processed_string = '['
        for i in range(0, length-1):
            processed_string += "\\\"" + str(joint_id[i]) + "\\\","
        processed_string += "\\\"" + str(joint_id[length-1]) + "\\\"]"

        cmd = "\"{"
        cmd += "\\\"action\\\": \\\"release\\\","
        cmd += "\\\"joint_ids\\\": "+processed_string
        cmd += "}\""
        myp.send(cmd)
        return 1
        
    def hold(self, joint_id=[1]):
        # TODO: make hold all joints to default
        length = len(joint_id)
        processed_string = '['
        for i in range(0, length-1):
            processed_string += "\\\"" + str(joint_id[i]) + "\\\","
        processed_string += "\\\"" + str(joint_id[length-1]) + "\\\"]"

        cmd = "\"{"
        cmd += "\\\"action\\\": \\\"hold\\\","
        cmd += "\\\"joint_ids\\\": "+processed_string
        cmd += "}\""
        myp.send(cmd)
        return 1

    def execute_script(self, script_id=1):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"execute_script\\\","
        cmd += "\\\"script_id\\\":"+str(script_id)
        cmd += "}\""
        myp.send(cmd)
        return 1
        
    def test_script(self, script_code=""):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"test_script\\\","
        cmd += "\\\"script_id\\\":0,"
        cmd += "\\\"script_code\\\":\\\""+script_code+"\\\""
        cmd += "}\""
        #print(cmd)
        myp.send(cmd)
        #self.wait_for_robot()
        return 1

    # def move_to_pose(self, pose_id=1, velocity=15):
    #    cmd =  "\"{"
    #    cmd += "\\\"action\\\":\\\"move_to_pose\\\","
    #    cmd += "\\\"pose_id\\\":"+str(pose_id)+","
    #    cmd += "\\\"velocity\\\":"+str(velocity)
    #    cmd += "}\""
    #    myp.send(cmd)
    #    return 1
    # TODO: tcp_move
    # def tcp_move(self):
    # data = {
    #         action: 'tcp_move',
    #         x: parseFloat(x),
    #         y: parseFloat(y),
    #         z: parseFloat(z),
    #         orientation: orientation,
    #         only_position: $('#only_position').prop('checked'),
    #         linear: $('#tcp_linear').prop('checked'),
    #         velocity: parseFloat(v)
    #     };
    #    return 0
    #def play_path(self):
    #    return 0    
    #def pick_and_place(self):
    #    return 0
    #def record_sample(self):
    #    return 0                
    
    #####################################
    # script commands                   #
    #####################################
    def move_joint(self, *arguments):
        #print("move_joint"+str(arguments))
        return self.test_script("move_joint"+str(arguments))
    def move_joint_monitored(self, *arguments):
        return self.test_script("move_joint"+str(arguments))
    def move_tool(self, *arguments):
        return self.test_script("move_tool"+str(arguments))
    def move_linear(self, *arguments):
        return self.test_script("move_linear"+str(arguments))
    def move_to_pose(self, *arguments):
        return self.test_script("move_to_pose"+str(arguments))
    def open_gripper(self, *arguments):
        return self.test_script("open_gripper"+str(arguments))
    def close_gripper(self, *arguments):
        return self.test_script("close_gripper"+str(arguments))
    def path_move(self, *arguments):
        return self.test_script("path_move"+str(arguments))
    def play_path(self, *arguments):
        return self.test_script("play_path"+str(arguments))
    def reactive_spline(self, *arguments):
        return self.test_script("reactive_spline"+str(arguments))


    #####################################
    # database functions                #
    #####################################
    def request_script(self, script_id=1):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"get_script\\\","
        cmd += "\\\"script_id\\\":\\\""+str(script_id)+"\\\""
        cmd += "}\""
        myp.send(cmd)
        return 1
        
    def get_script(self, script_id=1):
        myp.remove_status("one_script")
        self.request_script(script_id)
        script = myp.get_status("one_script")
        counter = 0
        while (script == 0):
            time.sleep(0.001) 
            script = myp.get_status("one_script")
            counter += 1
            if (counter > 100):
                print("script == 0!")
                return 0
        return script

    def get_scripts(self):
        scripts = myp.get_status("scripts")
        print("Scripts: ", scripts)
        return scripts
          
    def get_script_id(self, name):
        scripts = myp.get_status("scripts")
        for script in scripts:
            if script[1] == name:
              return script[0]
        return 0
        
    def save_script(self, script_name="", script_code=""):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"save_script\\\","
        cmd += "\\\"script_name\\\":\\\""+script_name+"\\\","
        cmd += "\\\"script_code\\\":\\\""+script_code+"\\\""
        cmd += "}\""
        myp.send(cmd)
        return 1
        
    def delete_script(self, script_id=1):
        cmd =  "\"{"
        cmd += "\\\"action\\\":\\\"delete_script\\\","
        cmd += "\\\"script_id\\\":\\\""+str(script_id)+"\\\""
        cmd += "}\""
        myp.send(cmd)
        return 1
        
    def get_poses(self):
        poses = myp.get_status("poses")
        print("Poses: ", poses)
        return poses
        
    def get_paths(self):
        paths = myp.get_status("paths")
        print("Paths: ", paths)
        return paths
        
    #TODO: needs to be implemented...
    def record_path(self):
        return 0  
    #TODO: needs to be implemented...    
    def stop_record_path(self):
        return 0  
    #TODO: needs to be implemented...    
    def delete_path(self):
        return 0 
    #TODO: needs to be implemented...    
    def save_pose(self):
        return 0 
    #TODO: needs to be implemented...    
    def delete_pose(self):
        return 0

    def wait_for_robot(self, show=False):
        counter = 0
        while self.get_status_info() != 1:
            time.sleep(0.1)
            if counter % 20 == 0 and show:
                print("Robot not ready yet!")
            counter += 1

    #def get_lesson(self):
    #    return 0   
    #def save_lesson(self):
    #    return 0   
    #def delete_lesson(self):
    #    return 0     
    #def discard_samples(self):
    #    return 0  
    #def save_samples_and_train(self):
    #    return 0     
    #def get_item(self):
    #    return 0     
    #def save_item(self):
    #    return 0   
    #def delete_item(self):
    #    return 0          
    #def answer_dialog(self):
    #    return 0           
    

    def publisher(self):
        pub1 = rospy.Publisher('connection', Status, queue_size=10)
        pub2 = rospy.Publisher('status', Status, queue_size=10) 
        pub3 = rospy.Publisher('position', Position, queue_size=10) 
        rate = rospy.Rate(10) #hz
        print("Publisher started to publish Status Infos..")
        while not rospy.is_shutdown():
            connection = self.get_connection_info()
            status = self.get_status_info()
            position = self.get_position()
            pub1.publish(connection)
            pub2.publish(status)
            if position != 0:
                pub3.publish(position)
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
    srv_move_wait_for_robot = rospy.Service('wait_for_robot', WaitForRobot, handle_wait_for_robot)
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
