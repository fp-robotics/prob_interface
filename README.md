# prob_interface
A repository for establishing an interface to the P-Rob in ROS.

## Setup
Clone the 'prob_interface' repository into your catkin workspace 'src' folder and run `catkin_make`.
Make sure the PRob is connected to the same network as our machine and run a ROS master.
In a second terminal, source first the your catkin workspace (ex. `~/catkin_ws/devel/setup.bash`) and run
```
rosrun prob_interface prob_server.py <PROB_IP_ADDRESSS>
```
Replace `<PROB_IP_ADDRESSS>`with the IP of the robot. Now the robot is ready to be controlled through ROS service calls.

To initialize and calibrate the robot, you may run in a third terminal
```
rosrun prob_interface prob_initialize.py <CALIBRATE_FLAG>
```
If the robot should be calibrated, set `CALIBRATE_FLAG` to `0`, otherwise to `-1`. After calibrating the robot, you may have a look into the demo script of the repository or execute it:
```
rosrun prob_interface client_function_demo.py
```

## Packages
**prob_interface** - base class/package contains the prob_server that builds the connection to the P-Rob and handles all the traffic between ROS and the P-Rob

**prob_msgs** - collection of messages for interacting with the prob.

## Dependencies
Please install `ros-indigo-desktop-full' for using this package

## Nodes
### prob_server
Builds the connection to the P-Rob over a socket and contains multiple services for sending commands to the P-Rrob

#### Parameters
* `ip` (string, default: "127.0.0.1") - The ip of the host that runs myP

#### Subscribed Topics
None at the moment

#### Published Topics
* `connection` ([prob_msgs/Status]) - Connection Info of the P-Rob
* `status` ([prob_msgs/Status]) - Status Info of the P-Rob
* `position`([prob_msgs/Array]) - Position of the P-Rob (X|Y|Z|Roll|Pich|Yaw)
* `current`([prob_msgs/Array]) - Currents in the joints of the P-Rob (1|2|3|4|5|6|7)
* `actuator_angles`([prob_msgs/Array]) - Angles of the joints of the P-Rob (1|2|3|4|5|6|7)

#### Services
* `move_joint` ([prob_msgs/MoveJoint]) - move joint.
* `move_tool` ([prob_msgs/MoveTool]) - move tool.
* `move_to_pose` ([prob_msgs/MoveToPose]) - move to pose.
* `wait_for_robot` ([prob_msgs/WaitForRobot]) - wait until the robot's status is 1 for Ready again
* `wait` ([prob_msgs/Wait]) - Tell the robot to wait some time
* `initialize` ([prob_msgs/Initialize]) - Initialize the P-Rob.
* `calibrate` ([prob_msgs/Calibrate]) - Calibrate the P-Rob.
* `test_script` ([prob_msgs/TestScript]) - Send the P-Rob a Script to Execute.
* `execute_script` ([prob_msgs/ExecuteScript]) - Tell the P-Rob to execute a Script that is already on the P-Rob's database
* `release` ([prob_msgs/Release]) - Release one or more joints of the P-Rob.
* `hold` ([prob_msgs/Release]) - Hold one or more joints of the P-Rob.
* `open_gripper` ([prob_msgs/OpenGripper]) - Open the P-Rob's Gripper.
* `close_gripper` ([prob_msgs/CloseGripper]) - Close the P-Rob's Gripper.
* `get_connection_info` ([prob_msgs/GetInfo]) - Get the Connection Status of the P-Rob.
* `get_status_info` ([prob_msgs/GetInfo]) - Get the Status Info of the P-Rob.
* `get_all_status` ([prob_msgs/GetInfoString]) - Get all the Status infos.
* `get_kinematic_indices` ([prob_msgs/GetInfoString]) - Get all the Status infos.
* `get_actuator_release_state` ([prob_msgs/GetInfoString]) - Get all the Status infos.
* `get_print_info` ([prob_msgs/GetInfoString]) - Get all the Status infos.
* `get_application_info` ([prob_msgs/GetInfoString]) - Get all the Status infos.

#### Run
```
rosrun prob_interface prob_server.py myp_host_ip
```

### execute_script
Runs a whole script file containing valid MyP script commands (See MyP Scriptcommand Refrence Manual)
on the P-Rob

#### Parameters
* `filename` (string) - The path and filename of the script. Ex: ~/scripts/script.script

#### Run
```
rosrun prob_interface execute_script.py ~/scripts/test_script.script
```

### client_functions_demo
Is a demo node for showing how one could work with the robot in ROS.
It will initialize and calibrate the robot, move joint 5 to 90 degrees and the open and close the gripper.

#### Run
```
rosrun prob_interface client_functions_demo.py
```

### prob_status
This node just prints the status of the robot every 2 second.

#### Run
```
rosrun prob_interface prob_status.py
```

### prob_initialize
A Node for initializing and calibrating the robot

#### Parameters
* `model` (String) - Model of the Robot that's connected. ex: 'PRob2R', 'PRob1R', 'PRob1Uplus', 'PRob1U'
* `script_id` (int) - -1: Calibrate with existing values, 0: calibrate the robot -> movement
* `channel_name` (String) - only needed if connected to a robot with usb, channel name is the device path of the usb. ex: '/dev/pcanusb2'

#### Run
```
rosrun prob_interface prob_initialize.py PRob2R True /dev/pcanusb2
```

### move_joint
Is a demo node which takes the same arguments as move_joint and executes this command over the move_joint service

#### Parameters
* `joint_id` (Int Array) - Ids of one or more joints that want to be moved
* `Degree` (Array) - Array of Degrees each joint should be moved
* `Velocity` (Int, default = 15) - Velocity
* `Acceleration` (Int, default = 60) - Acceleration
* `Blocking` (Bool, default = True) - Should the movement be blocking or not
* `Relative` (Bool, default = False) - Should the movement be relative to the current angle or not

#### Run
```
rosrun prob_interface move_joint.py 1 50 50 50
```

### prob_plot
Publishes an arrow marker, which can be plotted in rviz. This arrow shows the position and orientation of the tool center point

#### Published Topics
* `connection` ([visualization_msgs/Marker]) - Arrow Marker for rviz

#### Run
Terminal 1:
```
rviz
```

Terminal 2:
```
rosrun prob_interface prob_plot.py
```

[std_srvs/Empty]: http://docs.ros.org/indigo/api/std_srvs/html/srv/Empty.html
[prob_msgs/MoveJoint]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/MoveJoint.srv
[prob_msgs/MoveTool]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/MoveTool.srv
[prob_msgs/MoveToPose]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/MoveToPose.srv
[prob_msgs/WaitForRobot]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/WaitForRobot.srv
[prob_msgs/Wait]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/Wait.srv
[prob_msgs/Initialize]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/Initialize.srv
[prob_msgs/Calibrate]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/Calibrate.srv
[prob_msgs/TestScript]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/TestScript.srv
[prob_msgs/ExecuteScript]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/ExecuteScript.srv
[prob_msgs/Release]:https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/Release.srv
[prob_msgs/OpenGripper]:https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/OpenGripper.srv
[prob_msgs/CloseGripper]:https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/CloseGripper.srv
[prob_msgs/GetInfo]:https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/GetInfo.srv
[prob_msgs/GetInfoString]:https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/srv/GetInfoString.srv
[prob_msgs/Status]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/msg/Status.msg
[prob_msgs/Position]: https://github.com/fp-robotics/prob_interface/blob/master/prob_msgs/msg/Position.msg
[visualization_msgs/Marker]: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html

