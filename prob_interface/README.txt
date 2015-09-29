ROS package containing an interface to the PRob by F&P Robotics

Following ROS services are supported:
	- MoveJoint:	moves an axis to a given position
	- ...

To talk to the ROS server you have to write your own ROS client. 
A sample code is in testprogram/client_demo.py

=============================================================
Installing ROS:

	1) Install ROS Indigo: http://wiki.ros.org/ROS/Installation
	2) Create a catkin workspace
	3) Copy this package ‚prob_ros_interafce‘ to the workspace
	4) Compile the code and source your setup
		$ catkin_make
		$ source devel/setup.bash
	5) run roscore
	6) Now your ready to run the testprogram


=============================================================
Example testprogram/:

Requirements:	
	- ROS Indigo has to be installed and roscore running
	- prob_ros_interface is installed
	- XMLRPC Server needs to be running on the PRob (xmlrpc_server.py)

start the server with the ip address of the robot:
	$ rosrun prob_interface  prob_server.py 192.168.21.157

Run the sample demo and move axis 1 to 10 deg and back:
	$ rosrun prob_interface client_demo.py 1 10
	$ rosrun prob_interface client_demo.py 1 0


