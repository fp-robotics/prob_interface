# prob_interface
A repository for establishing an interface to the P-Rob in ROS.

## Packages
**prob_interface** - base class/package contains the prob_server that builds the connection to the P-Rob and handles all the traffic between ROS and the P-Rob
**prob_msgs** - collection of messages for interacting with the prob.

## Dependencies
Please install `ros-indigo-desktop-full' for using this package

## Nodes
### prob_server
Builds the connection to the P-Rob and contains multiple services for sending commands to the P-Rrob

#### Parameters
* `ip` (string, default: "127.0.0.1") - The ip of the host that runs the xml_rpc server

#### Subscribed Topics
None at the moment

#### Published Topics
None at the moment

#### Services
* `move_joint` ([std_srvs/Empty]) - move_joint.

[std_srvs/Empty]: http://docs.ros.org/indigo/api/std_srvs/html/srv/Empty.html
