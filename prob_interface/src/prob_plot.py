#!/usr/bin/env python
from prob_msgs.msg import *
from visualization_msgs.msg import *
import rospy
import sys
import tf

def callback(data):
	marker = Marker()
	marker.header.frame_id= "/my_frame"
	marker.header.stamp = rospy.get_rostime()
	marker.ns = "my_namespace"
	marker.id = 0
	marker.type = 0
	marker.action = 0
	marker.scale.x = 0.1
	marker.scale.y = 0.01
	marker.scale.z = 0.02
	marker.pose.position.x = data.position[0]
	marker.pose.position.y = data.position[1]
	marker.pose.position.z = data.position[2]
	quaternion = tf.transformations.quaternion_from_euler(data.position[3], data.position[4], data.position[5])
	#type(pose) = geometry_msgs.msg.Pose
	marker.pose.orientation.x = quaternion[0]
	marker.pose.orientation.y = quaternion[1]
	marker.pose.orientation.z = quaternion[2]
	marker.pose.orientation.w = quaternion[3]
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	pub.publish(marker)

def listener():

	rospy.init_node('listener', anonymous=True)

	pub = rospy.Publisher('visualization_marker', Marker, queue_size=10) 
	rospy.Subscriber("position", Position, callback)
	rospy.spin()

if __name__ == "__main__":
	listener()