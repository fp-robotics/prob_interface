#!/usr/bin/env python
from prob_msgs.msg import *
from visualization_msgs.msg import *
import rospy
import sys
import tf
from math import pi

def callback(data):
        br = tf.TransformBroadcaster()
        br.sendTransform((data.position[0], data.position[1], data.position[2]),
                        tf.transformations.quaternion_from_euler(data.position[3], data.position[4], data.position[5]),
                        rospy.Time.now(),
                        "TCP",
                        "world")
        pub = rospy.Publisher('visualization_marker', Marker, queue_size=10) 
	marker = Marker()
	marker.header.frame_id= "world"
	marker.header.stamp = rospy.get_rostime()
	marker.ns = "my_namespace"
	marker.id = 1
	marker.type = 0
	marker.action = 0
	marker.scale.x = 0.15
	marker.scale.y = 0.02
	marker.scale.z = 0.04
	marker.pose.position.x = data.position[0]/1000
	marker.pose.position.y = data.position[1]/1000
	marker.pose.position.z = data.position[2]/1000
	quaternion = tf.transformations.quaternion_from_euler((data.position[3]+180)*pi/180, (data.position[4]+180)*pi/180, (data.position[5]+180)*pi/180)
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
	rospy.Subscriber("position", Position, callback)
	rospy.spin()

if __name__ == "__main__":
	listener()
