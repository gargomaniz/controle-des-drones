#!/usr/bin/env python 
#from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time


class MoveDrone:

 	def __init__(self):
 		self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100) 
		#TODO put the takeoff topic name here

 		self.landing_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=100) 
		#TODO put the landing topic name here

		self.front_image_pub = rospy.Publisher("/ardrone/front/image_raw", Empty, queue_size=100)

	def takeoff_drone(self):
 		empty_msg = Empty()
 		self.takeoff_pub.publish(empty_msg)

 	def land_drone(self):
 		empty_msg = Empty()
 		self.landing_pub.publish(empty_msg)
	def camera_drone(self):
 		empty_msg = Empty()
		self.front_image_pub.publish(empty_msg)		


if __name__ == '__main__':

	rospy.init_node('basic_controller', anonymous=True)

	move = MoveDrone()

	# TODO define your time counter here !

	t1 = time.time()	
	while(time.time()-t1 < 3):
		move.takeoff_drone()

	t1 = time.time()
	move.camera_drone()	

	t1 = time.time()
	while(time.time()-t1 < 4):
		move.land_drone()
