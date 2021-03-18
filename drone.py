#!/usr/bin/env python
#from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from multiprocessing import Process
from multiprocessing import Value
global dir
class MoveDrone:
  def __init__(self):
    self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100)
    #TODO put the takeoff topic name here
    self.landing_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=100)
    #TODO put the landing topic name here
    self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    # Publish commands to drone
  def move_drone(self, speed=[0.0, 0.0, 0.0], orient=[0.0, 0.0, 0.0]):
    vel_msg = Twist()	
	
    # TODO: fill the velocity fields here with the desired values
    vel_msg.linear.x = speed[0]
    vel_msg.linear.y = speed[1]
    vel_msg.linear.z = speed[2]
    #TODO: fill the angulare velocities here with the desired values
    vel_msg.angular.x = orient[0]
    vel_msg.angular.y = orient[1]
    vel_msg.angular.z = orient[2]
    self.move_pub.publish(vel_msg)
    return 0 	
	
  def takeoff_drone(self):
    empty_msg = Empty()
    self.takeoff_pub.publish(empty_msg)
  def land_drone(self):
    empty_msg = Empty()
    self.landing_pub.publish(empty_msg)
class image_converter():
  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    self.dir = dir
  def callback(self, data):
        try:
            print("image")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.first_callback = False
        except CvBridgeError as e:
            print(e)
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)
        original = cv_image.copy()
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = np.array([20, 100, 100], dtype="uint8")
        upper = np.array([30, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        max_area = cv2.contourArea(cnts[0])
        biggest_cnt = cnts[0]
        for c in cnts:
            area = cv2.contourArea(c)
	    if area > max_area :
	        max_area = area
                biggest_cnt = c
        x,y,w,h = cv2.boundingRect(biggest_cnt)
        cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
        cx = int(x+(w/2))
        cy = int(y+(h/2))
        if (cx <int(cols/2)-100):
            cv2.putText(original,"GO LEFT" , (20,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
            dir = 1
        elif (cx >int(cols/2)+100):
            cv2.putText(original,"GO RIGHT" , (20,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
            dir = 2
        elif (cy <int(rows/2)-100):
            cv2.putText(original,"GO UP" , (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
            dir = 3
        elif (cy >int(rows/2)+100):
            cv2.putText(original,"GO DOWN" , (50,50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
            dir = 4
	else :
	    dir = 0
        cv2.imshow('original', original)
        cv2.waitKey(3)
def move():
  print("move")
  speed = [0,0,0]
  orient = [0,0,0]
  md = MoveDrone()
  rospy.init_node('move_drone', anonymous=True)
  t1 = time.time()
  while(time.time()-t1 < 4):
    md.takeoff_drone()
  t_deb = time.time()
  while(time.time() - t_deb < 60):
    if dir == 1:
        orient = [0,0,0.5]
    elif dir == 2:
        orient = [0,0,-0.5]
    elif dir == 3:
        speed = [0,0,0.5]
    elif dir == 4:
        speed = [0,0,-0.5]
    t = time.time()
    while(time.time()-t< 1):
      move.move_drone(speed, orient)
      pass
      t = time.time()
  t1 = time.time()
  while(time.time()-t1 < 4):
    md.land_drone()
def cam():
  #recuperer le flux video
  print("cam")
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
def main():
  print("go")
  threadMove = Process(target=move)
  threadCam = Process(target=cam)
  threadMove.start()
  threadCam.start()
  time.sleep(5)
  threadMove.join()
  threadCam.join()
if __name__ == '__main__':
      main()