#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time

def drive_callback(data):

	global vel
	ball_x 	= data.x
	ball_y 	= data.y
	width  	= data.z
	
	vel = Twist()
	
	if ball_x < 0 and ball_y < 0:
		vel.angular.z = 0
	else:
		mid_x  	= int(width/2)
		delta_x	= ball_x - mid_x
		norm_x 	= delta_x/width

		if norm_x > 0.15:
			print ("Turn right".format(norm_x))
			vel.angular.z = -0.5
		elif norm_x < -0.15:
			print ("Turn left".format(norm_x))
			vel.angular.z = 0.5
		if abs(norm_x) < 0.15:
			print ("Stay in center".format(norm_x))			
			vel.angular.z = 0.5
			vel.linear.x = 0.2
	pub_vel.publish(vel)

if __name__ == '__main__':
	global vel, pub_vel

	rospy.init_node('drive_wheel', anonymous=True)

	img_sub = rospy.Subscriber("/ball_location",Point, drive_callback)

	pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
	rospy.spin()
