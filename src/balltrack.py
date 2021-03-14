#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image

'''
Function for detecting the ball 
and returning its coordinate
'''
def detectBall(frame):
	global counter, X, Y
	counter += 1

	
	colorLower = (  0, 75,145)
	colorUpper = ( 50,255,255)
	i1, i2 = 3, 5

	hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=i1)
	mask = cv2.dilate(mask, None, iterations=i2)
	
	(_,cnts, _) = cv2.findContours(mask.copy(), \
		cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	if counter%4 == 0:
		X, Y = [], []

	for (i,c) in enumerate(cnts):
		area = cv2.contourArea(c)
		perimeter = cv2.arcLength(c, True)
		if area > 1000 and perimeter > 100:
			print ("Contour #%d -- area: %.2f, perimeter: %.2f" \
				% (i + 1, area, perimeter))			
			c = max(cnts, key=cv2.contourArea)
			M = cv2.moments(c)
			(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			X.append(cX)
			Y.append(cY)

	if X and Y:
		cX = int(sum(X)/len(X))
		cY = int(sum(Y)/len(Y))
		return cX, cY, mask
	else:
		return -100, -100, mask

def image_callback(data):
	global cX, cY, pub

	image = bridge.imgmsg_to_cv2(data, "bgr8")
	h , w = image.shape[:2]

	cX, cY, mask = detectBall(image)

	point = Point()
	point.x = cX
	point.y = cY
	point.z = w 
	pub_point.publish(point)

	length = int(w/100)
	(startX, endX) = (int(cX - length), int(cX + length))
	(startY, endY) = (int(cY - length), int(cY + length))
	cv2.line(image, (startX, cY), (endX, cY), (0, 0, 255), 2)
	cv2.line(image, (cX, startY), (cX, endY), (0, 0, 255), 2)


if __name__ == '__main__':
	global counter, X, Y, cX, cY, pub
	counter = 0
	X, Y = [], []

	rospy.init_node('find_ball', anonymous=True)

	img_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,image_callback)
	bridge = CvBridge()
	
	pub_point = rospy.Publisher('/ball_location', Point, queue_size=5)
	rospy.spin()
