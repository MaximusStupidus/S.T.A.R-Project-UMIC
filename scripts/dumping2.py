#!/usr/bin/env python
from __future__ import print_function
import time
import roslib
roslib.load_manifest('umic')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
import ros_numpy
import math



#defining x,y of green ball for global use
x = 0
y = 0
rad=0
perpendicular_distance_of_ball = 0
pub = None
pub_gate=None
twist_msg = Twist()
move_msg = Twist()
isBallVisible = False

class image_converter():
	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
		self.bridge = CvBridge()
		self.depth_array=np.zeros((640,480))
		rospy.Subscriber("/camera1/color/image_raw",Image,self.callback)
		rospy.Subscriber("/camera1/depth/points",PointCloud2,self.callback2)

	def callback2(self,data):
		self.depth_array=ros_numpy.numpify(data)
		#cv2.imshow("depth_array",self.depth_array)
		# print(self.depth_array)

	def callback(self,data):
		 
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)

		# convert to hsv
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))

		## slice the green
		imask = mask>0
		green = np.zeros_like(cv_image, np.uint8)
		green[imask] = cv_image[imask]
		green = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)


		# find contours in the thresholded image
		cnts = cv2.findContours(green.copy(), cv2.RETR_EXTERNAL,
						cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		#c_ = max(cnts, key=cv2.contourArea)
		# loop over the contours
		for c in cnts:
						# compute the center of the contour
						M = cv2.moments(c)
						global cX, cY,rad
						cX = int(M["m10"] / M["m00"])
						cY = int(M["m01"] / M["m00"])
						((x_, y_), radius) = cv2.minEnclosingCircle(c)
						rad=radius
						print("x in class is", cX)
						print("radius",radius)
						Yaw_idenify()
						keep_moving_forward()
						dumping()
						Dist_of_green_ball_after_alignment (radius)
						depth_Data=self.depth_array[cX][cY] 

						print(cX," ",cY)
						print(depth_Data)
						# draw the contour and center of the shape on the image
						cv2.drawContours(green, [c], -1, (0, 255, 0), 2)
						cv2.circle(green, (cX, cY), 7, (255, 255, 255), -1)

						# show the image
						cv2.imshow("Final", green)

		cv2.waitKey(3)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

theta = 10
#calculating required angular displacement to face green ball
def Yaw_idenify() :
	camera_view_angle = math.radians(60)
	global cX, pub, twist_msg, theta
	dx = (cX-320)

	theta = math.atan( math.tan( camera_view_angle/2 ) * dx / 320 )
	print("theta in yaw_identify is", theta)
	print("x in yaw_identify", cX)
	twist_msg.linear.x = 0
	twist_msg.angular.z = -theta
	pub.publish(twist_msg)
	#return theta
	
def return_yaw ():
	global perpendicular_distance_of_ball
	return perpendicular_distance_of_ball

def keep_moving_forward():
	global pub, move_msg, theta, cY, theta
	if(abs(theta)<0.01) or (cY > 475):
		move_msg.linear.x = 1.00
		move_msg.angular.z = -theta
		print("keep moving forward ")
		pub.publish(move_msg)

def dumping():
	global pub, move_msg, theta, cY, theta,pub_gate,rad
	if rad >= 320:
		pub.publish(Twist()) #cmd to stop
		for i in range(10):
			pub_gate.publish(np.float64(100).item()) #cmd to open gate
			rospy.sleep(0.2)
		for j in range(20):
			msg=Twist()
			msg.linear.x = -1.00
			msg.angular.z = 0.00
			pub.publish(msg)
			print("Reverse")
			rospy.sleep(0.2)
		for i in range(10):
			pub_gate.publish(np.float64(0).item()) #cmd to open gate
			rospy.sleep(0.2)
		pub.publish(Twist())
		pub_gate.publish(np.float64(100).item())
		rospy.signal_shutdown("Done")


#dumping()

def Dist_of_green_ball_after_alignment (radius) :
	global perpendicular_distance_of_ball
	camera_view_angle = math.radians(60)
	ball_radius = 0.05

	theta2 = math.atan( math.tan( camera_view_angle/2 ) * radius / 320 )

	perpendicular_distance_of_ball = ball_radius / math.tan(theta2)

	print ("distance after alignment" , perpendicular_distance_of_ball)
	



	'''
	twist_msg = Twist()
	twist_msg.linear.x = 0.75
	pub.publish(twist_msg)
	'''
	
	
	'''
	while True :
		if Dist_of_green_ball_after_alignment > 0.2 :
			twist_msg = Twist()
			twist_msg.linear.x = 0.75
			pub.publish(twist_msg)
			Rate.sleep(0.003)
		else :
			twist_msg = Twist()
			twist_msg.linear.x = 0.15
			pub.publish(twist_msg)
			Rate.sleep(2)
			twist_msg.linear.x = 0
			break
	'''

def main(args):
	global pub,pub_gate
	rospy.init_node('image_converter', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
	pub_gate = rospy.Publisher('/joint1_position_controller/command', Float64,queue_size=10)
	ic = image_converter()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		pub_gate.publish(np.float64(100).item())
	cv2.destroyAllWindows()


if __name__ == '__main__':

	main(sys.argv)
