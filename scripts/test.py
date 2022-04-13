#!/usr/bin/env python
from __future__ import print_function
import time
import roslib
roslib.load_manifest('umic')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
import ros_numpy
import math
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import os

#defining x,y of green ball for global use
x = 0
y = 0
perpendicular_distance_of_ball = 0 
pub = None


#######################################################################################

#variables
gate_threshold = 0.20
 
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
'''
perpendicular_distance_of_ball = 0
print("before initialize" , perpendicular_distance_of_ball)
perpendicular_distance_of_ball = perpendicular_distance_of_ball
print("after initialize " , perpendicular_distance_of_ball)
'''
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0

# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3
 
# publishers
pub = None

# callbacks
def clbk_odom(msg):
	global position_
	global yaw_
	global perpendicular_distance_of_ball
   
	# position
	position_ = msg.pose.pose.position
   
	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
	
	
	desired_position_.x = position_.x + (0.2 + perpendicular_distance_of_ball)*math.cos(yaw_) 
	desired_position_.y = position_.y + (0.2 + perpendicular_distance_of_ball)*math.sin(yaw_)
	desired_position_.z = 0

def change_state(state):
	global perpendicular_distance_of_ball
	global state_
	state_ = state
	print ('State changed to [%s]' % state_)
 
def fix_yaw(des_pos):
	global yaw_, pub, yaw_precision_, state_
	global perpendicular_distance_of_ball

	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
   
	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_:
		twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
   
	pub.publish(twist_msg)
   
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_:
		print('Yaw error: [%s]' % err_yaw)
		change_state(1)




def go_straight_ahead(des_pos):
	global yaw_, pub, yaw_precision_, state_
	global perpendicular_distance_of_ball
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	

	#desired_yaw = draw_circle.Yaw_idenify()
	print(desired_yaw)
	
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
   
	#if err_pos < gate_threshold:
		#publish gate vertical motion
 
	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 0.6
		pub.publish(twist_msg)
	else:
		print ('Position error: [%s]' % err_pos)
		change_state(2)
   
	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
		print ('Yaw error: [%s]' % err_yaw)
		change_state(0)

def gate_up():
	os.system("rosrun umic move_gate_up.py")

def gate_down():
	move_gate_up.pub.publish(np.float64(0).item())

def done():
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)


#########################################################################################





class image_converter():
	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
		self.bridge = CvBridge()
		self.depth_array=np.zeros((480,640))
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
						cX = int(M["m10"] / M["m00"])
						cY = int(M["m01"] / M["m00"])

						((x_, y_), radius) = cv2.minEnclosingCircle(c)

						global x, y, theta

						x = cX
						print("x in class is", x)
						y = cY
						while(theta):
							Yaw_idenify()
						Dist_of_green_ball_after_alignment (radius)

						
						#clbk_odom(odom_msg)


						#motion_planning.call_from_draw_circle(distance_value_aligned)

						# depth_Data=self.depth_array[cX][cY] 

						# print(cX," ",cY)
						# print(depth_Data)
						# # draw the contour and center of the shape on the image
						# cv2.drawContours(green, [c], -1, (0, 255, 0), 2)
						# cv2.circle(green, (cX, cY), 7, (255, 255, 255), -1)

						# show the image
						cv2.imshow("Final", green)

		cv2.waitKey(3)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

theta = 1.0

#calculating required angular displacement to face green ball
def Yaw_idenify() :
	global theta
	
	camera_view_angle = math.radians(60)
	global x, pub
	dx = (x-320)

	theta = math.atan( math.tan( camera_view_angle/2 ) * dx / 320 )
	print("theta in yaw_identify is", theta)
	print("x in yaw_identify", x)
	twist_msg = Twist()
	twist_msg.angular.z = -theta
	pub.publish(twist_msg)
	#return theta
	
def return_yaw ():
	global distance
	return 

def Dist_of_green_ball_after_alignment (radius) :
	global perpendicular_distance_of_ball
	camera_view_angle = math.radians(60)
	ball_radius = 0.05

	theta2 = math.atan( math.tan( camera_view_angle/2 ) * radius / 320 )

	perpendicular_distance_of_ball = ball_radius / math.tan(theta2)

	print ("distance after alignment" , perpendicular_distance_of_ball)



	#return perpendicular_distance_of_ball
	
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
	global pub
	rospy.init_node('image_converter', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

	ic = image_converter()
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		twist_msg = Twist()
		twist_msg.linear.x = 0.75
		pub.publish(twist_msg)
		if state_ == 0:
			fix_yaw(desired_position_)
		elif state_ == 1:
			go_straight_ahead(desired_position_)
		elif state_ == 2:
			done()
			pass
		else:
			rospy.logerr('Unknown state!')
			pass
		#move_gate_up.main()
		rate.sleep()


	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	try :
		main(sys.argv)
	except rospy.ROSInterruptException:
		pass
