#! /usr/bin/env python
 
# import ros stuff
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from dc2 import perpendicular_distance_of_ball
import os
import math 
import dc2
 
#variables
gate_threshold = 0.20
 
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
distance_from_draw_circle = 0
print("before initialize" , distance_from_draw_circle)
distance_from_draw_circle = dc2.return_yaw()
print("after initialize " , distance_from_draw_circle)
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
    global distance_from_draw_circle
   
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
    
    
    desired_position_.x = position_.x + distance_from_draw_circle*math.cos(yaw_) 
    desired_position_.y = position_.y + distance_from_draw_circle*math.sin(yaw_)
    desired_position_.z = 0

def change_state(state):
    global distance_from_draw_circle
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)
 
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    global distance_from_draw_circle

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
    global distance_from_draw_circle
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

'''
def call_from_draw_circle(distance_after_alignment):
    global distance_from_draw_circle
    distance_from_drawcircle = distance_after_alignment
    main()
'''

def main():
    global pub
   
    rospy.init_node('go_to_point')
   
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
   
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
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



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
