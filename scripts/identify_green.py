#!/usr/bin/env python
 

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, MultiArrayDimension
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
bridge=CvBridge()
pub = rospy.Publisher('ball_msg', MultiArrayDimension, queue_size=1)
color_boundaries = [ ([0, 0, 220], [30, 30, 255]), ([0, 220, 0], [30, 255, 30])]
msg=MultiArrayDimension()

def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    try:
        img = bridge.imgmsg_to_cv2(img_msg, "passthrough")

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (400,400))
        classifier(img)


    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def classifier(img):
    masks = []
    for (low, up) in color_boundaries:
        low=np.array(low, dtype='uint8')
        up=np.array(up, dtype='uint8')
        mask=cv2.inRange(img, low, up)
        op=cv2.bitwise_and(img, img, mask=mask)
        masks.append(op)

    red_zone = cv2.cvtColor(masks[0], cv2.COLOR_BGR2GRAY)
    green_zone = cv2.cvtColor(masks[1], cv2.COLOR_BGR2GRAY)

    _, red_zone = cv2.threshold(red_zone, 20, 255, cv2.THRESH_BINARY)
    _, green_zone = cv2.threshold(green_zone, 20, 255, cv2.THRESH_BINARY)

    _,contours_red, hierarchy = cv2.findContours(red_zone, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _,contours_green, hierarchy = cv2.findContours(green_zone, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    MAX = 0.0
    color = ""
    idx = -1

    for i in range(len(contours_red)):
        if(cv2.contourArea(contours_red[i])>MAX):
            MAX = cv2.contourArea(contours_red[i])
            color = "red"
            idx = i

    for i in range(len(contours_green)):
        if(cv2.contourArea(contours_green[i])>MAX):
            MAX = cv2.contourArea(contours_green[i])
            color = "green"
            idx = i

    if(idx != -1 and MAX>70):
        if(color == "green"):
            M = cv2.moments(contours_green[idx])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img, (cx, cy), 2, (255, 0, 0), 2)
            msg.label='green'
            msg.size=cx
            msg.stride=cy
            pub.publish(msg)
            print("green", cx, cy)
            #return ("green", cx, cy)
        else:
            M = cv2.moments(contours_red[idx])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(img, (cx, cy), 2, (255, 0, 0), 2)
            msg.label='red'
            msg.size=cx
            msg.stride=cy
            pub.publish(msg)
            print("red", cx, cy)
            #return ("red", cx, cy)
        cv2.imshow('',img)
        cv2.waitKey(10)
    else:
        cv2.imshow('',img)
        cv2.waitKey(10)
        msg.label='None'
        msg.size=200
        msg.stride=400
        pub.publish(msg)
        print("None")
         
        #return ("No ball")

def listener():

    rospy.init_node('ball_detection',anonymous=True)
    rospy.Subscriber("/camera1/color/image_raw", Image, image_callback)
    rospy.spin()



### RUN ###
if __name__ == '__main__':
    listener()