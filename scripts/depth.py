#!/usr/bin/env python
import roslib
roslib.load_manifest('umic')
import sys
import rospy
import cv2
#import cv2.cv
import message_filters
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
 
class follow_person:
    def __init__(self):
        self.bridge = CvBridge()
 
        image_sub = message_filters.Subscriber("/camera1/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)
 
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
        self.ts.registerCallback(self.callback)
 
        self.body_cascade = cv2.CascadeClassifier('/home/areeb/catkin_ws/src/umic/package.xml')
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
 
    def callback(self, rgb_data, depth_data):
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
        except CvBridgeError as e:
            print (e)
 
        depth_array = np.array(depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
 
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        upper_bodys = self.body_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=10,
            minSize=(100,100),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )
 
        for (x, y, w, h) in upper_bodys:
            cv2.rectangle(depth_array, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(depth_array, (x+(w/2), h), 3, (171,110,0), 2)
 
        cv2.imshow('Body Recognition', depth_array)
        cv2.waitKey(3)
 
def main(args):
    fp = follow_person()
    rospy.init_node('follow_person', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)