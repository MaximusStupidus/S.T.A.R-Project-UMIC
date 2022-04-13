#! /usr/bin/env python
 
 
#do not consider green balls
 
import rospy
 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None
regions = {'front':0,'fleft':0,'right':0,'fright':0,'left':0}
thresh = 0.5
thresh2 = 6*thresh
 
def clbk_laser(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:143]), 5),
        'fright': min(min(msg.ranges[144:287]), 5),
        'front':  min(min(msg.ranges[288:431]), 5),
        'fleft':  min(min(msg.ranges[432:575]), 5),
        'left':   min(min(msg.ranges[576:719]), 5),
    }
   
    take_action(regions)
   
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
   
    state_description = ''
    thresh = 0.5
   
    if regions['front'] > thresh and regions['fleft'] > thresh and regions['fright'] > thresh:
        state_description = 'case 1 - nothing'
        linear_x = 0.6
        angular_z = 0
    elif regions['front'] < thresh and regions['fleft'] > thresh and regions['fright'] > thresh:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > thresh and regions['fleft'] > thresh and regions['fright'] < thresh:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > thresh and regions['fleft'] < thresh and regions['fright'] > thresh:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < thresh and regions['fleft'] > thresh and regions['fright'] < thresh:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < thresh and regions['fleft'] < thresh and regions['fright'] > thresh:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < thresh and regions['fleft'] < thresh and regions['fright'] < thresh:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > thresh and regions['fleft'] < thresh and regions['fright'] < thresh:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

        
    if (regions['front'] > thresh2 and regions['fleft'] > thresh2 and regions['fright'] > thresh2 and regions['left'] > thresh2 and regions['right'] > thresh2) :
        pub.publish(Twist())
        rospy.signal_shutdown("Done")
 
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
 
def main():
    global pub, regions
   
    rospy.init_node('reading_laser')
   
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   
    rospy.Subscriber('/bot1/laser/scan', LaserScan, clbk_laser)



   
    rospy.spin()
    
 
if __name__ == '__main__':
    main()

    pub.publish(Twist())