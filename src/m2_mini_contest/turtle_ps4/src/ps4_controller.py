#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from m2_ps4.msg import Ps4Data


old_data = Ps4Data()

def callback(data):
    global old_data
 
    vel = Twist()	
    
    vel.angular.z = data.hat_rx
    vel.linear.x = data.hat_ly
    
    #publish the velocity
    pub.publish(vel)
    
    action = Pose2D()
    
    #shoot
    if ((data.r2 != old_data.r2)):
    	action.x = 1.0
    
    #reload
    if ((data.circle != old_data.circle)):
    	action.y = 1.0
    
    pub2.publish(action)
    	 
    old_data = data
    

if __name__ == '__main__':
    rospy.init_node('ps4_controller')

    #publish movement
    pub = rospy.Publisher('movement', Twist, queue_size = 1) 

    #publish action
    pub2 = rospy.Publisher('action', Pose2D, queue_size = 1)

    #ds4 subscriber
    sub = rospy.Subscriber('/input/ps4_data', Ps4Data, callback) 
    
    rospy.spin()
