#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from m2_ps4.msg import Ps4Data
import time

old_data = Ps4Data()

x=90
y=146

def callback(data):
    global old_data
    global x
    global y
    
    if x>179:	x=179
    elif x<1:	x=1
    
    if y>179:	y=179
    elif y<90:	y=90
    	
    rx=data.hat_rx
    ly=data.hat_ly
    
    if (rx>0.1):
    	x+=1
    elif(rx<-0.1):
    	x-=1  
    	
    pub1.publish(x)
    
    if (ly>0.1):
    	y+=1
    if (ly<-0.1):
    	y-=1
    	
    	
    pub2.publish(y)
    
    #angle y = 168
    if ((data.dpad_y == 1.0 and  data.dpad_y != old_data.dpad_y)):
    	y=148
    	pub2.publish(y)
    	
    if ((data.dpad_y == -1.0 and  data.dpad_y != old_data.dpad_y)):
    	y=168
    	pub2.publish(y)
    #shoot
    if ((data.cross == True and  data.cross != old_data.cross)):
    	pub4.publish(1)
    	time.sleep(1.5)
    	pub3.publish(90)
    	time.sleep(0.4)
    	pub3.publish(30)
    
    #reload
    if ((data.circle == True and data.circle != old_data.circle)):
    	pub3.publish(90)
    	time.sleep(0.4)
    	pub3.publish(30)
    
    	 
    old_data = data
    

if __name__ == '__main__':
    rospy.init_node('ps4_controller')

    #publish movement
    pub1 = rospy.Publisher('movement_x', Int16, queue_size = 1)
    pub2 = rospy.Publisher('movement_y', Int16, queue_size = 1)

    #publish action
    pub3 = rospy.Publisher('reload', Int16, queue_size = 1)
    pub4 = rospy.Publisher('shoot', Int16, queue_size = 1)

    #ds4 subscriber
    sub = rospy.Subscriber('/input/ps4_data', Ps4Data, callback) 
    
    rospy.spin()
