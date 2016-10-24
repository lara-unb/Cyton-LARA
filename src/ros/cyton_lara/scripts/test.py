#!/usr/bin/env python
#
# Test Script for Cyton
# @author Rafael
#

import rospy
from sensor_msgs.msg import JointState
#from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np

def cyton_feedback_callback(data):
    rospy.loginfo(data);

if __name__ == '__main__':
    try:
        rospy.init_node('cyton_feedback',anonymous=False)
        rospy.Subscriber('/Cyton/jointAngles',JointState,cyton_feedback_callback)
        pub = rospy.Publisher('/Cyton/jointCmd', Float32MultiArray, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        i=-1
    	while not rospy.is_shutdown():
	        cmd_msg = Float32MultiArray()
	        cmd_msg.data = [0,0,0,0,0,0,90*np.sin(i)];
	        pub.publish(cmd_msg)
	        i = i +0.01
	        rate.sleep()
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass