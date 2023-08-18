#!/usr/bin/env python3
"""
This node sends the cruise action to the car to provoke an accident. 
"""
import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Float64MultiArray, Empty, Bool

def cruise():
    global pub_cruise
    pub_cruise.publish(True)


def main():
    global success
    global pub_cruise
    print("INITIALIZING CRUISE_ACTION...")
    rospy.init_node("cruise_action")
    pub_start_signal  = rospy.Publisher("/policy_started", Empty, queue_size=1)
    pub_cruise = rospy.Publisher("/cruise/enable", Bool, queue_size=1)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pub_start_signal.publish()
        cruise()

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

