#!/usr/bin/env python3
"""
This node implements an open loop set of movements to pass a vehicle.
(Actually, it only changes lane)
"""
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool

def callback_start_passing(msg):
    global start_passing
    start_passing = True

def main():
    global start_passing
    turning_left_time = 0.7*8
    turning_right_time = 0.7*8
    print('INITIALIZING PASS-BEHAVIOR NODE...')
    rospy.init_node('passsing')
    rate = rospy.Rate(30)

    rospy.Subscriber("/passing/start", Bool, callback_start_passing)
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/passing/finished', Empty, queue_size=10)
    print("Passing parameters:")
    print("Turning_left_time: " + str(turning_left_time))
    print("Turning_right_time: " + str(turning_right_time))
    start_passing = False

    while not rospy.is_shutdown():
        if start_passing:
            start_passing = False
            print("Passing: moving left")
            pub_speed.publish(36.0)
            pub_angle.publish(0.2)
            rospy.sleep(turning_left_time)
            print("Passing: moving right")
            pub_speed.publish(36.0)
            pub_angle.publish(-0.2)
            rospy.sleep(turning_right_time)
            print("Passing: finished")
            pub_angle.publish(0.0)
            pub_finish.publish()
        rate.sleep()
    

if __name__ == "__main__":
    main()

    

