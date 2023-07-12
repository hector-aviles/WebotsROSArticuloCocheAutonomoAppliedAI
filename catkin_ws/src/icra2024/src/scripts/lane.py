#!/usr/bin/env python3
"""
This node determines if the car is at the right or left lane of the road. 
"""
import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

def callback_right_lane(msg):
    global pub_right_lane, in_right_lane, x, y, theta   

    x = msg.x
    y = msg.y
    z = msg.theta

    in_right_lane = True 
    # But...
    if y > 0:
       in_right_lane =  False

    pub_in_right_lane.publish(in_right_lane)


def main():
    global pub_in_right_lane, in_right_lane, x, y , theta
    
    x = 0.0
    y = 0.0
    theta = 0.0
    in_right_lane = False
    
    print("INITIALIZING LANE NODE...")
    rospy.init_node("LANE")

    rospy.Subscriber("/current_pose", Pose2D, callback_right_lane)
    pub_in_right_lane  = rospy.Publisher("/in_right_lane", Bool, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
    
        #print("In right lane ", in_right_lane, flush=True)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

