#!/usr/bin/env python3
"""
This node implements change lane from right to left.
"""
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
from icra2024.msg import TwoBool

from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import Pose2D

# Constants
MAX_TIME_CHANGE_LANE = 0.58
nSLEEP = 0.015 # 30000000 nsecs = 30 msecs    

def callback_start_change_lane(msg):
    global start_change_lane, right_lane
    start_change_lane = msg.start
    right_lane = msg.right_lane
        
def callback_sim_time(msg):
    global sim_secs    
    global sim_nsecs            
    current_time = msg
    sim_secs = current_time.clock.secs 
    sim_nsecs = current_time.clock.nsecs 
    
def main():
    global start_change_lane, right_lane, sim_secs, sim_nsecs
    
    elapsed_time = 0.0
    prev_time = 0.0
    first_time = True
    curr_time = 0.0
    start_change_lane = False
    right_lane = False    

    sim_secs = 0.0
    sim_nsecs = 0.0                    
    
    print('INITIALIZING CHANGE_LANE NODE...')
    rospy.init_node('change_lane')

    rospy.Subscriber("/change_lane/start", TwoBool, callback_start_change_lane)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
        
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish_right_to_left = rospy.Publisher('/change_lane/finished', Empty, queue_size=10)
    pub_requested_speed = rospy.Publisher('/accelerate/requested_speed', Float64, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
     
    rate = rospy.Rate(30)

    start_change_lane = False
    while not rospy.is_shutdown():
    
        curr_time = sim_secs + sim_nsecs / (10**9)

        if start_change_lane:           
            if first_time:
               print("change_lane: start change_lane", flush=True)
               prev_time = curr_time
               first_time = False
               #pub_requested_speed.publish(50.0)
               if right_lane:
                  pub_angle.publish(0.2)
               else:    
                  pub_angle.publish(-0.2)
                  
            elapsed_time = curr_time - prev_time
            if elapsed_time >= MAX_TIME_CHANGE_LANE:
               print("Elapsed_time", elapsed_time, flush=True)
               first_time = True

               pub_angle.publish(0.0)
               #pub_requested_speed.publish(0.0)
               pub_finish.publish()
               print("change_lane: finish change_lane", flush=True) 
               first_time = True
               start_change_lane = False
        #elif:
        #if start_change_lane_left_to_right
            
        else:
            continue
        #'''
        # My sleep
        i = 0
        while i < 1 and not rospy.is_shutdown():
            prev_sleep = sim_secs + sim_nsecs / (10**9)
            i = i + 1         
        diff = 0.0
        while diff <= nSLEEP and not rospy.is_shutdown():
            curr_time = sim_secs + sim_nsecs / (10**9)            
            diff  = curr_time - prev_sleep
        #'''            
        #rate.sleep()
   

if __name__ == "__main__":
    main()

    

