#!/usr/bin/env python3
"""
This node implements change lane from right to left.
"""
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import Pose2D

# Constants
INIT_CHANGE_LANE_L_TO_R = 1
MAX_TIME_INIT_CHANGE_LANE_L_TO_R = 0.58
nSLEEP = 0.015 # 30000000 nsecs = 30 msecs    

def callback_start_change_lane_l_to_r(msg):
    global start_change_lane_l_to_r
    start_change_lane_l_to_r = True
    
def callback_sim_time(msg):
    global sim_secs    
    global sim_nsecs            
    current_time = msg
    sim_secs = current_time.clock.secs 
    sim_nsecs = current_time.clock.nsecs 
    
def main():
    global start_change_lane_l_to_r, sim_secs, sim_nsecs
    
    state = INIT_CHANGE_LANE_L_TO_R # Initial state of the maneuver
    elapsed_time = 0.0
    prev_time = 0.0
    first_time = True
    curr_time = 0.0

    sim_secs = 0.0
    sim_nsecs = 0.0                    
    
    print('INITIALIZING CHANGE_LANE_L_TO_R NODE...')
    rospy.init_node('change_lane_l_to_r')

    rospy.Subscriber("/change_lane_l_to_r/start", Bool, callback_start_change_lane_l_to_r)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
        
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/change_lane_l_to_r/finished', Empty, queue_size=10)
    pub_requested_speed = rospy.Publisher('/change_lane_l_to_r/requested_speed', Float64, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
     
    rospy.init_node('change_lane_l_to_r')
    rate = rospy.Rate(30)

    rospy.Subscriber("/change_lane_l_to_r/start", Bool, callback_start_change_lane_l_to_r)
    rospy.Subscriber("/clock", Clock, callback_sim_time)     
    
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/change_lane_l_to_r/finished', Empty, queue_size=10)
    start_change_lane_l_to_r = False
    while not rospy.is_shutdown():
    
        curr_time = sim_secs + sim_nsecs / (10**9)

        if start_change_lane_l_to_r:           
            if first_time:
               print("change_lane_l_to_r: init change_lane_l_to_r", flush=True)
               prev_time = curr_time
               first_time = False

            pub_speed.publish(36.0)
            pub_angle.publish(-0.2)
            
            elapsed_time = curr_time - prev_time
            if elapsed_time >= MAX_TIME_INIT_CHANGE_LANE_L_TO_R:
               print("Elapsed_time", elapsed_time, flush=True)
               first_time = True

               pub_angle.publish(0.0)
               pub_finish.publish()
               first_time = True
               start_change_lane_l_to_r = False
            
        else:
            continue

        # My sleep
        i = 0
        while i < 1 and not rospy.is_shutdown():
            prev_sleep = sim_secs + sim_nsecs / (10**9)
            i = i + 1
             
        diff = 0.0
        while diff <= nSLEEP and not rospy.is_shutdown():
            curr_time = sim_secs + sim_nsecs / (10**9)            
            diff  = curr_time - prev_sleep
            
        rate.sleep()
   

if __name__ == "__main__":
    main()

    

