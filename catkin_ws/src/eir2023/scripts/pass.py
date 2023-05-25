#!/usr/bin/env python3
"""
This node implements an open loop set of movements to pass a vehicle.
(Actually, it only changes lane)
"""
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import Pose2D

def callback_start_passing(msg):
    global start_passing
    start_passing = True
    
def callback_sim_time(msg):
    global sim_secs    
    global sim_nsecs            
    current_time = msg
    sim_secs = current_time.clock.secs 
    sim_nsecs = current_time.clock.nsecs 

def main():
    global start_passing
    global sim_secs    
    global sim_nsecs
    
    state = 1 # Initial state of the maneuver
    elapsed_time = 0.0
    max_time_s1 = 0.65
    max_time_s2 = 0.65   
    max_time_s3 = 2.5
    max_time_s4 = 0.65    
    prev_time = 0.0
    first_time = True
    curr_time = 0.0

    sim_secs = 0.0
    sim_nsecs = 0.0                    
    
    print('INITIALIZING PASS-BEHAVIOR NODE...')
    rospy.init_node('passing')
    rate = rospy.Rate(30)

    rospy.Subscriber("/passing/start", Bool, callback_start_passing)
    rospy.Subscriber("/clock", Clock, callback_sim_time)     
    
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/passing/finished', Empty, queue_size=10)
    start_passing = False
     

    while not rospy.is_shutdown():
        if start_passing:
        
            curr_time = sim_secs + sim_nsecs / 1000000000
           
            if state == 1:
               print("Passing: moving left from right lane")

               if first_time:
                  prev_time = curr_time
                  first_time = False

               pub_speed.publish(36.0)
               pub_angle.publish(0.2)
               if elapsed_time >= max_time_s1:
                  state = 2
                  prev_time = curr_time
            elif state == 2:
               print("Passing: moving right left lane")
               pub_speed.publish(36.0)
               pub_angle.publish(-0.2)            
               if elapsed_time >= max_time_s2:
                  state = 3
                  prev_time = curr_time
            elif state == 3:
               print("Passing: moving straight left lane")
               pub_speed.publish(36.0)
               pub_angle.publish(0.0)            
               if elapsed_time >= max_time_s3:
                  state = 4
                  prev_time = curr_time
            elif state == 4:
               print("Passing: moving right from left lane")
               pub_speed.publish(36.0)
               pub_angle.publish(-0.2)            
               if elapsed_time >= max_time_s4:
                  print("Passing: finished")
                  pub_angle.publish(0.0)
                  pub_finish.publish()
                  state = 1
                  first_time = True
                  start_passing = False
                                    
            elapsed_time = curr_time - prev_time
            print("elapsed_time ", elapsed_time, flush=True)
                  
        rate.sleep()
    

if __name__ == "__main__":
    main()

    

