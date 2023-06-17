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

# Constants
INIT_PASSING = 1
ACCELERATE = 2
PASSING = 3
END_PASSING = 4
MAX_TIME_INIT_PASSING = 0.58
MAX_TIME_END_PASSING = 0.58
nSLEEP = 0.015 # 30000000 nsecs = 30 msecs    

def callback_start_passing(msg):
    global start_passing
    start_passing = True
    
def callback_obstacle_east(msg):
    global obstacle_east
    obstacle_east = msg.data      
    
def callback_sim_time(msg):
    global sim_secs    
    global sim_nsecs            
    current_time = msg
    sim_secs = current_time.clock.secs 
    sim_nsecs = current_time.clock.nsecs 
    
def main():
    global start_passing, sim_secs, sim_nsecs, obstacle_east
    
    state = INIT_PASSING # Initial state of the maneuver
    elapsed_time = 0.0
    prev_time = 0.0
    first_time = True
    curr_time = 0.0

    sim_secs = 0.0
    sim_nsecs = 0.0                    
    
    print('INITIALIZING PASS-BEHAVIOR NODE...')
    rospy.init_node('passing')

    rospy.Subscriber("/passing/start", Bool, callback_start_passing)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
    rospy.Subscriber("/obstacle/east", Bool, callback_obstacle_east)
        
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/passing/finished', Empty, queue_size=10)
    pub_requested_speed = rospy.Publisher('/passing/requested_speed', Float64, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
    start_passing = False
     
    rospy.init_node('passing')
    rate = rospy.Rate(30)

    rospy.Subscriber("/passing/start", Bool, callback_start_passing)
    rospy.Subscriber("/clock", Clock, callback_sim_time)     
    
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/passing/finished', Empty, queue_size=10)
    start_passing = False
    while not rospy.is_shutdown():

        curr_time = sim_secs + sim_nsecs / (10**9)

        if start_passing:
           
            if state == INIT_PASSING:
               if first_time:
                  print("Passing: init passing", flush=True)
                  prev_time = curr_time
                  first_time = False

               pub_speed.publish(36.0)
               pub_angle.publish(0.2)
               
               elapsed_time = curr_time - prev_time
               if elapsed_time >= MAX_TIME_INIT_PASSING:
                  print("Elapsed_time", elapsed_time, flush=True)
                  state = ACCELERATE
                  first_time = True
            elif state == ACCELERATE:
               if first_time:
                  print("Passing: accelerate", flush=True)
                  prev_time = curr_time
                  first_time = False
                                
               pub_requested_speed.publish(55.0)
               pub_steady_motion.publish(True)
                         
               if obstacle_east:
                  state = PASSING
                  first_time = True
            elif state == PASSING:

               if first_time:
                  print("Passing: passing", flush=True)
                  pub_steady_motion.publish(True)
                  prev_time = curr_time
                  first_time = False          

               if not obstacle_east:
                  pub_steady_motion.publish(False)
                  # Next line MUST be here 
                  # to override the previous speed request
                  pub_requested_speed.publish(0.0)
                  state = END_PASSING
                  first_time = True                                   
            elif state == END_PASSING:
              
               if first_time:
                  print("Passing: ending passing", flush=True)
                  prev_time = curr_time
                  pub_speed.publish(36.0)
                  pub_angle.publish(-0.2)   
                  first_time = False
   
               elapsed_time = curr_time - prev_time 
               if elapsed_time >= MAX_TIME_END_PASSING:
                  print("Passing: finished", flush=True)
                  #print("elapsed_time ", elapsed_time, "curr_time ", curr_time, "prev_time ", prev_time, flush=True)
                  pub_angle.publish(0.0)
                  pub_finish.publish()
                  state = INIT_PASSING
                  first_time = True
                  start_passing = False
            
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

    

