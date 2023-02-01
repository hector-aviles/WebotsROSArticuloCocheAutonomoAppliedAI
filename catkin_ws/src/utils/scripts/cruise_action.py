#!/usr/bin/env python3
"""
This node is intended to be used to code the resulting policy
after modeling and training the MDPs using ProbLog.
This node provides serveral functions to check if there are
other vehicles around the car and to execute the three
different behaviors: steady motion, follow and pass. 
"""
import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Float64MultiArray, Empty, Bool, String
from sensor_msgs.msg import PointCloud2


def callback_success(msg):
    global success
    success = msg.data

def enable_steady_motion():
    global pub_follow_car, pub_steady_motion, pub_start_passing, pub_action, pub_stop
    pub_steady_motion.publish(True)
    pub_follow_car.publish(False)
    pub_stop.publish(False)
   
def stop_motion():
    global pub_follow_car, pub_steady_motion, pub_start_passing, pub_action, pub_stop
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(True)    

def main():
    global success
    global pub_follow_car, pub_steady_motion, pub_start_passing, pub_action, pub_stop
    print("INITIALIZING POLICY...")
    rospy.init_node("cruise_action")
    rospy.Subscriber("/success", Bool, callback_success)    
    pub_start_signal  = rospy.Publisher("/start", Empty, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
    pub_follow_car    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_start_passing = rospy.Publisher("/passing/start", Bool, queue_size=10)
    pub_stop = rospy.Publisher("/stop", Bool, queue_size=10)    
    pub_action = rospy.Publisher("/action", String, queue_size=10)    
    rate = rospy.Rate(10)
    
    success = True

    action = "NA"
    action_prev = "NA"      
    while not rospy.is_shutdown():
        pub_start_signal.publish()
        #
        # TODO:
        # Code here the resulting policy
        # You can use the variables obstacle_north, obstacle_north_west, obstacle_west and obstacle_south_west
        # You can start the different behaviors with the functions:
        # execute_passing()
        # enable_steady_motion()
        # enable_follow_car()
        #

        if success:
           action = "Cruise"
           if action_prev != action:
              print(action, flush = True)                      
              action_prev = action  
           enable_steady_motion()
        else:       
           action = "Stop"
           if action_prev != action:
              print(action, flush = True)                      
              action_prev = action             
           stop_motion()     
          
        pub_action.publish(action)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

