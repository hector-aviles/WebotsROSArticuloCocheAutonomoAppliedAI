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

def callback_obstacle_north(msg):
    global obstacle_north
    obstacle_north = msg.data

def callback_obstacle_north_west(msg):
    global obstacle_north_west
    obstacle_north_west = msg.data

def callback_obstacle_west(msg):
    global obstacle_west
    obstacle_west = msg.data

def callback_obstacle_south_west(msg):
    global obstacle_south_west
    obstacle_south_west = msg.data

def enable_steady_motion():
    global pub_steady_motion
    pub_steady_motion.publish(True)
    pub_follow_car.publish(False)

def enable_follow_car():
    global pub_follow_car
    pub_follow_car.publish(True)
    pub_steady_motion.publish(False)

def execute_passing():
    global pub_start_passing
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_start_passing.publish(True)
    msg_finished = rospy.wait_for_message('/passing/finished', Empty, timeout=100.0)

def main():
    global obstacle_north, obstacle_north_west, obstacle_west, obstacle_south_west
    global pub_follow_car, pub_steady_motion, pub_start_passing
    print("INITIALIZING POLICY...")
    rospy.init_node("policy")
    rospy.Subscriber("/obstacle/north"     , Bool, callback_obstacle_north)
    rospy.Subscriber("/obstacle/north_west", Bool, callback_obstacle_north_west)
    rospy.Subscriber("/obstacle/west"      , Bool, callback_obstacle_west)
    rospy.Subscriber("/obstacle/south_west", Bool, callback_obstacle_south_west)
    pub_start_signal  = rospy.Publisher("/start", Empty, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
    pub_follow_car    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_start_passing = rospy.Publisher("/passing/start", Bool, queue_size=10)
    pub_action = rospy.Publisher("/action", String, queue_size=10)
    rate = rospy.Rate(10)

    obstacle_north      = False
    obstacle_north_west = False
    obstacle_west       = False
    obstacle_south_west = False
    
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
        action = "NA"
        if obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
            action = "Keep distance"
            enable_follow_car()
        elif not obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
            action = "Keep distance"
            enable_follow_car()
        elif not obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
            action = "Keep distance"
            enable_follow_car()
        elif not obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
            action = "Keep distance"
            enable_follow_car()
        elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
            action = "Keep distance"
            enable_follow_car()
        elif not obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
            action = "Change lane"
            execute_passing()
        elif not obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
            action = "Keep distance"
            enable_follow_car()
        elif not obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
            action = "Cruise"
            enable_steady_motion()
        elif obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
            action = "Change lane"
            execute_passing()
        elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
            action = "Cruise"
            enable_steady_motion()

        print(action, flush = True)    
        pub_action.publish(action)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

