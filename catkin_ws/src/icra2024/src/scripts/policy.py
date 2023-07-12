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
    
def callback_obstacle_east(msg):
    global obstacle_east
    obstacle_east = msg.data    

def callback_obstacle_south_west(msg):
    global obstacle_south_west
    obstacle_south_west = msg.data

def callback_success(msg):
    global success
    success = msg.data

def enable_steady_motion():
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_action, pub_stop
    pub_steady_motion.publish(True)
    pub_follow_car.publish(False)
    pub_stop.publish(False)

def enable_follow_car():
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_action, pub_stop
    pub_follow_car.publish(True)
    pub_steady_motion.publish(False)
    pub_stop.publish(False)

def change_lane_r_to_l():
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_action, pub_stop
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(False)
    pub_change_lane_r_to_l.publish(True)
    msg_finished = rospy.wait_for_message('/change_lane_r_to_l/finished', Empty, timeout=100.0)
    
def stop_motion():
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_action, pub_stop
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(True)    

def main():
    global obstacle_north, obstacle_north_west, obstacle_west, obstacle_east, obstacle_south_west, success
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_action, pub_stop
    print("INITIALIZING POLICY...")
    rospy.init_node("policy")
    rospy.Subscriber("/obstacle/north"     , Bool, callback_obstacle_north)
    rospy.Subscriber("/obstacle/north_west", Bool, callback_obstacle_north_west)
    rospy.Subscriber("/obstacle/west"      , Bool, callback_obstacle_west)
    rospy.Subscriber("/obstacle/east"      , Bool, callback_obstacle_east)    
    rospy.Subscriber("/obstacle/south_west", Bool, callback_obstacle_south_west)
    rospy.Subscriber("/success", Bool, callback_success)    
    pub_start_signal  = rospy.Publisher("/start", Empty, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
    pub_follow_car    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_change_lane_r_to_l = rospy.Publisher("/change_lane_r_to_l/start", Bool, queue_size=10)
    pub_action = rospy.Publisher("/action", String, queue_size=10)
    pub_stop = rospy.Publisher("/stop", Bool, queue_size=10)    
    rate = rospy.Rate(10)

    obstacle_north      = False
    obstacle_north_west = False
    obstacle_west       = False
    obstacle_east       = False    
    obstacle_south_west = False
    success = True

    action = "NA"
    action_prev = "NA"            
    while not rospy.is_shutdown():
        pub_start_signal.publish()

        #####################################
        if right_lane:
           if success:
              if obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Change lane"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane_r_to_l()
              elif not obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Change lane"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane_r_to_l()
              elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
           else:       
              action = "stop"
              stop_motion() 
                                              
           pub_action.publish(action)
           
        #####################################   
        if left_lane:    

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

