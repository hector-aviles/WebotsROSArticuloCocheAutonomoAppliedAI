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

def callback_obstacle_north_east(msg):
    global obstacle_north_east
    obstacle_north_east = msg.data    

def callback_obstacle_east(msg):
    global obstacle_east
    obstacle_east = msg.data    
    
def callback_obstacle_south_east(msg):
    global obstacle_south_east
    obstacle_south_east = msg.data        

def callback_success(msg):
    global success
    success = msg.data
    
def callback_in_right_lane(msg):
    global in_right_lane
    in_right_lane = msg.data    

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
    global pub_requested_speed, pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_change_lane_l_to_r, pub_stop

    #pub_requested_speed.publish(0.0)        
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(False)
    pub_change_lane_l_to_r.publish(False)
    pub_change_lane_r_to_l.publish(True)
    
    msg_finished = rospy.wait_for_message('/change_lane_r_to_l/finished', Empty, timeout=100.0)
    
def change_lane_l_to_r():
    global pub_requested_speed, pub_follow_car, pub_steady_motion, pub_change_lane_l_to_r, pub_change_lane_r_to_l, pub_stop
    # Stop accelerating
    #pub_requested_speed.publish(0.0)    
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(False)
    #pub_change_lane_r_to_l.publish(False)
    pub_change_lane_l_to_r.publish(True)
    msg_finished = rospy.wait_for_message('/change_lane_l_to_r/finished', Empty, timeout=100.0)    

def accelerate():
    #global pub_steady_motion, pub_requested_speed
    #pub_requested_speed.publish(55.0)
    #pub_steady_motion.publish(True)  
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_change_lane_l_to_r, pub_action, pub_stop
    pub_steady_motion.publish(True)
    pub_follow_car.publish(False)
    pub_stop.publish(False)      

def stop_motion():
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_action, pub_stop
    pub_steady_motion.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(True)    

def main():
    global obstacle_north, obstacle_north_west, obstacle_west, obstacle_east, obstacle_south_west, success, in_right_lane
    global pub_follow_car, pub_steady_motion, pub_change_lane_r_to_l, pub_change_lane_l_to_r, pub_requested_speed, pub_action, pub_stop
    print("INITIALIZING POLICY...")
    rospy.init_node("policy")
    rospy.Subscriber("/obstacle/north"     , Bool, callback_obstacle_north)
    rospy.Subscriber("/obstacle/north_west", Bool, callback_obstacle_north_west)
    rospy.Subscriber("/obstacle/west"      , Bool, callback_obstacle_west)
    rospy.Subscriber("/obstacle/south_west", Bool, callback_obstacle_south_west)
    rospy.Subscriber("/obstacle/north_east", Bool, callback_obstacle_north_east)
    rospy.Subscriber("/obstacle/east"      , Bool, callback_obstacle_east)    
    rospy.Subscriber("/obstacle/south_east", Bool, callback_obstacle_south_east)
    rospy.Subscriber("/success", Bool, callback_success)    

    rospy.Subscriber("/in_right_lane", Bool, callback_in_right_lane)

    #print("Hola mundo", flush = True)  

    pub_start_signal  = rospy.Publisher("/start", Empty, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
    pub_follow_car    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_change_lane_r_to_l = rospy.Publisher("/change_lane_r_to_l/start", Bool, queue_size=10)
    pub_change_lane_l_to_r = rospy.Publisher("/change_lane_l_to_r/start", Bool, queue_size=10)    
    #pub_requested_speed = rospy.Publisher('/accelerate/requested_speed', Float64, queue_size=10)
    pub_action = rospy.Publisher("/action", String, queue_size=10)
    pub_stop = rospy.Publisher("/stop", Bool, queue_size=10)    

    rate = rospy.Rate(10)

    obstacle_north      = False
    obstacle_north_west = False
    obstacle_west       = False
    obstacle_south_west = False
    obstacle_north_east = False    
    obstacle_east = False    
    obstacle_south_east = False    
    success = True
    in_right_lane = True

    action = "NA"
    action_prev = "NA"            
    while not rospy.is_shutdown():
        pub_start_signal.publish()

        # Right lane
        if in_right_lane:
           if success:
              if obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Change lane"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane_r_to_l()
              elif not obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
              elif obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Change lane"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane_r_to_l()
              elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Cruise"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_steady_motion()
           else:       
              action = "Right stop"
              stop_motion() 
                                              
           pub_action.publish(action)
        '''   
        # Left lane   
        else:    
           if success:
              if obstacle_north and obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and not obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and not obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and not obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Change lane"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane_l_to_r()
              elif not obstacle_north and not obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Keep distance"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
              elif obstacle_north and not obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Change lane"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane_l_to_r()
              elif not obstacle_north and not obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Accelerate"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_accelerate()
           else:       
              action = "Left stop"
              stop_motion()           
        '''

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

