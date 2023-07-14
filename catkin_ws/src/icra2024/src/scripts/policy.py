#!/usr/bin/env python3
"""
This node is intended to be used to code the resulting policy
after modeling and training the MDPs using ProbLog.
This node provides several functions to check if there are
other vehicles around the car and to execute the three
different behaviors: steady motion, follow and pass. 
"""
import rospy
from std_msgs.msg import Float64MultiArray, Empty, Bool, String
from icra2024.msg import TwoBool

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
    
def callback_right_lane(msg):
    global right_lane
    right_lane = msg.data        

def enable_cruise():
    global pub_follow_car, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, right_lane
    pub_cruise.publish(True)
    pub_follow_car.publish(False)
    pub_stop.publish(False)
    msg = TwoBool()
    msg.start = False
    msg.right_lane = right_lane
    pub_change_lane.publish(msg)

def enable_follow_car():
    global pub_follow_car, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, right_lane
    pub_follow_car.publish(True)
    pub_cruise.publish(False)
    pub_stop.publish(False)
    msg = TwoBool()
    msg.start = False
    msg.right_lane = right_lane 
    pub_change_lane.publish(msg)
    

def change_lane():
    global pub_follow_car, pub_cruise, pub_change_lane, pub_action, pub_stop, right_lane
    msg = TwoBool()
    msg.start = True
    msg.right_lane = right_lane
    pub_cruise.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(False)
    pub_change_lane.publish(msg)
    msg_finished = rospy.wait_for_message('/change_lane/finished', Empty, timeout=200.0)
        
def stop_motion():
    global pub_follow_car, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, right_lane
    pub_cruise.publish(False)
    pub_follow_car.publish(False)
    pub_stop.publish(True)    
    msg = TwoBool()
    msg.start = False
    msg.right_lane = right_lane 
    pub_change_lane.publish(msg)
    
def main():
    global obstacle_north, obstacle_north_west, obstacle_west, obstacle_south_west, obstacle_north_east, obstacle_east, obstacle_south_east,  success, right_lane
    global pub_follow_car, pub_cruise, pub_change_lane, pub_action, pub_stop
    print("INITIALIZING POLICY...", flush=True)
    rospy.init_node("policy")
    rospy.Subscriber("/obstacle/north"     , Bool, callback_obstacle_north)
    rospy.Subscriber("/obstacle/north_west", Bool, callback_obstacle_north_west)
    rospy.Subscriber("/obstacle/west"      , Bool, callback_obstacle_west)
    rospy.Subscriber("/obstacle/south_west", Bool, callback_obstacle_south_west)
    rospy.Subscriber("/obstacle/north_east"      , Bool, callback_obstacle_north_east)        
    rospy.Subscriber("/obstacle/east"      , Bool, callback_obstacle_east)    
    rospy.Subscriber("/obstacle/south_east"      , Bool, callback_obstacle_south_east)    
    rospy.Subscriber("/success", Bool, callback_success) 
    rospy.Subscriber("/right_lane", Bool, callback_right_lane)
       
    pub_policy_started  = rospy.Publisher("/policy_started", Empty, queue_size=10)
    pub_cruise = rospy.Publisher("/cruise/enable", Bool, queue_size=10)
    pub_follow_car    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_change_lane = rospy.Publisher("/change_lane/start", TwoBool, queue_size=10)    
    pub_action = rospy.Publisher("/action", String, queue_size=10)
    pub_stop = rospy.Publisher("/stop", Bool, queue_size=10)    
    rate = rospy.Rate(10)

    obstacle_north      = False
    obstacle_north_west = False
    obstacle_west       = False
    obstacle_south_west = False
    obstacle_north_east = False
    obstacle_east       = False                
    obstacle_west_east  = False
    success             = True
    right_lane          = True

    action = "NA"
    action_prev = "NA"            
    while not rospy.is_shutdown():
        pub_policy_started.publish()
        
        if success:
           # right lane
           if right_lane:
              if obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Keep distance 1"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Cruise 2"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Keep distance 3"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_west and obstacle_south_west and obstacle_west:
                 action = "Right Cruise 4"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Keep distance 5"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Cruise 6"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Keep distance 7"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and obstacle_west:
                 action = "Right Cruise 8"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Keep distance 9"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Cruise 10"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Change lane 11"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()
              elif not obstacle_north and not obstacle_north_west and obstacle_south_west and not obstacle_west:
                 action = "Right Cruise 12"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Keep distance 13"
              if action_prev != action:
                 print(action, flush = True)                      
                 action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Cruise 14"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Change lane 15"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()
              elif not obstacle_north and not obstacle_north_west and not obstacle_south_west and not obstacle_west:
                 action = "Right Cruise 16"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
           # left lane      
           elif not right_lane:
              if obstacle_north and obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Keep distance 1"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Cruise 2"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Keep distance 3"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_east and obstacle_south_east and obstacle_east:
                 action = "Left Cruise 4"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Keep distance 5"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Cruise 6"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Keep distance 7"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and not obstacle_north_east and not obstacle_south_east and obstacle_east:
                 action = "Left Cruise 8"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Keep distance 9"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Cruise 10"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Change lane 11"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()
              elif not obstacle_north and not obstacle_north_east and obstacle_south_east and not obstacle_east:
                 action = "Left Cruise 12"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Keep distance 13"
              if action_prev != action:
                 print(action, flush = True)                      
                 action_prev = action                
                 enable_follow_car()
              elif not obstacle_north and obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Cruise 14"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()
              elif obstacle_north and not obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Change lane 15"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()
              elif not obstacle_north and not obstacle_north_east and not obstacle_south_east and not obstacle_east:
                 action = "Left Cruise 16"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 enable_cruise()

        else: # success       
           action = "stop"
           stop_motion() 
                                              
        pub_action.publish(action)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

