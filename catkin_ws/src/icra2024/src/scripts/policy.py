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
from icra2024.msg import ChangeLaneMSG

def callback_free_N(msg):
    global free_N
    free_N = msg.data

def callback_free_NW(msg):
    global free_NW
    free_NW = msg.data

def callback_free_W(msg):
    global free_W
    free_W = msg.data
    
def callback_sW1(msg):
    global sW1
    sW1 = msg.data
    
def callback_sW2(msg):
    global sW2
    sW2 = msg.data    

def callback_free_NE(msg):
    global free_NE
    free_NE = msg.data    

def callback_free_E(msg):
    global free_E
    free_E = msg.data    

def callback_sE1(msg):
    global sE1
    sE1 = msg.data    
    
def callback_sE2(msg):
    global sE2
    sE2 = msg.data        

def callback_success(msg):
    global success
    success = msg.data
    
def callback_right_lane(msg):
    global right_lane
    right_lane = msg.data        

def cruise():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, right_lane
    pub_cruise.publish(True)
    pub_keep_distance.publish(False)
    pub_stop.publish(False)
    msg = ChangeLaneMSG()
    msg.start = False
    msg.right_lane = right_lane
    pub_change_lane.publish(msg)

def keep_distance():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, right_lane
    pub_keep_distance.publish(True)
    pub_cruise.publish(False)
    pub_stop.publish(False)
    msg = ChangeLaneMSG()
    msg.start = False
    msg.right_lane = right_lane 
    pub_change_lane.publish(msg)
    

def change_lane():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, right_lane
    msg = ChangeLaneMSG()
    msg.start = True
    msg.right_lane = right_lane
    pub_cruise.publish(False)
    pub_keep_distance.publish(False)
    pub_stop.publish(False)
    pub_change_lane.publish(msg)
    msg_finished = rospy.wait_for_message('/change_lane/finished', Empty, timeout=200.0)
        
def stop():
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop, pub_change_lane, right_lane
    pub_cruise.publish(False)
    pub_keep_distance.publish(False)
    pub_stop.publish(True)    
    msg = ChangeLaneMSG()
    msg.start = False
    msg.right_lane = right_lane 
    pub_change_lane.publish(msg)
    
def main():
    global free_N, free_NW, free_W, sW1, sW2, free_NE, free_E, sE1, sE2, success, right_lane
    global pub_keep_distance, pub_cruise, pub_change_lane, pub_action, pub_stop
    print("INITIALIZING POLICY...", flush=True)
    rospy.init_node("policy")
    rospy.Subscriber("/obstacle/north"     , Bool, callback_free_N)
    rospy.Subscriber("/obstacle/north_west", Bool, callback_free_NW)
    rospy.Subscriber("/obstacle/west"      , Bool, callback_free_W)
    rospy.Subscriber("/obstacle/sW1", Bool, callback_sW1)
    rospy.Subscriber("/obstacle/sW2", Bool, callback_sW2)    
    rospy.Subscriber("/obstacle/north_east"      , Bool, callback_free_NE)        
    rospy.Subscriber("/obstacle/east"      , Bool, callback_free_E)    
    rospy.Subscriber("/obstacle/sE1"      , Bool, callback_sE1)
    rospy.Subscriber("/obstacle/sE2"      , Bool, callback_sE2)    
    rospy.Subscriber("/success", Bool, callback_success) 
    rospy.Subscriber("/right_lane", Bool, callback_right_lane)
       
    pub_policy_started  = rospy.Publisher("/policy_started", Empty, queue_size=10)
    pub_cruise = rospy.Publisher("/cruise/enable", Bool, queue_size=10)
    pub_keep_distance    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_change_lane = rospy.Publisher("/change_lane/start", ChangeLaneMSG, queue_size=10)    
    pub_action = rospy.Publisher("/action", String, queue_size=10)
    pub_stop = rospy.Publisher("/stop", Bool, queue_size=10)    
    rate = rospy.Rate(10)

    free_N  = True
    free_NW = True
    free_W  = True
    sW1 = True
    sW2 = True    
    free_NE = True
    free_E  = True                
    sE1 = True
    sE2 = True    
    success = True
    right_lane = True

    action = "NA"
    action_prev = "NA"            
    while not rospy.is_shutdown():
        pub_policy_started.publish()
        
        if success:
           # right lane
           if right_lane:
              if not free_N and not free_NW and not free_W and not sW1 and not sW2:
                 action = "Right Keep distance 1"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and not free_W and not sW1 and not sW2:
                 action = "Right Cruise 2"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and not free_W and not sW1 and not sW2:
                 action = "Right Keep distance 3"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and not free_W and not sW1 and not sW2:
                 action = "Right Cruise 4"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and free_W and not sW1 and not sW2:
                 action = "Right Keep distance 5"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and free_W and not sW1 and not sW2:
                 action = "Right Cruise 6"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and free_W and not sW1 and not sW2:
                 action = "Right Keep distance 7"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and free_W and not sW1 and not sW2:
                 action = "Right Cruise 8"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and not free_W and sW1 and not sW2:
                 action = "Right Keep distance 9"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and not free_W and sW1 and not sW2:
                 action = "Right Cruise 10"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and not free_W and sW1 and not sW2:
                 action = "Right Keep distance 11"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and not free_W and sW1 and not sW2:
                 action = "Right Cruise 12"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and free_W and sW1 and not sW2:
                 action = "Right Keep distance 13"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and free_W and sW1 and not sW2:
                 action = "Right Cruise 14"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and free_W and sW1 and not sW2:
                 action = "Right Change lane 15"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif free_N and free_NW and free_W and sW1 and not sW2:
                 action = "Right Cruise 16"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and not free_W and not sW1 and sW2:
                 action = "Right Keep distance 17"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and not free_W and not sW1 and sW2:
                 action = "Right Cruise 18"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and not free_W and not sW1 and sW2:
                 action = "Right Keep distance 19"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and not free_W and not sW1 and sW2:
                 action = "Right Cruise 20"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and free_W and not sW1 and sW2:
                 action = "Right Keep distance 21"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and free_W and not sW1 and sW2:
                 action = "Right Cruise 22"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and free_W and not sW1 and sW2:
                 action = "Right Change lane 23"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif free_N and free_NW and free_W and not sW1 and sW2:
                 action = "Right Cruise 24"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and not free_NW and not free_W and sW1 and sW2:
                 action = "Right Keep distance 25"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and not free_W and sW1 and sW2:
                 action = "Right Cruise 26"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_N and free_NW and not free_W and sW1 and sW2:
                 action = "Right Keep distance 27"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and free_NW and not free_W and sW1 and sW2:
                 action = "Right Cruise 28"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()
              elif not free_N and not free_NW and free_W and sW1 and sW2:
                 action = "Right Keep distance 29"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_N and not free_NW and free_W and sW1 and sW2:
                 action = "Right Cruise 30"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 Cruise()

              elif not free_N and free_NW and free_W and sW1 and sW2:
                 action = "Right Change lane 31"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif free_N and free_NW and free_W and sW1 and sW2:
                 action = "Right Cruise 32"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()
           # left lane      
           elif not right_lane:
              if not free_E and not free_N and not free_NE and not sE1 and not sE2:
                 action = "Left Keep distance 1"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and not free_NE and not sE1 and not sE2:
                 action = "Left Keep distance 2"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif not free_E and free_N and not free_NE and not sE1 and not sE2:
                 action = "Left Cruise 3"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()
              elif free_E and free_N and not free_NE and not sE1 and not sE2:
                 action = "Left Cruise 4"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and free_NE and not sE1 and not sE2:
                 action = "Left Keep distance 5"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and free_NE and not sE1 and not sE2:
                 action = "Left Change lane 6"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif not free_E and free_N and free_NE and not sE1 and not sE2:
                 action = "Left Cruise 7"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 Cruise()

              elif free_E and free_N and free_NE and not sE1 and not sE2:
                 action = "Left Cruise 8"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and not free_NE and sE1 and not sE2:
                 action = "Left Keep distance 9"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and not free_NE and sE1 and not sE2:
                 action = "Left Keep distance 10"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif not free_E and free_N and not free_NE and sE1 and not sE2:
                 action = "Left Cruise 11"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif free_E and free_N and not free_NE and sE1 and not sE2:
                 action = "Left Cruise 12"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and free_NE and sE1 and not sE2:
                 action = "Left Keep distance 13"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and free_NE and sE1 and not sE2:
                 action = "Left Change lane 14"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif not free_E and free_N and free_NE and sE1 and not sE2:
                 action = "Left Cruise 15"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif free_E and free_N and free_NE and sE1 and not sE2:
                 action = "Left Change lane 16"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif not free_E and not free_N and not free_NE and not sE1 and sE2:
                 action = "Left Keep distance 17"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and not free_NE and not sE1 and sE2:
                 action = "Left Keep distance 18"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif not free_E and free_N and not free_NE and not sE1 and sE2:
                 action = "Left Cruise 19"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif free_E and free_N and not free_NE and not sE1 and sE2:
                 action = "Left Cruise 20"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and free_NE and not sE1 and sE2:
                 action = "Left Keep distance 21"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and free_NE and not sE1 and sE2:
                 action = "Left Change lane 22"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif not free_E and free_N and free_NE and not sE1 and sE2:
                 action = "Left Cruise 23"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif free_E and free_N and free_NE and not sE1 and sE2:
                 action = "Left Change lane 24"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif not free_E and not free_N and not free_NE and sE1 and sE2:
                 action = "Left Keep distance 25"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and not free_NE and sE1 and sE2:
                 action = "Left Keep distance 26"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif not free_E and free_N and not free_NE and sE1 and sE2:
                 action = "Left Cruise 27"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif free_E and free_N and not free_NE and sE1 and sE2:
                 action = "Left Cruise 28"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif not free_E and not free_N and free_NE and sE1 and sE2:
                 action = "Left Keep distance 29"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 keep_distance()

              elif free_E and not free_N and free_NE and sE1 and sE2:
                 action = "Left Change lane 30"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()

              elif not free_E and free_N and free_NE and sE1 and sE2:
                 action = "Left Cruise 31"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 cruise()

              elif free_E and free_N and free_NE and sE1 and sE2:
                 action = "Left Change lane 32"
                 if action_prev != action:
                    print(action, flush = True)                      
                    action_prev = action                
                 change_lane()
        else: # not success       
           action = "stop"
           stop() 
                                              
        pub_action.publish(action)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

