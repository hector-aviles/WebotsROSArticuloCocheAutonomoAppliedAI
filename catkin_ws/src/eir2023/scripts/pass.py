#!/usr/bin/env python3
"""
This node implements an open loop set of movements to pass a vehicle.
(Actually, it only changes lane)
"""
import cv2
import numpy
import rospy
from rosgraph_msgs.msg import Clock 
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool

def callback_start_passing(msg):
    global start_passing
    start_passing = True

def main():
    global start_passing  
    turning_left_time = 5.6
    turning_right_time = 5.6
    print('INITIALIZING PASS-BEHAVIOR NODE...')
    rospy.init_node('passsing')
    #sim_speed_multiplier = 10  
    pub_sim_time = rospy.Publisher('/sim_time', Clock, queue_size=1)
    rate = rospy.Rate(30)
    
    rospy.Subscriber("/passing/start", Bool, callback_start_passing)
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/passing/finished', Empty, queue_size=10)
    print("Passing parameters:")
    print("Turning_left_time: " + str(turning_left_time))
    print("Turning_right_time: " + str(turning_right_time))
    start_passing = False
    sim_clock = Clock()
    now= rospy.get_time()
            
    while not rospy.is_shutdown(): 
        sim_clock.clock = rospy.Time.from_sec(rospy.get_time()- now)
       #rospy.loginfo(sim_clock)
       # pub_sim_time.publish(sim_clock)    
         
        if start_passing:         
            start_passing = False
            print("Passing: moving left")
            pub_speed.publish(36.0)
            pub_angle.publish(0.2)
            rospy.sleep(1.5)
            
            '''
            print("Passing: moving right")  
            pub_speed.publish(36.0)
            pub_angle.publish(-0.2)
            rospy.sleep(turning_right_time)
            print("Passing: finished")
            pub_angle.publish(0.0)
            pub_finish.publish()
            '''
            print("Passing: finished")
            
        rate.sleep()
    
       
if __name__ == "__main__":    
    #try:
    main()
    #except rospy.ROSInterruptException:
     #   pass

    

