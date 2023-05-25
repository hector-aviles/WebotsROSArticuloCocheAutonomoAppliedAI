#!/usr/bin/env python3
"""
This node implements an open loop set of movements to pass a vehicle.
(Actually, it only changes lane)
"""
import cv2
import numpy
import rospy
from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool, String

def callback_start_passing(msg):
    global start_passing
    global right_lane
    start_passing = True
    right_lane = True
    
def callback_current_pose(msg):
    # Access the components of the Pose2D message
    global x
    global y
    global theta 
    x = msg.x
    y = msg.y
    theta = msg.theta

    #rospy.loginfo("Received Pose2D - x: %f, y: %f, theta: %f", x, y, theta)

def callback_sim_time(msg):
    global sim_secs    
    global sim_nsecs            
    current_time = msg
    sim_secs = current_time.clock.secs 
    sim_nsecs = current_time.clock.nsecs 
    #print("secs:{} , nsecs:{}".format(sim_secs, sim_nsecs))
   
def main():
    global start_passing   
    turning_left_time = 5.6
    turning_right_time = 5.6
    sleep_duration = rospy.Duration.from_sec(1.0)
    print('INITIALIZING PASS-BEHAVIOR NODE...')
    rospy.init_node('passsing')
    
    rate = rospy.Rate(30)
    
    rospy.Subscriber("/passing/start", Bool, callback_start_passing)
    rospy.Subscriber("/current_pose", Pose2D, callback_current_pose)
    rospy.Subscriber("/clock", Clock, callback_sim_time) 
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/passing/finished', Empty, queue_size=10)
    print("Passing parameters:")
    print("Turning_left_time: " + str(turning_left_time))
    print("Turning_right_time: " + str(turning_right_time))
    start_passing = False
    right_lane = True
            
    while not rospy.is_shutdown():
  
        if start_passing and right_lane:                        
            print("Passing: moving left")
            pub_speed.publish(36.0)
            pub_angle.publish(0.2)
            #rospy.sleep(sleep_duration)
            if theta >= 0.2:
               pub_angle.publish(-0.2)
               print("A")
               start_passing = False   
               right_lane = False  
            print("Passing: moving right")
            '''
            pub_speed.publish(36.0)
            pub_angle.publish(-0.2)
            rospy.sleep(turning_right_time)
            print("Passing: finished")
            pub_angle.publish(0.0)
            pub_finish.publish()
            '''
        rate.sleep()
    
       
if __name__ == "__main__":    
    #try:
    main()
    #except rospy.ROSInterruptException:
     #   pass

    

