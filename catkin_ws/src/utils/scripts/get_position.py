#!/usr/bin/env python3
"""
This node is a logger and records the following data:

"""
import rospy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose2D

  
def callback_current_position(msg):
    global x
    global y
    global theta  
    
    x = msg.x
    y = msg.y
    theta = msg.theta

   
def main():
        
    global x
    global y
    global theta      
    x = 0.0
    y = 0.0
    theta = 0.0

    print("INITIALIZING LOGGER...")
    rospy.init_node("get_position")
    
    rate = rospy.Rate(10)
    
    rospy.Subscriber("/current_pose", Pose2D, callback_current_position)
    
    while not rospy.is_shutdown():
            
        print("Position x:{} , y:{}, theta:{}".format( x, y, theta))
        rate.sleep()

        
             
if __name__ == "__main__":
    try:
        main()
                        
    except:
        rospy.ROSInterruptException
        pass

    
