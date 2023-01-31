#!/usr/bin/env python3
"""
This node executes cruise action only to induce a car crash 
"""
import rospy
from std_msgs.msg import Float64MultiArray, Empty, Bool, String


def enable_steady_motion():
    #global pub_steady_motion, pub_follow_car, pub_action
    pub_steady_motion.publish(True)
    pub_follow_car.publish(False)
    pub_action.publish(action)

def main():
    global pub_steady_motion, pub_follow_car, pub_action, action
    print("INITIALIZING STEADY MOTION...")
    rospy.init_node("cruise_action")
    pub_start_signal  = rospy.Publisher("/start", Empty, queue_size=10)
    pub_steady_motion = rospy.Publisher("/steady_motion/enable", Bool, queue_size=10)
    pub_follow_car    = rospy.Publisher("/follow/enable", Bool, queue_size=10)
    pub_action = rospy.Publisher("/action", String, queue_size=10)
    rate = rospy.Rate(10)

    
    while not rospy.is_shutdown():
        pub_start_signal.publish()
        
        action = "Cruise"
        enable_steady_motion()

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

