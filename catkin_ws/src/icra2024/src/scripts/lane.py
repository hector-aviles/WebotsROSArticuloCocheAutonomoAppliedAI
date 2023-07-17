#!/usr/bin/env python3
"""
This node determines if the car is at the right or left lane of the road. 
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

def callback_current_pose(msg):
    global pub_right_lane, right_lane, prev_right_lane, x, y, theta   

    x = msg.x
    y = msg.y
    z = msg.theta
 
    if y > 0:
       right_lane =  False
    else:
       right_lane = True       

    if prev_right_lane != right_lane: 
       print("Position in y ", y, "right_lane", right_lane, "prev_right_lane", prev_right_lane,  flush=True)
       prev_right_lane = right_lane

    pub_right_lane.publish(right_lane)


def main():
    global pub_right_lane, right_lane, x, y, theta
    global prev_right_lane
    
    x = 0.0
    y = 0.0
    theta = 0.0
    right_lane = True
    prev_right_lane = True
    
    print("INITIALIZING LANE NODE...", flush=True)
    rospy.init_node("lane")
    rate = rospy.Rate(1)

    rospy.Subscriber("/current_pose", Pose2D, callback_current_pose)
    pub_right_lane  = rospy.Publisher("/right_lane", Bool, queue_size=2)

    while not rospy.is_shutdown():
    
        #print("In right lane ", right_lane, flush=True)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

