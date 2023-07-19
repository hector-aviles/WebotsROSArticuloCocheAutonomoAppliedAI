#!/usr/bin/env python3
"""
This node determines if the car is at the right or left lane of the road. 
"""
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from rosgraph_msgs.msg import Clock 

def mysleep(secs):
    global curr_time

    init_time = curr_time        
    diff = 0.0
    while diff <= secs: # and not rospy.is_shutdown():
       diff  = curr_time - init_time
    #print("init_time", init_time, "curr_time", curr_time, "diff", diff) 
    
def callback_sim_time(msg):
    global sim_secs, sim_nsecs, curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs 
    curr_time = sim_secs + sim_nsecs / (10**9)        

def callback_current_pose(msg):
    global pub_right_lane, right_lane, x, y, theta # , prev_right_lane   

    x = msg.x
    y = msg.y
    z = msg.theta
 
    if y > 0:
       right_lane =  False
    else:
       right_lane = True       

    #if prev_right_lane != right_lane: 
       #print("Position in y ", y, "right_lane", right_lane, "prev_right_lane", prev_right_lane,  flush=True)
       #prev_right_lane = right_lane

    pub_right_lane.publish(right_lane)


def main():
    global pub_right_lane, right_lane, x, y, theta
    global curr_time
    #global prev_right_lane
    
    x = 0.0
    y = 0.0
    theta = 0.0
    right_lane = True
    curr_time = 0.0    
    #prev_right_lane = True
    
    print("INITIALIZING LANE NODE...", flush=True)
    rospy.init_node("lane")
    rate = rospy.Rate(10)

    rospy.Subscriber("/current_pose", Pose2D, callback_current_pose)
    #rospy.Subscriber("/clock", Clock, callback_sim_time)    
    
    pub_right_lane  = rospy.Publisher("/right_lane", Bool, queue_size=2)

    while not rospy.is_shutdown():

        #mysleep(0.05)        
        #print("In right lane ", right_lane, flush=True)
        rate.sleep()
   

if __name__ == "__main__":
    main()

'''
if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
'''    

