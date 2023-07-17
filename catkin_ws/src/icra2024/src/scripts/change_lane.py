#!/usr/bin/env python3
"""
This node implements change lane from right to left.
"""
import rospy
from std_msgs.msg import Float64, Empty, Bool

from rosgraph_msgs.msg import Clock 
from geometry_msgs.msg import Pose2D

def callback_current_pose(msg):
    global x, y, theta   

    x = msg.x
    y = msg.y
    z = msg.theta
 
def callback_right_lane(msg):
    global right_lane
    right_lane = msg.data  

# Constants
MAX_TIME_CHANGE_LANE = 0.50
nSLEEP = 0.03 # 30000000 nsecs = 30 msecs    

def callback_start(msg):
    global start
    start = msg.data
        
def callback_sim_time(msg):
    global sim_secs, sim_nsecs, curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs 
    curr_time = sim_secs + sim_nsecs / (10**9)    
    
def main():
    global start, right_lane, sim_secs, sim_nsecs, curr_time, pub_finish    
    global x, y , theta    
    
    first_time = True
    start = False
    right_lane = False    

    sim_secs = 0.0
    sim_nsecs = 0.0                    
    curr_time = 0.0
    elapsed_time = 0.0
    start_time = 0.0
    
    print('INITIALIZING CHANGE_LANE NODE...', flush=True)
    rospy.init_node('change_lane')

    rospy.Subscriber("/change_lane/start", Bool, callback_start)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
    
    rospy.Subscriber("/current_pose", Pose2D, callback_current_pose) 
    rospy.Subscriber("/right_lane", Bool, callback_right_lane)       
        
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle  = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_finish = rospy.Publisher('/change_lane/finished', Empty, queue_size=10)
     
    rate = rospy.Rate(10)

    start = False
    while not rospy.is_shutdown():
              
        if start:
                          
            if first_time: 
               print("change_lane: start change_lane ", curr_time, flush=True) 
               start_time = curr_time
               first_time = False
            if right_lane:    
               pub_angle.publish(0.2)
               print("Publish 0.2", curr_time, "y pos", y, flush=True)
            else:
               pub_angle.publish(-0.2)   
               print("Publish -0.2", curr_time, "y pos", y, flush=True)
                                                
            elapsed_time = curr_time - start_time
            if elapsed_time >= MAX_TIME_CHANGE_LANE:
               print("Elapsed_time", elapsed_time, flush=True)           
               print("change_lane: finish change_lane ", curr_time, flush=True)
               pub_angle.publish(0.0)
               pub_finish.publish()                
               first_time = True
               start = False
               
        else:
            continue

        rate.sleep()            
        # My sleep
        '''
        i = 0
        while i < 1 and not rospy.is_shutdown():
            prev_sleep = sim_secs + sim_nsecs / (10**9)
            i = i + 1         
        diff = 0.0
        while diff <= nSLEEP and not rospy.is_shutdown():
            curr_time = sim_secs + sim_nsecs / (10**9)            
            diff  = curr_time - prev_sleep
        '''            
   

if __name__ == "__main__":
    main()

    

