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

def mysleep(secs):
    global curr_time

    init_time = curr_time        
    diff = 0.0
    while diff <= secs: # and not rospy.is_shutdown():
       diff  = curr_time - init_time
    #print("init_time", init_time, "curr_time", curr_time, "diff", diff)   
    
def callback_sim_time(msg):
    global curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs 
    curr_time = sim_secs + sim_nsecs / (10**9)        
    
def callback_start(msg):
    global start
    start = msg.data
           
def main():
    global start, right_lane, curr_time, pub_finish    
    global x, y , theta    
    
    start = False
    right_lane = False
    curr_time = 0.0
    
    print('INITIALIZING CHANGE_LANE NODE...', flush=True)
    rospy.init_node('change_lane')
    rate = rospy.Rate(10)

    rospy.Subscriber("/change_lane/start", Bool, callback_start)
    rospy.Subscriber("/clock", Clock, callback_sim_time)
    
    rospy.Subscriber("/current_pose", Pose2D, callback_current_pose) 
    rospy.Subscriber("/right_lane", Bool, callback_right_lane)       
        
    pub_speed  = rospy.Publisher('/speed', Float64, queue_size=2)
    pub_steering  = rospy.Publisher('/steering', Float64, queue_size=2)
    pub_finish = rospy.Publisher('/change_lane/finished', Empty, queue_size=2)

    start = False
    while not rospy.is_shutdown():
              
        if start:
                          
           print("change_lane: start change_lane ", curr_time, flush=True) 
           pub_speed.publish(3) #m/s
           if right_lane:  
              print("Giro a la izquierda", right_lane)
              pub_steering.publish(0.2)
              print("Publish 0.25", curr_time, "y pos", y, flush=True)
              mysleep(0.5)
           else:
              print("Giro a la derecha", right_lane)
              pub_steering.publish(-0.2)
              mysleep(0.50)           
           pub_finish.publish()           
              
           print("change_lane: finish change_lane ", curr_time, flush=True)                               
           start = False                           
                           
        else:
            continue

        #rate.sleep()            
        mysleep(0.1)

   

if __name__ == "__main__":
    main()

    

