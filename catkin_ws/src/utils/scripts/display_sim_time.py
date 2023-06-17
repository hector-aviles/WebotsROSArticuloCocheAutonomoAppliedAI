#!/usr/bin/env python3
"""
This node is a logger and records the following data:

"""
import rospy
from rosgraph_msgs.msg import Clock

def callback_simulated_time(msg):
    global sim_secs    
    global sim_nsecs        

    sim_time = msg
    sim_secs = sim_time.clock.secs 
    #print (sim_secs)
    sim_nsecs = sim_time.clock.nsecs     
    #print (sim_nsecs)
    
def main():
        
    global sim_secs    
    global sim_nsecs        
    sim_secs = 0.0
    sim_nsecs = 0.0

    print("INITIALIZING LOGGER...")
    rospy.init_node("display_sim_time")
    
    rate = rospy.Rate(10)

    rospy.Subscriber("/clock", Clock, callback_simulated_time) 
    
    while not rospy.is_shutdown():
            
        print ("secs and nsecs ", sim_secs, sim_nsecs)
        rate.sleep()

        
             
if __name__ == "__main__":
    try:
        main()
                        
    except:
        rospy.ROSInterruptException
        pass

    
