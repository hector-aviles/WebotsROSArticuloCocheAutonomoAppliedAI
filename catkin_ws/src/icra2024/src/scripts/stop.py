#!/usr/bin/env python3
"""
This node stop the car by killing the next nodes:

policy
behaviours

and sets the car speed to 0
  
"""
import os
import rospy
from std_msgs.msg import Float64, Empty, Bool
   
def callback_success(msg):
    global success
    success = msg.data    
    
def stop_motion():
    global pub_stop
    pub_stop.publish(True)        

def main():
    global success
            
    print('INITIALIZING STOP NODE...', flush=True)
    rospy.init_node('stop')
    rate = rospy.Rate(10)

    rospy.Subscriber("/success", Bool, callback_success) 
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=1)

    success = True
    while not rospy.is_shutdown():
        if not success:
            print("STOP: Starting system shutdown...",  flush = True)  

            os.system("rosnode kill /behaviors")
            os.system("rosnode kill /current_lane")
            os.system("rosnode kill /lane_detector")
            os.system("rosnode kill /obstacle_detector")
            os.system("rosnode kill /success")            
            os.system("rosnode kill /policy")
            os.system("rosnode kill /logger")

            print("STOP: Finishing system shutdown... ", end="", flush = True)

            speed = 0.0  
            pub_speed.publish(speed)
            
            print("Done", flush = True)             

            os.system("rosnode kill /stop")            
        else:
            continue

        rate.sleep()
    

if __name__ == "__main__":
    main()

    

