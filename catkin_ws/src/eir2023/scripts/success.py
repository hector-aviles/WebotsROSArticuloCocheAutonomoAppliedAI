#!/usr/bin/env python3
"""
This node detects if the car succeeded its last action 
"""
import math
import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu

def callback_accelerometer(msg):

    global success, first_time, prev_x, prev_y, prev_z

    diff = 0
    if success:
       if first_time:
          first_time = False   
       else:
          p = [prev_x, prev_y, prev_z]
          q = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
          diff = math.dist(p, q)
          if diff < 5:
             success = True
          else:    
             success = True
       prev_x = msg.linear_acceleration.x
       prev_y = msg.linear_acceleration.y
       prev_z = msg.linear_acceleration.z     

    pub_success.publish(success)
    pub_accel_diff.publish(diff)

def main():
    global success, first_time, prev_x, prev_y, prev_z, pub_success, pub_accel_diff  
    first_time = True
    success = True
    prev_x = 0
    prev_y = 0
    prev_z = 0
    
    print("INITIALIZING SUCCESS...")
    rospy.init_node("success")
    rospy.Subscriber('/accelerometer', Imu, callback_accelerometer)
    pub_success  = rospy.Publisher("/success", Bool, queue_size=10)
    pub_accel_diff  = rospy.Publisher("/accelerometer_diff", Float64, queue_size=10)    
    rate = rospy.Rate(10)
    
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

