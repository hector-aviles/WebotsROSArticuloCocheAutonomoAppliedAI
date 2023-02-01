#!/usr/bin/env python3
"""
This node implements a proportional control and is intended to be used
together with the lane_detector node. It is assumed both lane borders 
are given by two straight lines in rho-theta form. Given a desired rho-theta
for each lane border, an error is calculated.
Steering is calculated proportional to this error and linear speed is
set as a constant. 
"""
import cv2
import numpy
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool

#
# Steering is calculated proportional to two errors: distance error and angle error.
# These errors correspond to differences between an observed line (in normal form)
# and a desired line.
# Speed is calculated as the max speed minus a speed proportional to the steering.
# In this way, car goes with lower speed in curves and at max speed in straight roads. 
#
def calculate_control(rho_l, theta_l, rho_r, theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r, dist=None):
    global max_speed, k_rho, k_theta
    error_rho_l   = goal_rho_l   - rho_l
    error_theta_l = goal_theta_l - theta_l
    error_rho_r   = rho_r   - goal_rho_r
    error_theta_r = theta_r - goal_theta_r
    if rho_l != 0 and rho_r != 0:
        error_rho   = (error_rho_l + error_rho_r)/2
        error_theta = (error_theta_l + error_theta_r)/2
    elif rho_l != 0:
        error_rho   = error_rho_l
        error_theta = error_theta_l
    else:
        error_rho   = error_rho_r
        error_theta = error_theta_r
    
    steering = -k_rho*error_rho - k_theta*error_theta
    if dist is None:
        speed = max_speed*(1 - 1.5*abs(steering))
    else:
        speed = 36 + 1.5*(dist - 20)
    return speed, steering

def callback_left_lane(msg):
    global lane_rho_l, lane_theta_l
    lane_rho_l, lane_theta_l = msg.data

def callback_right_lane(msg):
    global lane_rho_r, lane_theta_r
    lane_rho_r, lane_theta_r = msg.data

def callback_enable_steady_motion(msg):
    global enable_steady_motion
    enable_steady_motion = msg.data

def callback_enable_follow(msg):
    global enable_follow
    enable_follow = msg.data

def callback_dist_to_obstacle(msg):
    global dist_to_obstacle
    dist_to_obstacle = msg.data

def main():
    global lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r
    global max_speed, k_rho, k_theta
    global enable_steady_motion, enable_follow, dist_to_obstacle
    max_speed = 10
    k_rho   = 0.001
    k_theta = 0.01
    lane_rho_l   = 0
    lane_theta_l = 0
    lane_rho_r   = 0
    lane_theta_r = 0
    goal_rho_l   = 370.0
    goal_theta_l = 2.4
    goal_rho_r   = 430.0
    goal_theta_r = 0.895
    print('INITIALIZING LANE TRACKING NODE...')
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(30)
    if rospy.has_param('~max_speed'):
        max_speed = rospy.get_param('~max_speed')
    if rospy.has_param('~k_rho'):
        k_rho = rospy.get_param('~k_rho')
    if rospy.has_param('~k_theta'):
        k_theta = rospy.get_param('k_theta')

    print("Waiting for lane detection...")
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    rospy.Subscriber("/steady_motion/enable", Bool, callback_enable_steady_motion)
    rospy.Subscriber("/follow/enable", Bool, callback_enable_follow)
    rospy.Subscriber("/obstacle/distance", Float64, callback_dist_to_obstacle)
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=10)
    msg_left_lane  = rospy.wait_for_message('/demo/left_lane' , Float64MultiArray, timeout=100)
    msg_right_lane = rospy.wait_for_message('/demo/right_lane', Float64MultiArray, timeout=100)
    print("Using:")
    print("Max speed: " + str(max_speed))
    print("K_rho: " + str(k_rho))
    print("K_theta: " + str(k_theta))
    enable_steady_motion = False
    enable_follow        = False
    dist_to_obstacle     = 9.0

    while not rospy.is_shutdown():
        if enable_steady_motion:
            #print ("enable_steady_motion is true", flush = True) 
            #print (enable_steady_motion, flush = True)             
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r)
        elif enable_follow:
            #print ("enable_follow is true", flush = True)            
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r, dist_to_obstacle)
        else:
            continue
        #print ("Lane tracking envia mensaje", flush = True)    
        pub_speed.publish(speed)
        pub_angle.publish(steering)
        rate.sleep()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

