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

def callback_enable_cruise(msg):
    global enable_cruise
    enable_cruise = msg.data

def callback_enable_follow(msg):
    global enable_follow
    enable_follow = msg.data

def callback_dist_to_obstacle(msg):
    global dist_to_obstacle
    dist_to_obstacle = msg.data
    
def callback_requested_speed(msg):
    global requested_speed
    requested_speed = msg.data    

def main():
    global lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r
    global max_speed, k_rho, k_theta
    global enable_cruise, enable_follow, dist_to_obstacle
    global curr_time
    
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
    requested_speed = 0.0
    curr_time = 0.0
    
    print('INITIALIZING LANE TRACKING NODE...', flush=True)
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(10)
    if rospy.has_param('~max_speed'):
        max_speed = rospy.get_param('~max_speed')
    if rospy.has_param('~k_rho'):
        k_rho = rospy.get_param('~k_rho')
    if rospy.has_param('~k_theta'):
        k_theta = rospy.get_param('k_theta')

    print("Waiting for lane detection...")
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    rospy.Subscriber("/cruise/enable", Bool, callback_enable_cruise)
    rospy.Subscriber("/follow/enable", Bool, callback_enable_follow)
    rospy.Subscriber("/obstacle/distance", Float64, callback_dist_to_obstacle)
    #rospy.Subscriber("/clock", Clock, callback_sim_time)
          
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=2)
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=2)
    msg_left_lane  = rospy.wait_for_message('/demo/left_lane' , Float64MultiArray, timeout=100)
    msg_right_lane = rospy.wait_for_message('/demo/right_lane', Float64MultiArray, timeout=100)
    print("Using:")
    print("Max speed: " + str(max_speed))
    print("K_rho: " + str(k_rho))
    print("K_theta: " + str(k_theta))
    enable_cruise = False
    enable_follow        = False
    dist_to_obstacle     = 9.0

    while not rospy.is_shutdown():
        if enable_cruise:
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r,
                                                goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r)
        elif enable_follow:
            speed, steering = calculate_control(lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r,
                                                goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r, dist_to_obstacle)
        else:
            continue

        pub_speed.publish(speed)
        pub_angle.publish(steering)
        #mysleep(0.05)
        
        rate.sleep()

    

if __name__ == "__main__":
    main()

    

