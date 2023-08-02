#!/usr/bin/env python3
"""
This node implements several behaviors:
* Lane tracking using a proportional control (inteded to be used together with the lane_detector node)
* Car following using also a proportional control (to be used together with the obstacle detector node)
* Change lane, using finite state machine and assuming we know the car position
* Pass, same conditions as the change lane behavior.  
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
    global max_speed, k_rho, k_theta, k_following, dist_to_car
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
        speed = max_speed + k_following*(dist - dist_to_car)
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
    
def main():
    global lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r
    global max_speed, k_rho, k_theta, k_following, dist_to_car
    global enable_cruise, enable_follow, dist_to_obstacle
    
    max_speed = 10      #Maximum speed for following and steady motion behaviors
    k_rho   = 0.001     #Gain for rho error in lane tracking
    k_theta = 0.01      #Gain for theta error in lane tracking
    k_following = 1.5
    dist_to_car = 20
    lane_rho_l   = 0
    lane_theta_l = 0
    lane_rho_r   = 0
    lane_theta_r = 0
    goal_rho_l   = 370.0
    goal_theta_l = 2.4
    goal_rho_r   = 430.0
    goal_theta_r = 0.895
    
    print('INITIALIZING LANE TRACKING NODE...', flush=True)
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(10)
    if rospy.has_param('~max_speed'):
        max_speed = rospy.get_param('~max_speed')
    if rospy.has_param('~k_rho'):
        k_rho = rospy.get_param('~k_rho')
    if rospy.has_param('~k_theta'):
        k_theta = rospy.get_param('k_theta')
    if rospy.has_param('~k_following'):
        k_following = rospy.get_param('~k_following')
    if rospy.has_param('~dist_to_car'):
        dist_to_car = rospy.get_param('~dist_to_car')

    print("Waiting for lane detection...")
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    rospy.Subscriber("/cruise/enable", Bool, callback_enable_cruise)
    rospy.Subscriber("/follow/enable", Bool, callback_enable_follow)
    rospy.Subscriber("/start_change_lane", Bool, callback_start_change_lane)
    rospy.Subscriber("/start_pass", Bool, callback_start_pass)
    rospy.Subscriber("/obstacle/distance", Float64, callback_dist_to_obstacle)
          
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=1)
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=1)
    pub_change_lane_finshed = rospy.Publisher('/change_lane_finished', Bool, queue_size=1)
    pub_pass_finished = rospy.Publisher('/pass_finished', Bool, queue_size=1)
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
        
        rate.sleep()

    

if __name__ == "__main__":
    main()

    

