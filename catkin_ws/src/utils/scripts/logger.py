#!/usr/bin/env python3
"""
This node is a logger and records the following data:

"""
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool, String
from sensor_msgs.msg import Imu
from pathlib import Path
import os.path

def callback_accelerometer(msg):

    global accel_x, accel_y, accel_z

    accel_x = msg.linear_acceleration.x
    accel_y = msg.linear_acceleration.y
    accel_z = msg.linear_acceleration.z    

def callback_accel_diff(msg):
    global accel_diff
    accel_diff = msg.data
    
def callback_action(msg):
    global action
    action = msg.data        

def callback_change_lane_finished(msg):
    global change_lane_finished
    change_lane_finished = msg.data
    
def callback_sim_time(msg):
    global curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs
    curr_time = sim_secs + sim_nsecs / (10**9)    

def callback_curr_lane(msg):
    global curr_lane
    curr_lane = msg.data    
    
def callback_cruise_enable(msg):
    global cruise_enable
    cruise_enable = msg.data        
    
def callback_left_lane(msg):
    global lane_rho_l, lane_theta_l
    lane_rho_l, lane_theta_l = msg.data

def callback_right_lane(msg):
    global lane_rho_r, lane_theta_r
    lane_rho_r, lane_theta_r = msg.data     
    
def callback_follow_enable(msg):
    global follow_enable
    callback_follow_enable = msg.data

def callback_free_east(msg):
    global free_east
    free_east = msg.data
    
def callback_free_north(msg):
    global free_north
    free_north = msg.data
    
def callback_free_north_east(msg):
    global free_north_east
    free_north_east = msg.data    
    
def callback_free_north_west(msg):
    global free_north_west
    free_north_west = msg.data

def callback_free_south_east(msg):
    global free_south_east
    free_south_east = msg.data

def callback_free_south_west(msg):
    global free_south_west
    free_south_west = msg.data

def callback_free_west(msg):
    global free_west
    free_west = msg.data

def callback_obstacle_distance(msg):
    global obstacle_distance
    obstacle_distance = msg.data

def callback_pass_finished(msg):
    global pass_finished
    callback_pass_finished = msg.data   
    
def callback_policy_started(msg):
    global policy_started
    callback_policy_started = msg.data            

def callback_curr_pose(msg):
    global curr_x, curr_y, curr_theta   

    curr_x = msg.x
    curr_y = msg.y
    curr_theta = msg.theta
        
def callback_speed(msg):
    global speed
    speed = msg.data 
    
def callback_start_change_lane_on_left(msg):
    global start_change_lane_on_left
    start_change_lane_on_left = msg.data

def callback_start_change_lane_on_right(msg):
    global start_change_lane_on_right
    start_change_lane_on_right = msg.data
    
def callback_steering(msg):
    global steering
    steering = msg.data      

def callback_success(msg):
    global success
    success = msg.data
       
           
def main():
    global cruise_enable
    global accel_x, accel_y, accel_z, accel_diff, action, change_lane_finished, curr_time, curr_lane, lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, follow_enable, free_east, free_north, free_north_east, free_north_west, free_south_east, free_south_west, free_west, obstacle_distance, pass_finished, policy_started, curr_x, curr_y, curr_theta, speed, start_change_lane_on_left, start_change_lane_on_right, steering, success
    
    trial_number = 0
    free_north      = False
    free_north_west = False
    free_west       = False
    free_east       = False    
    free_south_west = False
    obstacle_distance = 100000
    follow_enable = False 
    cruise_enable = False 
    passing_start = False 
    passing_finished = False  
    speed = 0.0
    steering = 0.0
    accel_x = 0.0 
    accel_y = 0.0 
    accel_z = 0.0 
    accel_diff = 0.0 
    success = True
    action = ""
    
        
    num_trials_file = "trial_number.data"

    print("INITIALIZING LOGGER...")
    rospy.init_node("logger")
    
    rospy.Subscriber('/accelerometer', Imu, callback_accelerometer)    
    rospy.Subscriber("/accelerometer_diff", Float64, callback_accel_diff)
    rospy.Subscriber("/action", String, callback_action)    
    rospy.Subscriber("/change_lane_finished", Bool, callback_change_lane_finished)
    rospy.Subscriber("/clock", Clock, callback_sim_time) 
    rospy.Subscriber("/cruise/enable", Bool, callback_cruise_enable)
    rospy.Subscriber("/current_lane", Bool, callback_curr_lane)
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    rospy.Subscriber("/follow/enable", Bool, callback_follow_enable)    
    rospy.Subscriber("/free/east"     , Bool, callback_free_east)
    rospy.Subscriber("/free/north"     , Bool, callback_free_north)
    rospy.Subscriber("/free/north_east"     , Bool, callback_free_north_east)
    rospy.Subscriber("/free/north_west", Bool, callback_free_north_west)
    rospy.Subscriber("/free/south_east", Bool, callback_free_south_east)
    rospy.Subscriber("/free/south_west", Bool, callback_free_south_west)
    rospy.Subscriber("/free/west"      , Bool, callback_free_west)
    rospy.Subscriber("/obstacle/distance"  , Float64, callback_obstacle_distance)
    rospy.Subscriber("/pass_finished", Bool, callback_pass_finished)
    rospy.Subscriber("/policy_started", Bool, callback_policy_started)    
    rospy.Subscriber("/self_driving_pose", Pose2D, callback_curr_pose) 
    rospy.Subscriber("/speed", Float64, callback_speed)
    rospy.Subscriber("/start_change_lane_on_left", Bool, callback_start_change_lane_on_left)
    rospy.Subscriber("/start_change_lane_on_right", Bool, callback_start_change_lane_on_right)
    rospy.Subscriber("/steering", Float64, callback_steering)
    rospy.Subscriber("/success", Bool, callback_success)    

    rate = rospy.Rate(10)

    # Lectura del número de repetición
    file_exists = os.path.exists(num_trials_file)
    if file_exists:
       c = open(num_trials_file, "r")
       repetition = c.read()
       c.close()
       trial_number = int(repetition)
    else:
       myfile = Path(num_trials_file)
       myfile.touch(exist_ok=True)
       trial_number = trial_number + 1 
       c = open(num_trials_file, "w")        
       c.write(str(trial_number))
       c.close()      
       
    str_repetition = str(trial_number)

    # Write header only once
    f = open("logfile.csv","a")
    if trial_number == 1:
       output = "repetition," +"iteration," + "time," + "speed," + "steering," + "free_North," + "free_NorthWest," + "free_SouthWest," + "free_West," + "free_East," +"free_distance," + "follow_enable," +  "cruise_enable," + "passing_start," + "accel_x," + "accel_y," + "accel_z," + "accel_diff," +"success," +  "action" + "\n"     
       f.write(output)
     
    print("Logger.->Waiting for start signal")
    rospy.wait_for_message("/policy_started", Empty, timeout=10000.0)
    print("Logger.->Start signal received")

    iteration = 0
    while not rospy.is_shutdown():
    
        iteration = iteration + 1
        now = rospy.get_time()
                       
        output = str(trial_number) + "," + str(iteration) +"," + str(now) + "," + str(speed) + "," + str(steering) + "," + str(free_north) + "," + str(free_north_west) + "," + str(free_south_west) + "," + str(free_west) + "," + str(free_east) + "," +  str(free_distance) + "," + str(follow_enable) +  "," + str(cruise_enable) + "," + str(passing_start) + "," +  str(accel_x) + "," + str(accel_y) + "," + str(accel_z) + "," + str(accel_diff) + "," + str(success) + "," + action + "\n" 

        f.write(output) 
        
        rate.sleep()

    f.close()
    c = open(num_trials_file, "w")        
    trial_number = trial_number + 1
    c.write(str(trial_number))
    c.close()
             
if __name__ == "__main__":
    try:
        main()
                        
    except:
        rospy.ROSInterruptException
        pass

    
