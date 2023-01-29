#!/usr/bin/env python3
"""
This node is a logger and records the following data:

"""
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
from datetime import datetime
import os.path
from pathlib import Path


def callback_obstacle_north(msg):
    global obstacle_north
    obstacle_north = msg.data

def callback_obstacle_north_west(msg):
    global obstacle_north_west
    obstacle_north_west = msg.data

def callback_obstacle_west(msg):
    global obstacle_west
    obstacle_west = msg.data

def callback_obstacle_south_west(msg):
    global obstacle_south_west
    obstacle_south_west = msg.data

def callback_obstacle_distance(msg):
    global obstacle_distance
    obstacle_distance = msg.data

def callback_follow_enable(msg):
    global follow_enable
    callback_follow_enable = msg.data
    
def callback_steady_motion_enable(msg):
    global steady_motion_enable
    steady_motion_enable = msg.data    
    
def callback_passing_start(msg):
    global passing_start
    passing_start = msg.data    
    
def callback_passing_finished(msg):
    global passing_finished
    callback_passing_finished = msg.data        

def main():
    global obstacle_north, obstacle_north_west, obstacle_west, obstacle_south_west
    global obstacle_distance, follow_enable, steady_motion_enable, passing_start, passing_finished
    
    num_trial = 0
    trials_file = "num_trials.data"
    
    print("INITIALIZING LOGGER...")
    rospy.init_node("logger")
    rospy.Subscriber("/obstacle/north"     , Bool, callback_obstacle_north)
    rospy.Subscriber("/obstacle/north_west", Bool, callback_obstacle_north_west)
    rospy.Subscriber("/obstacle/west"      , Bool, callback_obstacle_west)
    rospy.Subscriber("/obstacle/south_west", Bool, callback_obstacle_south_west)

    rospy.Subscriber("/obstacle/distance"  , Float64, callback_obstacle_distance)
    rospy.Subscriber("/follow/enable"      , Bool, callback_follow_enable)
    rospy.Subscriber("/steady_motion/enable", Bool, callback_steady_motion_enable)
    rospy.Subscriber("/passing/start"      , Bool, callback_passing_start)
    rospy.Subscriber("/passing/finished"   , Bool, callback_passing_finished)
    
    rate = rospy.Rate(10)
    obstacle_north      = False
    obstacle_north_west = False
    obstacle_west       = False
    obstacle_south_west = False
    obstacle_distance = 100000
    follow_enable = False 
    steady_motion_enable = False 
    passing_start = False 
    passing_finished = False   

    # Lectura del número de corrida
    file_exists = os.path.exists(trials_file)
    if file_exists:
       c = open(trials_file, "r")
       trial = c.read()
       c.close()
       num_trial = int(trial)

    else:
       myfile = Path(trials_file)
       myfile.touch(exist_ok=True)
       num_trial = num_trial + 1       
       
    str_trial = str(num_trial)

    # Archivo log
    f = open("logger.log","a")
    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    f.write('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n')
    output = " Test number " +  str_trial + " " + dt_string + "\n"
    f.write(output)
    f.write('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n')
       
    print("Logger.->Waiting for start signal")
    rospy.wait_for_message("/start", Empty, timeout=10000.0)
    print("Logger.->Start signal received")

    iteration = 0
    while not rospy.is_shutdown():
    
        iteration = iteration + 1
        now = datetime.now()
        #dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        dt_string = now.strftime("%H:%M:%S")
        
        if follow_enable:
           action = "keep_distance"
        elif steady_motion_enable:
           action = "cruise"
        elif passing_start or passing_finished:
           action = "change_lane"          
   
        output = str(iteration) +") " + dt_string + " O_N= " + str(obstacle_north) + " O_NW= " + str(obstacle_north_west) + " O_SN= " + str(obstacle_south_west) + " O_SN= " + str(obstacle_west) + " O_D= " + str(obstacle_distance) + " F_E= " + str(follow_enable) +  " S_M_E= " + str(steady_motion_enable) + " P_S= " + str(passing_start) + " P_F= " + str(passing_finished) +  " action= " + action + "\n"
          
        f.write(output)  
        rate.sleep()
             
    f.close()
    c = open(trials_file, "w")        
    num_trial = num_trial + 1
    c.write(str(num_trial))
    c.close()
             

if __name__ == "__main__":
    try:
        main()
                        
    except:
        rospy.ROSInterruptException
        pass

    

