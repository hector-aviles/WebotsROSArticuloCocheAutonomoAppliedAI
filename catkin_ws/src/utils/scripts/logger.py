#!/usr/bin/env python3
"""
This node is a logger and records the following data:

"""
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
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
    
def callback_speed(msg):
    global speed
    speed = msg.data 
    
def callback_steering(msg):
    global steering
    steering = msg.data         
    
    
def main():
    global obstacle_north, obstacle_north_west, obstacle_west, obstacle_south_west
    global obstacle_distance, follow_enable, steady_motion_enable, passing_start, passing_finished
    global speed, steering
        
    repetitions_file = "num_repetition.data"

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
    
    rospy.Subscriber("/speed", Float64, callback_speed)
    rospy.Subscriber("/steering", Float64, callback_steering)

    rate = rospy.Rate(10)

    num_repetition = 0
    obstacle_north      = False
    obstacle_north_west = False
    obstacle_west       = False
    obstacle_south_west = False
    obstacle_distance = 100000
    follow_enable = False 
    steady_motion_enable = False 
    passing_start = False 
    passing_finished = False  
    speed = 0.0
    steering = 0.0


    print("Hola mundo", flush = True)
    # Lectura del número de corrida
    file_exists = os.path.exists(repetitions_file)
    if file_exists:
       c = open(repetitions_file, "r")
       repetition = c.read()
       c.close()
       num_repetition = int(repetition)
    else:
       myfile = Path(repetitions_file)
       myfile.touch(exist_ok=True)
       num_repetition = num_repetition + 1 
       c = open(repetitions_file, "w")        
       c.write(str(num_repetition))
       c.close()      
       
    str_repetition = str(num_repetition)

    # Record header only once
    f = open("logger.log","a")
    if num_repetition == 1:
       output = "repetition," +"iteration," + "time," + "speed," + "steering," + "obstacle_North," + "obstacle_NorthWest," + "obstacle_SouthWest," + "obstacle_West," + "obstacle_distance," + "follow_enable," +  "steady_motion_enable," + "passing_start," + "passing_finished," + "action" + "\n"     
       f.write(output)
     
    #print("Logger.->Waiting for start signal")
    #rospy.wait_for_message("/start", Empty, timeout=10000.0)
    #print("Logger.->Start signal received")

    iteration = 0
    while not rospy.is_shutdown():
    
        iteration = iteration + 1
        now = rospy.get_time()
        
        if follow_enable:
           action = "keep_distance"
        elif steady_motion_enable:
           action = "cruise"
        elif passing_start or passing_finished:
           action = "change_lane"  
        else:
           action = "NA"
                       
        output = str(num_repetition) + "," + str(iteration) +"," + str(now) + "," + str(speed) + "," + str(steering) + "," + str(obstacle_north) + "," + str(obstacle_north_west) + "," + str(obstacle_south_west) + "," + str(obstacle_west) + "," +  str(obstacle_distance) + "," + str(follow_enable) +  "," + str(steady_motion_enable) + "," + str(passing_start) + "," + str(passing_finished) + "," + action + "\n" 

        f.write(output) 
        
        rate.sleep()

    f.close()
    c = open(repetitions_file, "w")        
    num_repetition = num_repetition + 1
    c.write(str(num_repetition))
    c.close()
             
if __name__ == "__main__":
    try:
        main()
                        
    except:
        rospy.ROSInterruptException
        pass

    