#!/usr/bin/env python3
import numpy
import rospy
import math
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D
from controller import Supervisor

TIME_STEP = 10
robot = Supervisor()

def callback_start(msg):
    global start
    start = True

def main():
    global start
    print('Starting Controller Supervisor...')
    
    cars = [robot.getFromDef('obstacle_north'), robot.getFromDef('obstacle_north_west'),
            robot.getFromDef('obstacle_west' ), robot.getFromDef('obstacle_south_west')]
<<<<<<< HEAD
    
  
    bmw  = robot.getFromDef('BMW_X5')  
    
    linear_velocity_field = cars.getField("translation")
              
=======
    bmw  = robot.getFromDef('BMW_X5')            
>>>>>>> 4143c6923d6744f01fc2df6b1a92bfdcdda95d93
    start = False
    rospy.init_node("supervisor_node")
    rospy.Subscriber("/start", Empty, callback_start)
    pub_bmw_pose = rospy.Publisher('/current_pose', Pose2D, queue_size=10)    
    msg_bmw_pose = Pose2D()
    loop = rospy.Rate(1000/TIME_STEP)
    while robot.step(TIME_STEP) != -1 and not rospy.is_shutdown():
        if start:
            start = False
            for car in cars:
                if car is not None:
                    car.setVelocity([6,0,0, 0,0,0])
        bmw_position = bmw.getPosition()
        bmw_orientation = bmw.getOrientation()
        msg_bmw_pose.x = bmw_position[0]
        msg_bmw_pose.y = bmw_position[1]
        msg_bmw_pose.theta = math.atan2(bmw_orientation[3], bmw_orientation[0])

        pub_bmw_pose.publish(msg_bmw_pose)  
        
        linear_velocity = linear_velocity_field.getSFVec3f()
        linear_x, linear_y, linear_z = linear_velocity
        print("Linear velocity: ", linear_x, linear_y, linear_z)
                          

        loop.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except:
        pass

