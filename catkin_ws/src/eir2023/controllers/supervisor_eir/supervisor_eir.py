#!/usr/bin/env python3
import numpy
import rospy
import math
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
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
    bmw  = robot.getFromDef('BMW_X5')  

    print('breakpoint 1...')
    #linear_velocity_North = cars[0].getField("translation")
    print('breakpoint 1.5 ...')
    start = False
    rospy.init_node("supervisor_node")
    rospy.Subscriber("/start", Empty, callback_start)
    print('breakpoint 2...')
    pub_bmw_pose  = rospy.Publisher('/current_pose', Pose2D, queue_size=10)
    pub_obs_N_vel = rospy.Publisher('/obstacle_north_vel', Twist, queue_size=10)
    pub_obs_NW_vel = rospy.Publisher('/obstacle_north_west_vel', Twist, queue_size=10)
    pub_obs_W_vel = rospy.Publisher('/obstacle_west_vel', Twist, queue_size=10)
    pub_obs_SW_vel = rospy.Publisher('/obstacle_south_west_vel', Twist, queue_size=10)
    msg_N_vel = Twist()
    msg_NW_vel = Twist()
    msg_W_vel = Twist()
    msg_SW_vel = Twist()
    
    msg_bmw_pose = Pose2D()
    loop = rospy.Rate(1000/TIME_STEP)
    print('breakpoint 3...')
    while robot.step(TIME_STEP) != -1 and not rospy.is_shutdown():
        if start:
            start = False
            for car in cars:
                if car is not None:
                    car.setVelocity([0.5,0,0, 0,0,0])
        bmw_position = bmw.getPosition()
        bmw_orientation = bmw.getOrientation()
        msg_bmw_pose.x = bmw_position[0]
        msg_bmw_pose.y = bmw_position[1]
        msg_bmw_pose.theta = math.atan2(bmw_orientation[3], bmw_orientation[0])
        
        pub_bmw_pose.publish(msg_bmw_pose)  

        x,y,z,R,P,Y = cars[0].getVelocity()
        msg_N_vel.linear.x, msg_N_vel.linear.y, msg_N_vel.linear.z = x,y,z
        msg_N_vel.angular.x, msg_N_vel.angular.y, msg_N_vel.angular.z = R,P,Y
        pub_obs_N_vel.publish(msg_N_vel)
                          

        loop.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except:
        pass

