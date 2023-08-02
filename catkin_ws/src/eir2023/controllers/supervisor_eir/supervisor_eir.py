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

    start = False
    rospy.init_node("supervisor_node")
    rospy.Subscriber("/start", Empty, callback_start)

    pub_bmw_pose  = rospy.Publisher('/current_pose', Pose2D, queue_size=10)
    pub_car_N_pose  = rospy.Publisher('/current_pose_north_car', Pose2D, queue_size=10)
    pub_car_NW_pose  = rospy.Publisher('/current_pose_north_west_car', Pose2D, queue_size=10)
    pub_car_W_pose  = rospy.Publisher('/current_pose_west_car', Pose2D, queue_size=10)
    pub_car_SW_pose  = rospy.Publisher('/current_pose_south_west_car', Pose2D, queue_size=10)
    pub_obs_N_vel = rospy.Publisher('/obstacle_north_vel', Twist, queue_size=10)
    pub_obs_NW_vel = rospy.Publisher('/obstacle_north_west_vel', Twist, queue_size=10)
    pub_obs_W_vel = rospy.Publisher('/obstacle_west_vel', Twist, queue_size=10)
    pub_obs_SW_vel = rospy.Publisher('/obstacle_south_west_vel', Twist, queue_size=10)
    msg_N_vel = Twist()
    msg_NW_vel = Twist()
    msg_W_vel = Twist()
    msg_SW_vel = Twist()
    
    msg_bmw_pose = Pose2D()
    msg_car_N_pose = Pose2D()
    msg_car_NW_pose = Pose2D()
    msg_car_W_pose = Pose2D()
    msg_car_SW_pose = Pose2D()
    
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
               
        car_position = cars[0].getPosition()
        car_orientation = cars[0].getOrientation()
        msg_car_N_pose.x = car_position[0]
        msg_car_N_pose.y = car_position[1]
        msg_car_N_pose.theta = math.atan2(car_orientation[3], car_orientation[0])     
        pub_car_N_pose.publish(msg_car_N_pose)  
                
        car_position = cars[1].getPosition()
        car_orientation = cars[1].getOrientation()
        msg_car_NW_pose.x = car_position[0]
        msg_car_NW_pose.y = car_position[1]
        msg_car_NW_pose.theta = math.atan2(car_orientation[3], car_orientation[0])     
        pub_car_NW_pose.publish(msg_car_NW_pose)  
         
        car_position = cars[2].getPosition()
        car_orientation = cars[2].getOrientation()
        msg_car_W_pose.x = car_position[0]
        msg_car_W_pose.y = car_position[1]
        msg_car_W_pose.theta = math.atan2(car_orientation[3], car_orientation[0])     
        pub_car_W_pose.publish(msg_car_W_pose)  
        
        
        car_position = cars[3].getPosition()
        car_orientation = cars[3].getOrientation()
        msg_car_SW_pose.x = car_position[0]
        msg_car_SW_pose.y = car_position[1]
        msg_car_SW_pose.theta = math.atan2(car_orientation[3], car_orientation[0])     
        pub_car_SW_pose.publish(msg_car_SW_pose)  
               
        x,y,z,R,P,Y = cars[0].getVelocity()    
        msg_N_vel.linear.x, msg_N_vel.linear.y, msg_N_vel.linear.z = x,y,z
        msg_N_vel.angular.x, msg_N_vel.angular.y, msg_N_vel.angular.z = R,P,Y
        pub_obs_N_vel.publish(msg_N_vel)
        
        x,y,z,R,P,Y = cars[1].getVelocity()
        msg_NW_vel.linear.x, msg_NW_vel.linear.y, msg_NW_vel.linear.z = x,y,z
        msg_NW_vel.angular.x, msg_NW_vel.angular.y, msg_NW_vel.angular.z = R,P,Y
        pub_obs_NW_vel.publish(msg_NW_vel)
        
        x,y,z,R,P,Y = cars[2].getVelocity()
        msg_W_vel.linear.x, msg_W_vel.linear.y, msg_W_vel.linear.z = x,y,z
        msg_W_vel.angular.x, msg_W_vel.angular.y, msg_W_vel.angular.z = R,P,Y
        pub_obs_W_vel.publish(msg_W_vel)
        
        x,y,z,R,P,Y = cars[3].getVelocity()
        msg_SW_vel.linear.x, msg_SW_vel.linear.y, msg_SW_vel.linear.z = x,y,z
        msg_SW_vel.angular.x, msg_SW_vel.angular.y, msg_SW_vel.angular.z = R,P,Y
        pub_obs_SW_vel.publish(msg_SW_vel)
                         

        loop.sleep()


        
if __name__ == "__main__":
    try:
        main()
    except:
        pass

