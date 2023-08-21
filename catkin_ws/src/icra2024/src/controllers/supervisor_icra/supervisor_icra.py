#!/usr/bin/env python3
import numpy as np
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
    
    cars = [robot.getFromDef('vehicle_1'), robot.getFromDef('vehicle_2'),
robot.getFromDef('vehicle_3'), robot.getFromDef('vehicle_4'),  robot.getFromDef('vehicle_5'), robot.getFromDef('vehicle_6'), robot.getFromDef('vehicle_7'), robot.getFromDef('vehicle_8'), robot.getFromDef('vehicle_9'), robot.getFromDef('vehicle_10')]

    tf = []
    i = 0
    for car in cars:
        if car is not None:
              tf.append(car.getField("translation"))
              values = tf[i].getSFVec3f()
              #print(i, ")", "Initial:", values)
              rand_val = np.random.uniform(-2,2,1)
              #print("Random number", rand_val)
              values[0] = values[0] + rand_val
              #print("New x value", values[0])
              tf[i].setSFVec3f(values)
              car.resetPhysics()   
        i = i + 1 

    bmw  = robot.getFromDef('BMW_X5')  

    #linear_velocity_North = cars[0].getField("translation")
    start = False
    rospy.init_node("supervisor_node")
    rospy.Subscriber("/policy_started", Empty, callback_start)
    pub_bmw_pos  = rospy.Publisher("/self_driving_pose", Pose2D, queue_size=1)
    pub_car_1_pos  = rospy.Publisher("/car_1_pose", Pose2D, queue_size=1)
    pub_car_2_pos  = rospy.Publisher("/car_2_pose", Pose2D, queue_size=1)
    pub_car_3_pos  = rospy.Publisher("/car_3_pose", Pose2D, queue_size=1)
    pub_car_4_pos  = rospy.Publisher("/car_4_pose", Pose2D, queue_size=1)
    pub_car_5_pos  = rospy.Publisher("/car_5_pose", Pose2D, queue_size=1)
    pub_car_6_pos  = rospy.Publisher("/car_6_pose", Pose2D, queue_size=1)
    pub_car_7_pos  = rospy.Publisher("/car_7_pose", Pose2D, queue_size=1)
    pub_car_8_pos  = rospy.Publisher("/car_8_pose", Pose2D, queue_size=1)
    pub_car_9_pos  = rospy.Publisher("/car_9_pose", Pose2D, queue_size=1)
    pub_car_10_pos  = rospy.Publisher("/car_10_pose", Pose2D, queue_size=1)

    msg_bmw_pos = Pose2D()
    msg_car_pos = Pose2D()
        
    loop = rospy.Rate(1000/TIME_STEP)
    while robot.step(TIME_STEP) != -1 and not rospy.is_shutdown():
    
      if start:
        start = False    

        i = 0 
        for car in cars:
            if car is not None:        
               car.setVelocity([6,0,0, 0,0,0]) 
               values = tf[i].getSFVec3f()
               msg_car_pos.x = values[0] 
               msg_car_pos.y = values[1]
               msg_car_pos.theta = values[2]   
                           
               if i == 0:
                  pub_car_1_pos.publish(msg_car_pos)  
               elif i == 1:
                  pub_car_2_pos.publish(msg_car_pos)               
               elif i == 2:
                  pub_car_3_pos.publish(msg_car_pos)               
               elif i == 3:
                  pub_car_4_pos.publish(msg_car_pos)               
               elif i == 4:
                  pub_car_5_pos.publish(msg_car_pos)
               elif i == 5:
                  pub_car_6_pos.publish(msg_car_pos)               
               elif i == 6:
                  pub_car_7_pos.publish(msg_car_pos)
               elif i == 7:
                  pub_car_8_pos.publish(msg_car_pos)               
               elif i == 8:
                  pub_car_9_pos.publish(msg_car_pos)
               elif i == 9:
                  pub_car_10_pos.publish(msg_car_pos)
            i = i + 1                     
               
        bmw_pos = bmw.getPosition()
        bmw_orient = bmw.getOrientation()
        msg_bmw_pos.x = bmw_pos[0]
        msg_bmw_pos.y = bmw_pos[1]
        msg_bmw_pos.theta = math.atan2(bmw_orient[3], bmw_orient[0])
        
        pub_bmw_pos.publish(msg_bmw_pos)
                          
      loop.sleep()
  
  
        
if __name__ == "__main__":
    try:
        main()
    except:
        pass

