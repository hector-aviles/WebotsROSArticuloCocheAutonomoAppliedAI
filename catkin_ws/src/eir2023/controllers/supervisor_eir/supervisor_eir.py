#!/usr/bin/env python3
import numpy
import rospy
from std_msgs.msg import Empty
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
    start = False
    rospy.init_node("supervisor_node")
    rospy.Subscriber("/start", Empty, callback_start)
    loop = rospy.Rate(1000/TIME_STEP)
    while robot.step(TIME_STEP) != -1 and not rospy.is_shutdown():
        if start:
            start = False
            for car in cars:
                if car is not None:
                    car.setVelocity([6,0,0, 0,0,0])
        loop.sleep()
        
if __name__ == "__main__":
    try:
        main()
    except:
        pass

