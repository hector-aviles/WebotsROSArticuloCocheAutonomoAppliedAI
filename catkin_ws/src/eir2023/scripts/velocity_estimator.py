#!/usr/bin/env python3

import math
import numpy
import rospy
from kalman_filter import EKF
from std_msgs.msg import Float64MultiArray, Empty, Bool, Float64, Header, ColorRGBA
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import MarkerArray, Marker

def callback_obs_N(msg):
    global x_N
    x_N = ekf_N.estimate([msg.point.x, msg.point.y])

def callback_obs_NW(msg):
    global x_NW
    x_NW = ekf_NW.estimate([msg.point.x, msg.point.y])

def callback_obs_W(msg):
    global x_W
    x_W = ekf_W.estimate([msg.point.x, msg.point.y])

def callback_obs_SW(msg):
    global x_SW
    x_SW = ekf_SW.estimate([msg.point.x, msg.point.y])

def get_marker(x, id):
    mrk = Marker(header=Header(frame_id="car_link"), ns="obs_velocities", id=id, action=Marker.ADD, type=Marker.ARROW)
    mrk.pose.position.x = x[0]
    mrk.pose.position.y = x[1]
    mrk.pose.orientation.z = math.sin(math.atan2(x[3], x[2])/2)
    mrk.pose.orientation.w = math.cos(math.atan2(x[3], x[2])/2)
    mrk.scale.x = math.sqrt(x[2]**2 + x[3]**2) + 0.1
    mrk.scale.y = 0.2
    mrk.scale.z = 0.2
    mrk.color = ColorRGBA(r=1.0, a=1.0)
    return mrk

def get_marker_array(x_N, x_NW, x_W, x_SW):
    markers = MarkerArray()
    markers.markers.append(get_marker(x_N,0))
    markers.markers.append(get_marker(x_NW,1))
    markers.markers.append(get_marker(x_W,2))
    markers.markers.append(get_marker(x_SW,3))
    return markers

def main():
    global ekf_N, ekf_NW, ekf_W,  ekf_SW, x_N, x_NW, x_W, x_SW
    print("INITIALIZING OBSTACLE DETECTOR...")
    rospy.init_node("velocity_estimator")
    rospy.Subscriber("/pos_obstacle/north"     , PointStamped, callback_obs_N)
    rospy.Subscriber("/pos_obstacle/north_west", PointStamped, callback_obs_NW)
    rospy.Subscriber("/pos_obstacle/west"      , PointStamped, callback_obs_W)
    rospy.Subscriber("/pos_obstacle/south_west", PointStamped, callback_obs_SW)
    pub_markers = rospy.Publisher("/obstacle_velocities_markers", MarkerArray, queue_size=10)
    rate = rospy.Rate(10)
    Q = numpy.identity(4) * 0.1
    R = numpy.identity(2) * 0.01
    ekf_N  = EKF(0.1, Q, R)
    ekf_NW = EKF(0.1, Q, R)
    ekf_W  = EKF(0.1, Q, R)
    ekf_SW = EKF(0.1, Q, R)
    x_N  = ekf_N.estimate([0,0])
    x_NW = ekf_NW.estimate([0,0])
    x_W  = ekf_W.estimate([0,0])
    x_SW = ekf_SW.estimate([0,0])
    while not rospy.is_shutdown():
        pub_markers.publish(get_marker_array(x_N, x_NW, x_W, x_SW))
        rate.sleep()
    

if __name__ == "__main__":
    main()
    
