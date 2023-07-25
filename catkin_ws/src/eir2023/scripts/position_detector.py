#!/usr/bin/env python3
"""
This node detect obstacles around the car using the lidar sensor.
Obstacles are detected only by counting the number of points inside 
a bounding box. Each BB defines a region around de car:
----------------|
North           | CAR
----------------|---------|--------------|
North-west |     West     | South West   |
-----------|--------------|--------------|
The node also publishes the positions using the mean of the points. 
"""
import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Float64MultiArray, Empty, Bool, Float64, Header
from geometry_msgs.msg import PointStamped, Point 
from sensor_msgs.msg import PointCloud2

def callback_point_cloud(msg):
    distance = 2.65
    xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    xyz = xyz[(xyz[:,2] > -1.6) & (xyz[:,2] < 0.5) ] #Filters points on floor and higher points
    obstacle_N_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] <   25) & (xyz[:,1] < 1.5) & (xyz[:,1] > -1.5)]
    obstacle_NW_points = xyz[(xyz[:,0] >  4.5) & (xyz[:,0] <   25) & (xyz[:,1] < 5.0) & (xyz[:,1] >  1.5)]
    obstacle_W_points  = xyz[(xyz[:,0] > -4.5) & (xyz[:,0] <  4.5) & (xyz[:,1] < 5.0) & (xyz[:,1] >  1.5)]
    obstacle_E_points  = xyz[(xyz[:,0] > -2.5) & (xyz[:,0] <  4.5) & (xyz[:,1] > -5.0) & (xyz[:,1] <  -1.5)]    
    obstacle_SW_points = xyz[(xyz[:,0] >  -10) & (xyz[:,0] < -4.5) & (xyz[:,1] < 5.0) & (xyz[:,1] >  1.5)]
    pos_N  = numpy.mean(obstacle_N_points , axis=0) if obstacle_N_points .shape[0] > 500  else [0,0,0]
    pos_NW = numpy.mean(obstacle_NW_points, axis=0) if obstacle_NW_points.shape[0] > 500  else [0,0,0]
    pos_W  = numpy.mean(obstacle_W_points , axis=0) if obstacle_W_points .shape[0] > 1000 else [0,0,0]
    pos_E  = numpy.mean(obstacle_E_points , axis=0) if obstacle_E_points .shape[0] > 1000 else [0,0,0]
    pos_SW = numpy.mean(obstacle_SW_points, axis=0) if obstacle_SW_points.shape[0] > 1000 else [0,0,0]
    pub_obs_N.publish(PointStamped(header=Header(frame_id="car_link"), point=Point(x=pos_N [0], y=pos_N [1], z=pos_N [2])))
    pub_obs_NW.publish(PointStamped(header=Header(frame_id="car_link"), point=Point(x=pos_NW [0], y=pos_NW [1], z=pos_NW [2])))
    pub_obs_W.publish(PointStamped(header=Header(frame_id="car_link"), point=Point(x=pos_W [0], y=pos_W [1], z=pos_W [2])))
    pub_obs_E.publish(PointStamped(header=Header(frame_id="car_link"), point=Point(x=pos_E [0], y=pos_E [1], z=pos_E [2])))
    pub_obs_SW.publish(PointStamped(header=Header(frame_id="car_link"), point=Point(x=pos_SW [0], y=pos_SW [1], z=pos_SW [2])))

def main():
    global pub_obs_N, pub_obs_NW, pub_obs_W, pub_obs_E, pub_obs_SW
    print("INITIALIZING OBSTACLE DETECTOR...")
    rospy.init_node("obstacle_detector")
    rospy.Subscriber('/point_cloud', PointCloud2, callback_point_cloud)
    pub_obs_N  = rospy.Publisher("/pos_obstacle/north"     , PointStamped, queue_size=10)
    pub_obs_NW = rospy.Publisher("/pos_obstacle/north_west", PointStamped, queue_size=10)
    pub_obs_W  = rospy.Publisher("/pos_obstacle/west"      , PointStamped, queue_size=10)
    pub_obs_E  = rospy.Publisher("/pos_obstacle/east"      , PointStamped, queue_size=10)    
    pub_obs_SW = rospy.Publisher("/pos_obstacle/south_west", PointStamped, queue_size=10)
    rate = rospy.Rate(10)
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

