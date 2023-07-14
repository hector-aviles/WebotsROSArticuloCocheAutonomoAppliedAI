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
"""
import math
import numpy
import rospy
import ros_numpy
from std_msgs.msg import Float64MultiArray, Empty, Bool, Float64
from sensor_msgs.msg import PointCloud2

def callback_point_cloud(msg):
    distance = 2.65
    xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    xyz = xyz[(xyz[:,2] > -1.6) & (xyz[:,2] < 0.5) ] #Filters points on floor and higher points
    obstacle_N_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] <   25) & (xyz[:,1] < 1.5) & (xyz[:,1] > -1.5)]
    obstacle_NW_points = xyz[(xyz[:,0] >  4.5) & (xyz[:,0] <   25) & (xyz[:,1] < 5.0) & (xyz[:,1] >  1.5)]
    obstacle_W_points  = xyz[(xyz[:,0] > -4.5) & (xyz[:,0] <  4.5) & (xyz[:,1] < 5.0) & (xyz[:,1] >  1.5)]
    obstacle_SW_points = xyz[(xyz[:,0] >  -10) & (xyz[:,0] < -4.5) & (xyz[:,1] < 5.0) & (xyz[:,1] >  1.5)]
    
    obstacle_NE_points  = xyz[(xyz[:,0] >  4.5) & (xyz[:,0] <   25) & (xyz[:,1] > -5.0) & (xyz[:,1] <  -1.5)]        
    obstacle_E_points  = xyz[(xyz[:,0] > -4.5) & (xyz[:,0] <  4.5) & (xyz[:,1] > -5.0) & (xyz[:,1] <  -1.5)] 
    obstacle_SE_points  = xyz[(xyz[:,0] >  -10) & (xyz[:,0] < -4.5) & (xyz[:,1] > -5.0) & (xyz[:,1] <  -1.5)]               
    
    obstacle_N  = obstacle_N_points .shape[0] > 500
    obstacle_NW = obstacle_NW_points.shape[0] > 500
    obstacle_W  = obstacle_W_points .shape[0] > 1000
    obstacle_SW = obstacle_SW_points.shape[0] > 1000

    obstacle_NE  = obstacle_NE_points .shape[0] > 1000    
    obstacle_E  = obstacle_E_points .shape[0] > 1000    
    obstacle_SE  = obstacle_SE_points .shape[0] > 1000    
    
    pub_obs_N .publish(obstacle_N )
    pub_obs_NW.publish(obstacle_NW)
    pub_obs_W .publish(obstacle_W )
    pub_obs_SW.publish(obstacle_SW)
    pub_obs_NE .publish(obstacle_NE )    
    pub_obs_E .publish(obstacle_E )            
    pub_obs_SE .publish(obstacle_SE )    
        
    if obstacle_N:
        pub_obs_dist.publish(numpy.linalg.norm(numpy.mean(obstacle_N_points, axis=0)))

def main():
    global pub_obs_N, pub_obs_NW, pub_obs_W, pub_obs_SW, pub_obs_NE, pub_obs_E, pub_obs_SE, pub_obs_dist
    print("INITIALIZING OBSTACLE DETECTOR...", flush=True)
    rospy.init_node("obstacle_detector")
    rospy.Subscriber('/point_cloud', PointCloud2, callback_point_cloud)
    pub_obs_N  = rospy.Publisher("/obstacle/north"     , Bool, queue_size=10)
    pub_obs_NW = rospy.Publisher("/obstacle/north_west", Bool, queue_size=10)
    pub_obs_W  = rospy.Publisher("/obstacle/west"      , Bool, queue_size=10)
    pub_obs_SW = rospy.Publisher("/obstacle/south_west", Bool, queue_size=10)
    pub_obs_NE  = rospy.Publisher("/obstacle/north_east"      , Bool, queue_size=10) 
    pub_obs_SE  = rospy.Publisher("/obstacle/south_east"      , Bool, queue_size=10) 
    
    pub_obs_NE  = rospy.Publisher("/obstacle/north_east"      , Bool, queue_size=10)                
    pub_obs_E  = rospy.Publisher("/obstacle/east"      , Bool, queue_size=10)                
    pub_obs_SE  = rospy.Publisher("/obstacle/south_east"      , Bool, queue_size=10)                        
    
    pub_obs_dist = rospy.Publisher("/obstacle/distance", Float64, queue_size=10)
    rate = rospy.Rate(10)
    
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

