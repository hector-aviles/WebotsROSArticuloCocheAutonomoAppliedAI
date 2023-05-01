#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Clock

def main():
    pub1 = rospy.Publisher('clock_Webots',Clock, queue_size=10)
    print("INITIALIZING CLOCK_WEBOTS...")
    rospy.init_node("clock_Webots")
    rate = rospy.Rate(10) # 10hz 
    sim_clock = Clock()
    zero_time = rospy.get_time()

    while not rospy.is_shutdown():
       sim_clock.clock = rospy.Time.from_sec(rospy.get_time() - zero_time)
       rospy.loginfo(sim_clock)
       pub1.publish(sim_clock)
       rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
