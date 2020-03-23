#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
def callback(data):
    move = Twist()
    move.linear.x=0.1

    scan_data = np.array(data.ranges)
    print ("scan data: ", scan_data[0])

    min_data = 1.0

    if scan_data[0] < 0.5:
        move.linear.x=0
        move.angular.z = 0.3
        pub_move.publish(move)
        time.sleep(1)
    pub_move.publish(move)

if __name__ == '__main__':
    rospy.init_node('kadai_scan', anonymous=True)

    pub_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    scan_data_sub = rospy.Subscriber("/scan",LaserScan,callback)
    rospy.spin()
