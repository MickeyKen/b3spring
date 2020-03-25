#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def get_odom():
    #read laser data
    data = None
    while data is None:
        try:
            data = rospy.wait_for_message('/odom', Odometry, timeout=5)
        except:
            pass
    return data

if __name__ == '__main__':
    rospy.init_node('kadai_odom')

    pub_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    move = Twist()

    move.linear.x=0.1

    while not rospy.is_shutdown():
        data = get_odom()

        print ("current position: ", data.pose.pose.position.x)

        pub_move.publish(move)
