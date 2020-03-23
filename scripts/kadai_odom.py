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


    x_start = get_odom()

    while not rospy.is_shutdown():
        data = get_odom()

        print ("current position: ", data.pose.pose.position.x)
        print data.pose.pose.position.x - x_start.pose.pose.position.x

        if (data.pose.pose.position.x - x_start.pose.pose.position.x) > 1.0:
            print "in"
            move.linear.x = 0.0
            pub_move.publish(move)

        pub_move.publish(move)
