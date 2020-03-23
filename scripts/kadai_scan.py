import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(data):
    move = Twist()
    move.linear.x=0.1
    pub_move.publish(move)
if __name__ == '__main__':
    rospy.init_node('kadai_scan', anonymous=True)
    pub_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    rospy.Subscriber("/scan",LaserScan,callback)
    rospy.spin()
