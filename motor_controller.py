#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
import time

global pub
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

def motor_controller():
    rospy.init_node('motor_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(4) # 10hz
    speed = Twist()
    speed.linear.x = 1.0
    while not rospy.is_shutdown():
        rospy.loginfo(speed)
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass