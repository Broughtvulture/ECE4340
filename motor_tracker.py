#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
import time

def check_odom(msg):
    # print(msg)
    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    print(f"Hello World, we are cooking at {x} {y} ")
    # rot_q = msg.pose.pose.orientation
    # (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # # print(msg, x, y, roll, pitch, theta)
    # print(x, y, roll, pitch, theta)
    # print(f"Robot speed is set to linear x(1) :{ msg.twist.twist.linear.x}")

def motor_tracker():
    rospy.init_node('motor_tracker', anonymous=True)
    rospy.Subscriber("/odom", Odometry, check_odom)
    rospy.spin()


if __name__ == '__main__':
    try:
        motor_tracker()
    except rospy.ROSInterruptException:
        pass