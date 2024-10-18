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
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('motor_controller', anonymous=True)
    # pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, check_odom)
    # rate = rospy.Rate(4) # 10hz
    # speed = Twist()
    
    # speed.linear.x = 1.0
    # print("Publisher")
    # while not rospy.is_shutdown():
    #     # speed = "Publisher"
    #     speed = odom
    #     # hello_str = f"hello world {rospy.get_time()} : {speed}"
    #     # rospy.loginfo(speed)
    #     # pub.publish(speed)
    #     pub.publish(speed)
    #     # motor_correcter
    #     rate.sleep()
    rospy.spin()

def check_odom(msg):
    print(msg.pose.pose)
    # # Negate the position values
    # msg.pose.pose.position.x = -msg.pose.pose.position.x
    # msg.pose.pose.position.y = -msg.pose.pose.position.y
    # msg.pose.pose.position.z = -msg.pose.pose.position.z

    # # Negate the orientation values (quaternion)
    # msg.pose.pose.orientation.x = -msg.pose.pose.orientation.x
    # msg.pose.pose.orientation.y = -msg.pose.pose.orientation.y
    # msg.pose.pose.orientation.z = -msg.pose.pose.orientation.z
    # msg.pose.pose.orientation.w = -msg.pose.pose.orientation.w

    speed = Twist()
    # speed.linear.x = 1
    speed.linear.x = 1 - msg.twist.twist.linear.x
    speed.linear.y = 0 - msg.twist.twist.linear.y
    speed.linear.z = 0 - msg.twist.twist.linear.z
    speed.angular.x = 0 - msg.twist.twist.angular.x
    speed.angular.y = 0 - msg.twist.twist.angular.y
    speed.angular.z = 0 - msg.twist.twist.angular.z
    speed.angular.w = 0 - msg.twist.twist.angular.w
    pub.publish(speed)
    print(f"Subscriber Info :")
    # rospy.loginfo(f"Subscriber message: {-1*msg}")
    # global x, y, theta
    # x = msg.pose.pose.position.x
    # y = msg.pose.pose.position.y
    # # print(f"Hello World, we are cooking at {x} {y} ")
    # rot_q = msg.pose.pose.orientation
    # (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # # print(msg, x, y, roll, pitch, theta)
    # # print(x)
    # # print(f"Robot speed is set to linear x(1) :{ msg.twist.twist.linear.x}")

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
