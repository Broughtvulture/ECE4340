#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def move_straight(pub, speed, duration):
    move_cmd = Twist()
    move_cmd.linear.x = speed
    pub.publish(move_cmd)
    time.sleep(duration)
    move_cmd.linear.x = 0
    pub.publish(move_cmd)

def turn(pub, angular_speed, duration):
    move_cmd = Twist()
    move_cmd.angular.z = angular_speed
    pub.publish(move_cmd)
    time.sleep(duration)
    move_cmd.angular.z = 0
    pub.publish(move_cmd)

def draw_square():
    rospy.init_node('draw_square', anonymous=True)
    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    
    # Duration variables (you may need to adjust based on your robot's speed)
    straight_time = 2  # Time to move forward (adjust this for real robot speed)
    turn_time = 1.5  # Time to turn 90 degrees

    for _ in range(4):
        move_straight(pub, 0.5, straight_time)
        turn(pub, 0.5, turn_time)

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass
