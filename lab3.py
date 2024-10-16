#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math

# x = 0.0
# y = 0.0
# theta = 0.0
# goal_index = 0

# square_x = 1
# square_y = 0.5
# Define a list of goals
# goals = [Point(square_x/2, square_y/2, 0), Point(-square_x, square_y/2, 0), Point(-square_x, -square_y, 0), Point(square_x/2, -square_y, 0)]
# goals = [Point(square_x/2, square_y/2, 0), Point(square_x/2, -square_y, 0), Point(-square_x, 0, 0), Point(0, square_y, 0)]

def check_odom(msg):
    print(msg)
    # global x, y, theta
    # x = msg.pose.pose.position.x
    # y = msg.pose.pose.position.y
    # rot_q = msg.pose.pose.orientation
    # (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def odom_listener():
    rospy.init_node("odom_tracker", anonymous=True)
    
    sub = rospy.Subscriber("/odom", Odometry, check_odom)
    rospy.loginfo("Listening for Odometry data")
    rospy.spin()
    # pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # speed = Twist()
    # r = rospy.Rate(4)
    
    # goal = goals[goal_index]  # Get the current goal

    # inc_x = goal.x - x
    # inc_y = goal.y - y

    # angle_to_goal = math.atan2(inc_y, inc_x)
    # while (True):
        # r.sleep()
    # if abs(angle_to_goal - theta) > 0.1:
    #     speed.linear.x = 0.0
    #     # speed.angular.z = (theta-angle_to_goal)*-1
    #     speed.angular.z = 0.2   #(theta-angle_to_goal)*-1
    # else:
    #     speed.linear.x = 0.1
    #     speed.angular.z = 0.0

    # # Check if the robot has reached the goal (within a small threshold)
    # if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
    #     rospy.loginfo(f"Reached goal {goal_index + 1}: {goal.x}, {goal.y}")  # Log the reached goal
    #     goal_index += 1  # Move to the next goal

    #     # Check if we reached the last goal
    #     if goal_index >= len(goals):  # If it is the last goal
    #         speed.linear.x = 0.0
    #         speed.angular.z = 0.0
    #         pub.publish(speed)  # Stop the robot
    #         rospy.signal_shutdown("Reached the last goal")  # Shutdown the node
    #         break  # Exit the loop

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass