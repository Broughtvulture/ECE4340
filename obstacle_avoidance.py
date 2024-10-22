#!/usr/bin/env python
import rospy
from ca_msgs.msg import Bumper
from geometry_msgs.msg import Twist

# Global flags for bumper contact
left_contact = False
right_contact = False

def bumper_callback(data):
    global left_contact, right_contact
    right_contact = data.is_right_pressed
    left_contact = data.is_left_pressed

def obstacle_avoidance():
    rospy.init_node('obstacle_avoidance', anonymous=True)

    # Subscribe to the bumper topic
    sub = rospy.Subscriber('/bumper', Bumper, bumper_callback)
    # Publisher for velocity commands
    pub = rospy.Publisher('/obstacle_avoidance/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    move_cmd = Twist()

    while not rospy.is_shutdown():
        if left_contact:
            move_cmd.angular.z = -1  # Turn right if left bumper is pressed
        elif right_contact:
            move_cmd.angular.z = 1  # Turn left if right bumper is pressed
        else:
            move_cmd.angular.z = 0  # Stop turning when no bumper is pressed

        # Publish the velocity command
        pub.publish(move_cmd)

        rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_avoidance()
    except rospy.ROSInterruptException:
        pass
