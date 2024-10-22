#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class VelocityMultiplexer:
    def __init__(self):
        rospy.init_node('velocity_multiplexer', anonymous=True)

        # Subscribers to different velocity topics (e.g., teleop, obstacle avoidance)
        self.sub_teleop = rospy.Subscriber('/teleop/cmd_vel', Twist, self.teleop_callback)
        self.sub_obstacle = rospy.Subscriber('/obstacle_avoidance/cmd_vel', Twist, self.obstacle_callback)
        
        # Publisher for the final velocity command
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Storage for the latest velocity messages
        self.teleop_cmd = Twist()
        self.obstacle_cmd = Twist()
        
        self.rate = rospy.Rate(10)  # 10 Hz

    def teleop_callback(self, msg):
        self.teleop_cmd = msg

    def obstacle_callback(self, msg):
        self.obstacle_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            # Priority decision: obstacle avoidance has higher priority
            if self.obstacle_cmd.angular.z != 0:
                self.pub.publish(self.obstacle_cmd)
            else:
                self.pub.publish(self.teleop_cmd)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        vmux = VelocityMultiplexer()
        vmux.run()
    except rospy.ROSInterruptException:
        pass
