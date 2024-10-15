#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from collections import defaultdict

class VelocityMultiplexer:
    def __init__(self):
        rospy.init_node('velocity_multiplexer', anonymous=True)

        # Priority mapping
        self.priority_map = {
            'high_priority_cmd': 1,
            'medium_priority_cmd': 2,
            'low_priority_cmd': 3
        }

        # Store commands based on priority
        self.current_commands = defaultdict(lambda: None)

        # Subscribers for each velocity command topic
        self.subscribers = {
            'high_priority_cmd': rospy.Subscriber('high_priority_cmd', Twist, self.callback),
            'medium_priority_cmd': rospy.Subscriber('medium_priority_cmd', Twist, self.callback),
            'low_priority_cmd': rospy.Subscriber('low_priority_cmd', Twist, self.callback),
        }

        # Publisher for the final velocity command
        self.publisher = rospy.Publisher('final_velocity_command', Twist, queue_size=10)

    def callback(self, msg):
        # Store the latest command for the corresponding topic
        topic = rospy.get_caller_id()[1:]  # Get the topic name from caller id
        self.current_commands[topic] = msg
        self.publish_highest_priority_command()

    def publish_highest_priority_command(self):
        highest_priority = None
        final_command = Twist()  # Initialize a zero velocity Twist

        for cmd, priority in self.priority_map.items():
            if self.current_commands[cmd] is not None:
                # Only update if the command is not None
                if highest_priority is None or priority < highest_priority:
                    highest_priority = priority
                    final_command = self.current_commands[cmd]

        if highest_priority is not None:
            self.publisher.publish(final_command)
            rospy.loginfo(f'Publishing command: linear={final_command.linear}, angular={final_command.angular} from topic with priority: {highest_priority}')

if __name__ == '__main__':
    try:
        VelocityMultiplexer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
