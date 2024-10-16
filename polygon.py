#!/usr/bin/env python3 
import rospy 
from geometry_msgs.msg import Twist 
import time 

def moveRobot(move, Tx, Ty, Tz, Rx, Ry, Rz): 
    move.linear.x = Tx 
    move.linear.y = Ty 
    move.linear.z = Tz 
    move.angular.x = Rx 
    move.angular.y = Ry 
    move.angular.z = Rz 

rospy.init_node('lastforward') 
rate=rospy.Rate(1) 
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

move = Twist() 
print("Robot Starting...") 
moveRobot(move, 0, 0, 0, 0, 0, 0) 
pub.publish(move) 
rospy.sleep(5) 

for i in range(4): 
    print("Robot Moving Forward...") 
    moveRobot(move, 0.05, 0, 0, 0, 0, 0) 
    pub.publish(move) 
    rospy.sleep(10) 

    print("Robot Stop!") 
    moveRobot(move, 0, 0, 0, 0, 0, 0) 
    pub.publish(move) 
    rospy.sleep(5) 

    print("Robot Turn...") 
    moveRobot(move, 0, 0, 0, 0, 0, .16) 
    pub.publish(move) 
    rospy.sleep(10) 

    print("Robot Stop!") 
    moveRobot(move, 0, 0, 0, 0, 0, 0) 
    pub.publish(move) 
    rospy.sleep(5) 
    print(f"Movement Counter = {i+1}") 