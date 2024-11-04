#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
import math
import time

# Create a Length by Width Matrix
def make_Mat(x, y, value=None):
    return np.full((x, y), value)

# Create a Homogenious Matrix using Rotation & Translation Matrix
def make_Hmat(rot, trans):
    # Attach translation to homogenious matrix
    homo = np.hstack((rot, trans))
    # Create scaler matrix
    scaler = make_Mat(1, 4, 0)
    # Set last value to 1
    scaler[0, -1] = 1
    # Attach scaler to homogenious matrix
    homo = np.vstack((homo, scaler))
    # Return homogenious matrix
    return homo

# Extract x, y, theta from homogeneous matrix
def extract_Mat(Hmat):
    x = Hmat[0, 3]
    y = Hmat[1, 3]
    # Check for gimbal lock
    sy = np.sqrt(Hmat[0, 0]**2 + Hmat[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        # Rotation around z-axis (Yaw)
        theta = np.arctan2(Hmat[1, 0], Hmat[0, 0])
    else:
        # Arbitrary value
        theta = 0
    # Return x, y, theta(degrees)
    return x, y, theta
    
# Generate a Homogenious Matrix using x, y, z, theta(degrees)
def gen_Hmat(x, y, z, theta_d):
	# Define rotation for 90 degrees about the z-axis
	theta = theta_d # Convert 90 degrees to radians
	cos_theta = np.cos(theta)
	sin_theta = np.sin(theta)
	
	# Construct the homogeneous transformation matrix
	T = np.array([
	    [cos_theta, -sin_theta, 0, x],
	    [sin_theta, cos_theta,  0, y],
	    [0,         0,          1, z],
	    [0,         0,          0, 1]
	])
	
	# Return Homogeneous Transformation Matrix
	return T

# calculate the p using the change in delta x, delta y
def find_p(dx, dy):
    return ((dx**2 + dy**2)**0.5)

# calculate the p using the change in delta x, delta y, theta
def find_alpha(theta, dx, dy):
    # if dx == 0:
    #     if dy > 0:
    #         return 90  # 90 degrees if dy is positive
    #     elif dy < 0:
    #         return 270  # 270 degrees if dy is negative
    #     else:
    #         return np.nan  # Undefined angle if both are zero
    # else:
    return -theta + np.arctan2(dy, dx)
    
# calculate the p using the change in delta x, delta y, theta   
def find_beta(theta, alpha):
    return -(alpha + theta)
    
# calculate linear and angular velocity using Kp, Ka, Kb along within p, alpha, beta
def calc_v_w(Kp, Ka, Kb, dx, dy, theta):
    # Calculate the Error parameters
    p = find_p(dx, dy)
    alpha = find_alpha(theta, dx, dy)
    beta = find_beta(theta, alpha)
    # K constraints will different between Gazebo & Roomba
    # Gazebo- Kp= Ka= Kb=
    # Roomba- Kp= Ka= Kb=
    # stores matrix K contraints
    constraint_K = np.array([[Kp, 0, 0],[0, Ka, Kb]])
    # stores error calculations
    error_E = np.array([[p],[alpha],[beta]])
    # Perform the multiplication
    v, w = np.matmul(constraint_K, error_E)
    # Convert angular Velocity to Radians
    # w = w
    # returns linear & angular velocity
    return v, w
		
def check_odom(msg):
    # Get x and y positions from the odometry message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # Get orientation in quaternion form
    orientation_q = msg.pose.pose.orientation
    qx = orientation_q.x
    qy = orientation_q.y
    qz = orientation_q.z
    qw = orientation_q.w

    # Convert quaternion to euler angles (yaw represents theta) change to use tf
    (roll, pitch, theta) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    # siny_cosp = 2 * (qw * qz + qx * qy)
    # cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    # theta = np.arctan2(siny_cosp, cosy_cosp)

    # Print or log x, y, and theta
    # rospy.loginfo("Position -> x: %.2f, y: %.2f, theta: %.2f radians" % (x, y, theta))
    
	# # 1) Hard Coded - x, y, theta : (Destination Matrix) in reference to Source Matrix
    s_Hmat_d = gen_Hmat(2, 1, 0, 1.57)
	
    # -Loop Start
    # 2) Read Odometry - (Current Matrix) in reference to Source
    s_Hmat_c = gen_Hmat(x, y, z, theta)
    
    # 3) Compute x, y, theta
    # Inverse (Current Matrix) in refernece to Source
    c_Hmat_s = np.linalg.inv(s_Hmat_c)
    # (Current Matrix  multiplied by Destination Matrix) in reference to Source
    # (Destination Matrix) in Reference to Current
    c_Hmat_d = np.matmul(c_Hmat_s, s_Hmat_d)
    # Extract x, y, theta from Matrix
    dx = extract_Mat(c_Hmat_d)[0]
    dy = extract_Mat(c_Hmat_d)[1]
    dtheta = extract_Mat(c_Hmat_d)[2]

    # 4) Convert to p & b, Compute control
    # Find p
    p = find_p(dx, dy)
    # Find alpha
    alpha = find_alpha(dtheta, dx, dy)
    # Find beta
    beta = find_beta(dtheta, alpha)
    # Input K Constraints
    Kp = 0.1
    Ka = 0.05
    Kb = -0.01
    # Compute Linear & Angular Velocities
    v, w = calc_v_w(Kp, Ka, Kb, dx, dy, dtheta)
    # Debug Values : The positions arent accurate to odometry
    print(f"CURRENT MATRIX COORD - x:{x} y:{y} theta:{theta}")
    print(f"DISTANCE MATRIX COORD - dx:{dx} dy:{dy} dtheta:{dtheta}")
    print(f"v:{v} w:{w}")
    print(f"p:{p} beta:{beta} alpha:{alpha}\n")
    
    # Loop till destination mets
    # if w < 0.01:
    # Create a Twist message for linear and angular velocity
    velocity_cmd = Twist()
    # Move robot
    # Forward linear velocity (e.g., 0.5 m/s)
    velocity_cmd.linear.x = v
    velocity_cmd.angular.z = w
    motor_pub.publish(velocity_cmd)
    rospy.sleep(0.1)
	# -Loop End
    
def motor_controller():
    rospy.init_node('motor_controller', anonymous=True)
    # Get Current Matrix from Odometry
    rospy.Subscriber('/odom', Odometry, check_odom, queue_size=1)
    rospy.spin()

# Initialize publisher for velocity commands
global motor_pub
motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
