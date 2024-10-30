#!/usr/bin/env python3
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

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
        theta = np.degrees(np.arctan2(Hmat[1, 0], Hmat[0, 0]))  
    else:
        # Arbitrary value
        theta = 0
    # Return x, y, theta(degrees)
    return x, y, theta
    
# Generate a Homogenious Matrix using x, y, z, theta(degrees)
def gen_Hmat(x, y, z, theta_d):
	# Define rotation for 90 degrees about the z-axis
	theta = np.radians(theta_d)  # Convert 90 degrees to radians
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
    return ((dx**2, dy**2)**0.5)

# calculate the p using the change in delta x, delta y, theta
def find_alpha(theta, dx, dy):
    return (-theta + np.degrees(np.arctan(dy/dx)))
    
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
    v, w = np.dot(A, B)
    # returns linear & angular velocity
    return v, w
		
def odom_callback(msg):
    # Get x and y positions from the odometry message
    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Get orientation in quaternion form
    orientation_q = msg.pose.pose.orientation
    qx = orientation_q.x
    qy = orientation_q.y
    qz = orientation_q.z
    qw = orientation_q.w

    # Convert quaternion to euler angles (yaw represents theta)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    theta = math.atan2(siny_cosp, cosy_cosp)

    # Print or log x, y, and theta
    # rospy.loginfo("Position -> x: %.2f, y: %.2f, theta: %.2f radians" % (x, y, theta))

	# 1) Hard Code x, y, theta
    destination_Hmat = gen_Hmat(2, 1, 0, 90)
	
	# -Loop Start
	# Compute x, y, theta, 1, 2, 90 deg
	
	# 2) Find p, alpha, beta
    dx = x - extract_Mat(destination_Hmat)[0]
    dy = y - extract_Mat(destination_Hmat)[1]
    dtheta = theta - extract_Mat(destination_Hmat)[2]
    Kp = 1
    Ka = 1
    Kb = 1
    v, w = calc_v_w(Kp, Ka, Kb, dx, dy, dtheta)
	
    # Create a Twist message for linear and angular velocity
    velocity_cmd = Twist()
    # Forward linear velocity (e.g., 0.5 m/s)
    velocity_cmd.linear.x = v  
    # Angular velocity (e.g., 0.2 rad/s)
    velocity_cmd.angular.z = w  
	# 3) Move robot
    cmd_vel_pub.publish(velocity_cmd)
	
	# 4) Sleep
    rospy.sleep(1)
	# 5) Read Odometry: Only needed to compute dx, dy, theta changes [Done at (2)]
	
    # Build source HMat from current
    source_Hmat = gen_Hmat(0, 0, 0, 0)
    # Compute current HMat from destination
    current_Hmat = np.linalg.inv(source_Hmat) * destination_Hmat
    # Loop till destination met
	# -Loop End
		
def odom_listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

# Initialize publisher for velocity commands
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
