#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
import math
import time

# http://vigir.ee.missouri.edu/~gdesouza/ece4340/index.htm

# Initialize publisher for velocity commands, P tracker
global motor_pub
# Store Odometry Iterations
odom_counter = 0
# Store Previous Odometry Calculations
px, py, ptheta = 0, 0, 0
# Store Goal Reached Status
goal_reached = False

motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Prevent Angles from Falling Outside of -Pi & +Pi range
def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

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
    theta = np.arctan2(y, x)
    return x, y, theta
    
# Generate a Homogenious Matrix using x, y, z, theta(radians)
def gen_Hmat(x, y, z, theta):
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
    return normalize_angle(-theta + np.arctan2(dy, dx))
    
# calculate the p using the change in delta x, delta y, theta   
def find_beta(theta, alpha):
    return normalize_angle(-(alpha + theta))
    # # Store Math Function: Pi / 2
    # pi_2 = math.pi / 2
    # if alpha > -pi_2 and alpha < pi_2:
    #     return -(alpha + theta)
    # else:
    #     return theta
    
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
    # constraint_K = np.array([[Kp, 0, 0],[0, Ka, Kb]])
    # stores error calculations
    # error_E = np.array([[p],[alpha],[beta]])
    # Perform the multiplication
    # v, w = np.matmul(constraint_K, error_E)
    # Perform the simplified multiplication
    v, w = Kp*p, Ka*alpha + Kb*beta
    # returns linear & angular velocity
    return v, w
		
def check_odom(msg):
    # Setup Robot Counter, for checking iterations of Odometry Reading
    global odom_counter, px, py, ptheta, goal_reached

    # Check Goal Reached Status
    if goal_reached:
        # Stop Checking Odometry
        return
        #!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
import math
import time

# http://vigir.ee.missouri.edu/~gdesouza/ece4340/index.htm

# Initialize publisher for velocity commands, P tracker
global motor_pub
# Store Odometry Iterations
odom_counter = 0
# Store Previous Odometry Calculations
px, py, ptheta = 0, 0, 0
# Store Goal Reached Status
goal_reached = False

motor_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Prevent Angles from Falling Outside of -Pi & +Pi range
def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

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
    theta = np.arctan2(y, x)
    return x, y, theta
    
# Generate a Homogenious Matrix using x, y, z, theta(radians)
def gen_Hmat(x, y, z, theta):
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
    return normalize_angle(-theta + np.arctan2(dy, dx))
    
# calculate the p using the change in delta x, delta y, theta   
def find_beta(theta, alpha):
    return normalize_angle(-(alpha + theta))
    # # Store Math Function: Pi / 2
    # pi_2 = math.pi / 2
    # if alpha > -pi_2 and alpha < pi_2:
    #     return -(alpha + theta)
    # else:
    #     return theta
    
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
    # constraint_K = np.array([[Kp, 0, 0],[0, Ka, Kb]])
    # stores error calculations
    # error_E = np.array([[p],[alpha],[beta]])
    # Perform the multiplication
    # v, w = np.matmul(constraint_K, error_E)
    # Perform the simplified multiplication
    v, w = Kp*p, Ka*alpha + Kb*beta
    # returns linear & angular velocity
    return v, w
		
def check_odom(msg):
    # Setup Robot Counter, for checking iterations of Odometry Reading
    global odom_counter, px, py, ptheta, goal_reached

    # Check Goal Reached Status
    if goal_reached:
        # Stop Checking Odometry
        return
        
    # Get x and y positions from the odometry message
    ox = msg.pose.pose.position.x
    oy = msg.pose.pose.position.y
    oz = msg.pose.pose.position.z

    # Get orientation in quaternion form
    orientation_q = msg.pose.pose.orientation

    # Convert quaternion to euler angles (yaw represents theta) change to use tf
    (roll, pitch, otheta) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
	# 1) Hard Coded - x, y, z, theta : (Destination Matrix) in reference to Source Matrix
    s_Hmat_d = gen_Hmat(1, 0, 0, np.radians(0))
    # Extract x, y, theta from Destination
    ddx, ddy, ddtheta = extract_Mat(s_Hmat_d)
	
    # -Loop Start
    # 2) Read Odometry - (Current Matrix) in reference to Source
    # Check for Odometry Iteration
    if odom_counter >= 0:
        # 0) Hard Coded - (Source Matrix) in reference to Destination
        # Extract x, y, theta from Odometry
        dx = ddx - ox
        dy = ddy - oy
        dtheta = otheta - ddtheta
    # else:
    #     # Odometry Iteration > 0
    #     # Calculate Difference/Distance from Last to Current Position
    #     dx = ddx - ox - px
    #     dy = ddy - oy - py
    #     dtheta = otheta - ddtheta
    # Calculate (Current Matrix) in reference to Source
    s_Hmat_c = gen_Hmat(dx, dy, 0, dtheta)
    
    # 3) Compute x, y, theta
    # Inverse (Current Matrix) in reference to Source
    c_Hmat_s = np.linalg.inv(s_Hmat_c)
    # (Current Matrix  multiplied by Destination Matrix) in reference to Source
    # (Destination Matrix) in Reference to Current
    c_Hmat_d = np.matmul(c_Hmat_s, s_Hmat_d)

    # 4) Convert to p & b, Compute control
    # Find p
    p = find_p(dx, dy)
    # Find alpha
    alpha = find_alpha(dtheta, dx, dy)
    # Find beta
    beta = find_beta(dtheta, alpha)
    # # Input K Constraints
    Kp, Ka, Kb = 0.1, 0.15, -0.01
    # Compute Linear & Angular Velocities
    v, w = calc_v_w(Kp, Ka, Kb, dx, dy, dtheta)
    # # Debug Values : The positions arent accurate to odometry
    print(f"ODOM MATRIX COORD - {ox} {oy} {otheta}")
    print(f"CURRENT MATRIX COORD - {extract_Mat(s_Hmat_c)}")
    print(f"DELTA MATRIX COORD - {extract_Mat(c_Hmat_d)}")
    print(f"LOCATION MATRIX COORD - {extract_Mat(s_Hmat_d)}")
    print(f"INV MATRIX COORD - {extract_Mat(c_Hmat_s)}")
    print(f"v:{v} w:{w}")
    print(f"p:{p} beta:{beta} alpha:{alpha}\n")
    print(f"Counter: #{odom_counter}\n")
    odom_counter = odom_counter + 1
    
    # Loop till destination mets
    # Create a Twist message for linear and angular velocity
    velocity_cmd = Twist()
    # if p > 0.2:  # Start slowing down
        # v *= 0.2
        # w /= 0.2
    # Move robot
    velocity_cmd.linear.x = v
    velocity_cmd.angular.z = w
    # Rotate robot
    # if v <= 0.03:
    #     velocity_cmd.linear.x = 0
    # Send Velocity Data to Robot
    motor_pub.publish(velocity_cmd)
    rospy.sleep(0.1)

    # Store Previous x, y, theta
    # px = dx
    # py = dy
    # ptheta = dtheta
    # Break out of Loop
    if p <= 0.015:
        # Notify Goal Reached
        rospy.loginfo("Goal reached! Stopping the robot.")
        goal_reached = True
        # Stop Robot
        velocity_cmd.linear.x = 0
        velocity_cmd.angular.z = 0
        motor_pub.publish(velocity_cmd)
        rospy.sleep(0.1)
        

def start_odom_sub():
    # Sets Up Odometry Subscriber
    global odom_sub
    # Subscriber Starts Functionality
    odom_sub = rospy.Subscriber('/odom', Odometry, check_odom, queue_size=1)

def motor_controller():
    rospy.init_node('motor_controller', anonymous=True)
    start_odom_sub()
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
    # Get x and y positions from the odometry message
    ox = msg.pose.pose.position.x
    oy = msg.pose.pose.position.y
    oz = msg.pose.pose.position.z

    # Get orientation in quaternion form
    orientation_q = msg.pose.pose.orientation

    # Convert quaternion to euler angles (yaw represents theta) change to use tf
    (roll, pitch, otheta) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
	# 1) Hard Coded - x, y, z, theta : (Destination Matrix) in reference to Source Matrix
    s_Hmat_d = gen_Hmat(1, 0, 0, np.radians(0))
    # Extract x, y, theta from Destination
    ddx, ddy, ddtheta = extract_Mat(s_Hmat_d)
	
    # -Loop Start
    # 2) Read Odometry - (Current Matrix) in reference to Source
    # Check for Odometry Iteration
    if odom_counter >= 0:
        # 0) Hard Coded - (Source Matrix) in reference to Destination
        # Extract x, y, theta from Odometry
        dx = ddx - ox
        dy = ddy - oy
        dtheta = otheta - ddtheta
    # else:
    #     # Odometry Iteration > 0
    #     # Calculate Difference/Distance from Last to Current Position
    #     dx = ddx - ox - px
    #     dy = ddy - oy - py
    #     dtheta = otheta - ddtheta
    # Calculate (Current Matrix) in reference to Source
    s_Hmat_c = gen_Hmat(dx, dy, 0, dtheta)
    
    # 3) Compute x, y, theta
    # Inverse (Current Matrix) in reference to Source
    c_Hmat_s = np.linalg.inv(s_Hmat_c)
    # (Current Matrix  multiplied by Destination Matrix) in reference to Source
    # (Destination Matrix) in Reference to Current
    c_Hmat_d = np.matmul(c_Hmat_s, s_Hmat_d)

    # 4) Convert to p & b, Compute control
    # Find p
    p = find_p(dx, dy)
    # Find alpha
    alpha = find_alpha(dtheta, dx, dy)
    # Find beta
    beta = find_beta(dtheta, alpha)
    # # Input K Constraints
    Kp, Ka, Kb = 0.1, 0.15, -0.01
    # Compute Linear & Angular Velocities
    v, w = calc_v_w(Kp, Ka, Kb, dx, dy, dtheta)
    # # Debug Values : The positions arent accurate to odometry
    print(f"ODOM MATRIX COORD - {ox} {oy} {otheta}")
    print(f"CURRENT MATRIX COORD - {extract_Mat(s_Hmat_c)}")
    print(f"DELTA MATRIX COORD - {extract_Mat(c_Hmat_d)}")
    print(f"LOCATION MATRIX COORD - {extract_Mat(s_Hmat_d)}")
    print(f"INV MATRIX COORD - {extract_Mat(c_Hmat_s)}")
    print(f"v:{v} w:{w}")
    print(f"p:{p} beta:{beta} alpha:{alpha}\n")
    print(f"Counter: #{odom_counter}\n")
    odom_counter = odom_counter + 1
    
    # Loop till destination mets
    # Create a Twist message for linear and angular velocity
    velocity_cmd = Twist()
    # if p > 0.2:  # Start slowing down
        # v *= 0.2
        # w /= 0.2
    # Move robot
    velocity_cmd.linear.x = v
    velocity_cmd.angular.z = w
    # Rotate robot
    # if v <= 0.03:
    #     velocity_cmd.linear.x = 0
    # Send Velocity Data to Robot
    motor_pub.publish(velocity_cmd)
    rospy.sleep(0.1)

    # Store Previous x, y, theta
    # px = dx
    # py = dy
    # ptheta = dtheta
    # Break out of Loop
    if p <= 0.015:
        # Notify Goal Reached
        rospy.loginfo("Goal reached! Stopping the robot.")
        goal_reached = True
        # Stop Robot
        velocity_cmd.linear.x = 0
        velocity_cmd.angular.z = 0
        motor_pub.publish(velocity_cmd)
        rospy.sleep(0.1)
        

def start_odom_sub():
    # Sets Up Odometry Subscriber
    global odom_sub
    # Subscriber Starts Functionality
    odom_sub = rospy.Subscriber('/odom', Odometry, check_odom, queue_size=1)

def motor_controller():
    rospy.init_node('motor_controller', anonymous=True)
    start_odom_sub()
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
