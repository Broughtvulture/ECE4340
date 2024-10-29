import numpy as np

# Create a Length by Width Matrix
def make_Mat(x, y, value=None):
    return np.full((x, y), value)

# Create a Homogenious Matrix using Rotation & Translation Matrix
def make_Hmat(rot, trans):
    # Attach translation to homogenious matrix
    homo = np.hstack((rot, trans))
    # Create scaler matrix
    scaler = createMatrix(1, 4, 0)
    # Set last value to 1
    scaler[0, -1] = 1
    # Attach scaler to homogenious matrix
    homo = np.vstack((homo, scaler))
    # Return homogenious matrix
    return homo

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
def calc_v_w:(Kp, Ka, Kb, p, alpha, beta):
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

# def grab_theta()

# rot = createMatrix(3, 3, 3)
# tran = createMatrix(3, 1, 2)

# Hard Code x, y, theta
# -Loop Start
# Compute x, y, theta
# Find p, alpha, beta
# Move robot
# Sleep
# Read Odometry
# Build source HMat from current
# Compute current HMat from destination
# Loop till destination met
# -Loop End

print(createHomoMatrix(rot, tran))
# print(type(matrix))
