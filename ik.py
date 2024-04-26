import numpy as np
from scipy.optimize import minimize

# Forward Kinematics function (example)
def forward_kinematics(q):
    # degree to radian
    q = q * np.pi / 180.

    # Link lengths
    a1 = 7.
    a2 = 12.35
    a3 = 12.
    a4 = 13.4

    # Trigonometric modeling to get the end-effector pose
    q[1] = np.pi - q[1]

    cL3 = a4 * np.cos(q[1] + (np.pi / 2. - q[2]) + (np.pi / 2. - q[3]))
    cL2 = cL3 + a3 * np.cos(q[1] + (np.pi / 2. - q[2]))
    cL = cL2 + a2 * np.cos(q[1])

    sL3 = a4 * np.sin(q[1] + (np.pi / 2. - q[2]) + (np.pi / 2. - q[3]))
    sL2 = sL3 + a3 * np.sin(q[1] + (np.pi / 2. - q[2]))
    sL = sL2 + a2 * np.sin(q[1])

    x = cL * np.cos(q[0])
    y = cL * np.sin(q[0])
    z = a1 + sL

    # for simplicity we only calculate the 3D positions
    # tx = q[0]
    # ty = q[1] + (np.pi / 2. - q[2]) + (np.pi / 2. - q[3])
    # tz = q[4]
    
    return np.array([x, y, z])#, tx, ty, tz])

# Error function to minimize (squared error between desired and computed end-effector pose)
def error_function(q, desired_pose): # q (joint angles) --FK--> xyz (gripper pos) <--error--> desired_xyz
    computed_pose = forward_kinematics(q)
    error = np.sum((computed_pose[:3] - desired_pose[:3])**2)# + 10*(computed_pose[3:] - desired_pose[3:])**2)
    return error

# Inverse Kinematics function
def inverse_kinematics(desired_pose, initial_guess=None):
    # Initial guess for joint angles
    if initial_guess is None:
        initial_guess = np.array([90, 90, 90, 90, 90])  # Modify this according to your robot's joint configuration
    
    # Bounds for joint angles (if applicable)
    bounds = [(0, 180), (15, 165), (0, 180), (0, 180), (0, 180)]  # Modify this according to your joint limits
    
    # Using optimization to minimize the error function to find joint angles
    result = minimize(error_function, initial_guess, args=(desired_pose,), bounds=bounds)
    
    if result.success:
        return result.x
    else:
        raise ValueError("IK optimization failed.")

# Example usage
if __name__ == "__main__":
    # Example desired end-effector pose
    desired_poses = [np.array([x, 15, 15]) for x in range(16, -16, -4)]  # Modify this according to your desired pose
    #, 0, -np.pi / 4., 0])
    # circular path
    # desired_poses = [np.array([10 + 3 * np.cos(np.pi * i / 8), 3 * np.sin(np.pi * i / 8), 15]) for i in range(9)]

    print(";".join([f'[{d[0]}, {d[1]}, {d[2]}]' for d in desired_poses]))

    joint_angles_ = []
    approximated_targets = []
    for desired_pose in desired_poses:
        # Compute inverse kinematics
        joint_angles = inverse_kinematics(desired_pose)
        joint_angles_.append(joint_angles)
        approximated_targets.append(forward_kinematics(joint_angles))

        # without scientific notation
        np.set_printoptions(suppress=True)
        
        # print("Computed joint angles:", joint_angles)
        # print("Desired target:", desired_pose)
        # print("Approximated target:", forward_kinematics(joint_angles))

        # print(f"Braccio.ServoMovement(1000, {joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}, {joint_angles[3]}, {joint_angles[4]}, 73);")
        # print("delay(3000);")

    # set precision
    print(";".join([f'[{d[0]:.2f}, {d[1]:.2f}, {d[2]:.2f}, {d[3]:.2f}, {d[4]:.2f}]' for d in joint_angles_]))
    print(";".join([f'[{d[0]:.2f}, {d[1]:.2f}, {d[2]:.2f}]' for d in approximated_targets]))