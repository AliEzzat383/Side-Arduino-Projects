import numpy as np
from scipy.optimize import minimize

def forward_kinematics(theta, L1, L2, L3):
    x = L1 * np.cos(theta[0]) + L2 * np.cos(theta[0] + theta[1]) + L3 * np.cos(np.sum(theta))
    y = L1 * np.sin(theta[0]) + L2 * np.sin(theta[0] + theta[1]) + L3 * np.sin(np.sum(theta))
    return np.array([x, y])

def inverse_kinematics_objective(theta, target_position, L1, L2, L3):
    end_effector_position = forward_kinematics(theta, L1, L2, L3)
    return np.sum((end_effector_position - target_position)**2)

def inverse_kinematics_numerical(target_position, L1, L2, L3, initial_guess=None):
    if initial_guess is None:
        initial_guess = np.zeros(3)

    result = minimize(
        inverse_kinematics_objective,
        initial_guess,
        args=(target_position, L1, L2, L3),
        method='L-BFGS-B'  # You can choose other optimization methods as well
    )

    if result.success:
        return result.x
    else:
        raise ValueError("Optimization failed")

# Example usage:
x_target = 2.0
y_target = 3.0
L1 = 1.0
L2 = 2.0
L3 = 1.5

target_position = np.array([x_target, y_target])

joint_angles = inverse_kinematics_numerical(target_position, L1, L2, L3)
print("Joint angles:", joint_angles)
