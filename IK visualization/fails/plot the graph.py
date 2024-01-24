import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics(L1, L2, L3, theta1, theta2, theta3):
    x1 = L1 * np.cos(np.radians(theta1))
    x2 = x1 + L2 * np.cos(np.radians(theta1 + theta2))
    x3 = x2 + L3 * np.cos(np.radians(theta1 + theta2 + theta3))
    return x1, x2, x3

# Assuming some angles
theta1_values = np.arange(0, 181, 1)
theta2_values = np.zeros_like(theta1_values)
theta3_values = np.zeros_like(theta1_values)

# Assuming link lengths
L1 = 130
L2 = 120
L3 = 125

# Calculate x values for each joint
x1_values, x2_values, x3_values = forward_kinematics(L1, L2, L3, theta1_values, theta2_values, theta3_values)

# Plotting
plt.figure(figsize=(12, 6))

# Plot for joint 1
plt.subplot(131)
plt.plot(theta1_values, x1_values, label='Joint 1')
plt.title('Forward Kinematics: X1 vs Theta1')
plt.xlabel('Theta1 (degrees)')
plt.ylabel('X1 position')
plt.legend()
plt.grid(True)

# Plot for joint 2
plt.subplot(132)
plt.plot(theta1_values, x2_values, label='Joint 2')
plt.title('Forward Kinematics: X2 vs Theta1')
plt.xlabel('Theta1 (degrees)')
plt.ylabel('X2 position')
plt.legend()
plt.grid(True)

# Plot for joint 3
plt.subplot(133)
plt.plot(theta1_values, x3_values, label='End Effector (Joint 3)')
plt.title('Forward Kinematics: X3 vs Theta1')
plt.xlabel('Theta1 (degrees)')
plt.ylabel('X3 position')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
