import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import fsolve

class Arm3D:
    def __init__(self, a0, a1, a2, a3):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Arm parameters
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

        # Arm angles and positions
        self.theta1, self.theta2, self.theta3 = 0, 0, 0
        self.xyz1 = np.array([0, 0, 0])
        self.xyz2 = np.array([0, 0, 0])
        self.xyz3 = np.array([0, 0, 0])

        # Destination point
        self.destination = np.array([10, 10, 10])  # Fixed destination point

        # Initial update and print
        self.theta1, self.theta2, self.theta3 = self.inverse_kinematics_N3(self.a1, self.a2, self.a3,
                                                                            self.destination[0],
                                                                            self.destination[1],
                                                                            self.destination[2])
        self.update_arm_position()
        self.draw_3d_plot()
        print(f'Destination: {self.destination}, Angles: {self.theta1:.2f}, {self.theta2:.2f}, {self.theta3:.2f}')

    def inverse_kinematics_N3(self, L1, L2, L3, x, y, z, initial_guess=None):
        def equations(variables):
            theta_1, theta_2, theta_3 = variables
            eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
            eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
            eq3 = z
            return [eq1, eq2, eq3]

        if initial_guess is None:
            initial_guess = [5.06, -67.00, -12.00]

        result = fsolve(equations, initial_guess)

        theta_1, theta_2, theta_3 = result
        theta_1_deg = np.degrees(theta_1 % (2 * np.pi))
        theta_2_deg = np.degrees(theta_2 % (2 * np.pi))
        theta_3_deg = np.degrees(theta_3 % (2 * np.pi))

        if abs(theta_1_deg) > 180:
            theta_1_deg = int(theta_1_deg - math.copysign(360, theta_1_deg))
        if abs(theta_2_deg) > 180:
            theta_2_deg = int(theta_2_deg - math.copysign(360, theta_2_deg))
        if abs(theta_3_deg) > 180:
            theta_3_deg = int(theta_3_deg - math.copysign(360, theta_3_deg))
        if theta_1_deg > 0:
            theta_1_deg = -theta_1_deg

        return theta_1_deg, theta_2_deg, theta_3_deg

    def update_arm_position(self):
        self.xyz1 = np.array([0, 0, self.a0])
        self.xyz2 = self.xyz1 + np.array([self.a1 * np.cos(np.radians(self.theta1)),
                                          self.a1 * np.sin(np.radians(self.theta1)),
                                          0])
        self.xyz3 = self.xyz2 + np.array([self.a2 * np.cos(np.radians(self.theta1 + self.theta2)),
                                          self.a2 * np.sin(np.radians(self.theta1 + self.theta2)),
                                          0])

    def draw_3d_plot(self):
        self.ax.clear()
        
        # Plot the arm
        self.ax.plot([self.xyz1[0], self.xyz2[0], self.xyz3[0]],
                     [self.xyz1[1], self.xyz2[1], self.xyz3[1]],
                     [self.xyz1[2], self.xyz2[2], self.xyz3[2]], marker='o', linestyle='-')

        # Plot the destination point
        self.ax.scatter(self.destination[0], self.destination[1], self.destination[2], c='red', marker='o')
        
        # Plot the line from (0, 0, 0) to (0, 0, self.a0)
        self.ax.plot([0, 0], [0, 0], [0, self.a0], color='black', linestyle='solid')

        # Set axis limits
        self.ax.set_xlim([-20, 20])
        self.ax.set_ylim([-20, 20])
        self.ax.set_zlim([0, 20])

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.pause(0.01)

    def run(self):
        plt.ion()
        while plt.fignum_exists(self.fig.number):
            plt.pause(0.1)  # Add a short pause to allow the figure to be displayed
            break  # Exit the loop after printing once

        plt.ioff()
        plt.show()

if __name__ == "__main__":
    arm_instance = Arm3D(10, 13, 12, 12.5)
    arm_instance.run()
