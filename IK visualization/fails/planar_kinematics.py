import sys
import math
import pygame
import numpy as np
from scipy.optimize import fsolve
from scipy.optimize import root

# def inverse_kinematics_N(L1, L2, L3, x, y):
#     def equations(variables):
#         theta_1, theta_2, theta_3 = variables
#         eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
#         eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
#         return [eq1, eq2, 0]  # Added a third equation with a constant value (can be any value)

#     initial_guess = [0.0, 0.0, 0.0]  # Initial guess for theta_1, theta_2, and theta_3
#     theta_1, theta_2, theta_3 = fsolve(equations, initial_guess)

#     # Normalize angles to be within 0 to pi (0 to 180 degrees)
#     theta_1 = theta_1 % (np.pi)
#     theta_2 = theta_2 % (np.pi)
#     theta_3 = theta_3 % (np.pi)
#     theta_1_deg = np.degrees(theta_1)
#     theta_2_deg = np.degrees(theta_2)
#     theta_3_deg = np.degrees(theta_3)
#     return theta_1_deg, theta_2_deg, theta_3_deg
# 
# def inverse_kinematics_N(L1, L2, L3, x, y):
#     def equations(variables):
#         theta_1, theta_2, theta_3 = variables
#         eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
#         eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
#         return [eq1, eq2, 0]  # Added a third equation with a constant value (can be any value)

#     # initial_guess = [0.0, 0.0, 0.0]  # Initial guess for theta_1, theta_2, and theta_3
#     initial_guess = [math.radians(45), math.radians(45), math.radians(45)]  # Use reasonable initial values
#     # solution = fsolve(equations, initial_guess)
#     # solution = fsolve(equations, initial_guess, maxfev=2000)
# # After calculating the solution using root
#     solution = root(equations, initial_guess).x

#     # Convert angles to degrees
#     theta_1, theta_2, theta_3 = np.degrees(solution)

#     # Ensure angles are between 190 and 360 degrees
#     theta_1 = max(0, min(360, theta_1))
#     theta_2 = max(0, min(360, theta_2))
#     theta_3 = max(0, min(360, theta_3))

#     # Update the initial guess for the next iteration
#     initial_guess = np.radians([theta_1, theta_2, theta_3])

#     return theta_1, theta_2, theta_3
# 
def inverse_kinematics_N(L1, L2, L3, x, y, jac=None):
    def equations(variables):
        theta_1, theta_2, theta_3 = variables
        eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
        eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
        return [eq1, eq2, 0]

    initial_guess = [math.radians(45), math.radians(45), math.radians(45)]
    
    # Use transpose of Jacobian method if jac is provided
    if jac is not None:
        result = root(equations, initial_guess, jac=jac, method='hybr')
    else:
        result = root(equations, initial_guess, method='hybr')

    solution = result.x
    status = result.status

    # Convert angles to degrees
    theta_1, theta_2, theta_3 = np.degrees(solution)

    # Ensure angles are between 0 and 360 degrees
    theta_1 = max(0, min(360, theta_1))
    theta_2 = max(0, min(360, theta_2))
    theta_3 = max(0, min(360, theta_3))

    return theta_1, theta_2, theta_3, status



# Initialize Pygame
pygame.init()

# Screen dimensions
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Three-Link System Animation')

# Colors
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
blue = (0, 0, 255)

# Link lengths
L1 = 100
L2 = 100
L3 = 100  # Length of the new link

# Font setup
font = pygame.font.SysFont(None, 24)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Calculate inverse kinematics for mouse position using transpose of Jacobian
    theta1, theta2, theta3, status = inverse_kinematics_N(L1, L2, L3, mouse_x - width // 2, (mouse_y - height // 2), jac=None)

    # Convert angles to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)
    theta3_rad = math.radians(theta3)

    # Calculate positions of points
    x1 = L1 * math.cos(theta1_rad) + width // 2
    y1 = L1 * math.sin(theta1_rad) + height // 2
    x2 = x1 + L2 * math.cos(theta1_rad + theta2_rad)
    y2 = y1 + L2 * math.sin(theta1_rad + theta2_rad)
    x3 = x2 + L3 * math.cos(theta1_rad + theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta1_rad + theta2_rad + theta3_rad)

    screen.fill(white)

    # Draw links and joints
    pygame.draw.line(screen, black, (width // 2, height // 2), (x1, y1), 5)
    pygame.draw.line(screen, black, (x1, y1), (x2, y2), 5)
    pygame.draw.line(screen, black, (x2, y2), (x3, y3), 5)
    pygame.draw.circle(screen, blue, (width // 2, height // 2), 8)
    pygame.draw.circle(screen, black, (int(x1), int(y1)), 8)
    pygame.draw.circle(screen, black, (int(x2), int(y2)), 8)
    pygame.draw.circle(screen, red, (int(x3), int(y3)), 8)

    # Display angles text
    text_theta1 = font.render(f"Theta1: {math.degrees(theta1) % 360:.2f} degrees", True, black)
    text_theta2 = font.render(f"Theta2: {math.degrees(theta2) % 360:.2f} degrees", True, black)
    text_theta3 = font.render(f"Theta3: {math.degrees(theta3) % 360:.2f} degrees", True, black)
    screen.blit(text_theta1, (10, 10))
    screen.blit(text_theta2, (10, 30))
    screen.blit(text_theta3, (10, 50))

    pygame.display.flip()

pygame.quit()
sys.exit()
