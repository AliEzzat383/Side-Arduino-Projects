import sys
import math
import pygame
import numpy as np
from scipy.optimize import fsolve

def inverse_kinematics_G(a1=1, a2=1,x=1,y=1):
    R = math.sqrt(x**2 + y**2)
    N = (a1**2 + a2**2 - R**2) / (2 * a1 * a2)

    if -1 <= N <= 1:
        theta2 = 180 - math.degrees(math.acos(N))
        
        k1 = a1 + a2 * math.cos(math.radians(theta2))
        k2 = a2 * math.sin(math.radians(theta2))
        
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return (theta1)%360,(theta2)%360
    else:
        print("No solution exists for given x, y coordinates.")

def inverse_kinematics_N(L1,L2,x, y):
    def equations(variables):
        theta_1, theta_2 = variables
        eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2)
        eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2)
        return [eq1, eq2]

    initial_guess = [0.0, 0.0]  # Initial guess for theta_1 and theta_2
    theta_1, theta_2 = fsolve(equations, initial_guess)

    # Normalize angles to be within 0 to 2*pi (360 degrees)
    theta_1 = theta_1 % (2 * np.pi)
    theta_2 = theta_2 % (2 * np.pi)
    theta_1_deg = np.degrees(theta_1)
    theta_2_deg = np.degrees(theta_2)
    return theta_1_deg, theta_2_deg
# Initialize Pygame
pygame.init()

# Screen dimensions
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Two-Link System Animation')

# Colors
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
blue = (0, 0, 255)

# Link lengths
a1 = 100
a2 = 100


# Font setup
font = pygame.font.SysFont(None, 24)

# Main loop
# Inside the main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Calculate inverse kinematics for mouse position
    theta1, theta2 = inverse_kinematics_N(a1, a2, mouse_x - width // 2, (mouse_y - height // 2))

    # Convert angles to radians
    theta1_rad = math.radians(theta1)
    theta2_rad = math.radians(theta2)

    # Calculate positions of points
    x1 = a1 * math.cos(theta1_rad) + width // 2
    y1 = a1 * math.sin(theta1_rad) + height // 2
    x2 = x1 + a2 * math.cos(theta1_rad + theta2_rad)
    y2 = y1 + a2 * math.sin(theta1_rad + theta2_rad)

    # Rest of your code remains unchanged



    screen.fill(white)

    # Draw links and joints
    pygame.draw.line(screen, black, (width // 2, height // 2), (x1, y1), 5)
    pygame.draw.line(screen, black, (x1, y1), (x2, y2), 5)
    pygame.draw.circle(screen, blue, (width // 2, height // 2), 8)
    pygame.draw.circle(screen, black, (int(x1), int(y1)), 8)
    pygame.draw.circle(screen, red, (int(x2), int(y2)), 8)

    # Display angles text
    text_theta1 = font.render(f"Theta1: {math.degrees(theta1)%360:.2f} degrees", True, black)
    text_theta2 = font.render(f"Theta2: {math.degrees(theta2)%360:.2f} degrees", True, black)
    screen.blit(text_theta1, (10, 10))
    screen.blit(text_theta2, (10, 30))

    pygame.display.flip()

    # Update angles for animation (you can modify these values)
    # theta1 += 0.0001 
    # theta2 += 0.0003 

pygame.quit()
sys.exit()