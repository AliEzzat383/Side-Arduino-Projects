import sys
import math
import pygame
import numpy as np
from scipy.optimize import fsolve
def inverse_kinematics_N1(L1, theta, x, y):
    return math.degrees(math.atan2(y - (L1 * math.sin(theta)),x - (L1 * math.cos(theta))))

def inverse_kinematics_N2(L1,L2,x, y):
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
    theta_1_deg = np.degrees(theta_1) % 360
    theta_2_deg = np.degrees(theta_2) % 360
    if theta_1_deg > 180:
        theta_1_deg = theta_1_deg - 360
    if theta_2_deg > 180:
        theta_2_deg = theta_2_deg - 360
        
    return theta_1_deg, theta_2_deg
    
def inverse_kinematics_N3(L1, L2, L3, x, y,initial_theta_1=0,initial_theta_2=0,initial_theta_3=0):
    def equations(variables):
        theta_1, theta_2, theta_3 = variables
        eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
        eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
        return [eq1, eq2, 0]  # Adding a placeholder equation to match the number of variables

    initial_guess = [initial_theta_1, initial_theta_2, initial_theta_3]  # Initial guess for theta_1, theta_2, and theta_3
    theta_1, theta_2, theta_3 = fsolve(equations, initial_guess)

    # Normalize angles to be within 0 to 2*pi (360 degrees)
    theta_1 = theta_1 % (2 * np.pi)
    theta_2 = theta_2 % (2 * np.pi)
    theta_3 = theta_3 % (2 * np.pi)
    theta_1_deg = np.degrees(theta_1)
    theta_2_deg = np.degrees(theta_2)
    theta_3_deg = np.degrees(theta_3)
    if theta_1_deg > 180:
        theta_1_deg = theta_1_deg - 360
    if theta_2_deg > 180:
        theta_2_deg = theta_2_deg - 360
    if theta_3_deg > 180:
        theta_3_deg = theta_3_deg - 360
    return theta_1_deg, theta_2_deg, theta_3_deg

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
a1 = 100
a2 = 70
a3 = 50

# Font setup
font = pygame.font.SysFont(None, 24)

# Initialize variables for positions of points and angles
x1, y1, x2, y2, x3, y3 = 0, 0, 0, 0, 0, 0
theta1, theta2, theta3 = 0, 0, 0

# Main loop
running = True
prev_mouse_pos = (0, 0)  # Store the previous mouse position

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get current mouse position
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Check if mouse position changed
    if (mouse_x, mouse_y) != prev_mouse_pos:
        # Update the previous mouse position
        prev_mouse_pos = (mouse_x, mouse_y)

        loc_x=(mouse_x - width // 2)
        loc_y=(mouse_y - height // 2)
        if loc_y>0:
            loc_y=-(mouse_y - height // 2)

        # Calculate inverse kinematics for mouse position
        theta1, theta2, theta3 = inverse_kinematics_N3(a1, a2, a3, loc_x, loc_y)
        if theta1>0:
            theta1=-theta1
        if theta2<-90:
            theta2=-90
        if theta3<-90:
            theta3=-90

        # Calculate positions of points
        x1 = a1 * math.cos(math.radians(theta1)) + width // 2
        y1 = a1 * math.sin(math.radians(theta1)) + height // 2
        x2 = x1 + a2 * math.cos(math.radians(theta1) + math.radians(theta2))
        y2 = y1 + a2 * math.sin(math.radians(theta1) + math.radians(theta2))
        x3 = x2 + a3 * math.cos(math.radians(theta1) + math.radians(theta2) + math.radians(theta3))
        y3 = y2 + a3 * math.sin(math.radians(theta1) + math.radians(theta2) + math.radians(theta3))

    screen.fill(white)

    # Draw links and joints
    pygame.draw.line(screen, black, (width // 2, height // 2), (x1, y1), 5)
    pygame.draw.line(screen, black, (x1, y1), (x2, y2), 5)
    pygame.draw.line(screen, black, (x2, y2), (x3, y3), 5)
    pygame.draw.circle(screen, blue, (width // 2, height // 2), 8)
    pygame.draw.circle(screen, black, (int(x1), int(y1)), 8)
    pygame.draw.circle(screen, red, (int(x2), int(y2)), 8)
    pygame.draw.circle(screen, red, (int(x3), int(y3)), 8)

    # Display text on screen
    text_x = font.render(f"x: {mouse_x - width // 2}", True, black)
    text_y = font.render(f"y: {mouse_y - height // 2}", True, black)
    text_theta1 = font.render(f"Theta1: {theta1:.2f} degrees", True, black)
    text_theta2 = font.render(f"Theta2: {theta2:.2f} degrees", True, black)
    text_theta3 = font.render(f"Theta3: {theta3:.2f} degrees", True, black)

    screen.blit(text_x, (10, 70))
    screen.blit(text_y, (10, 90))
    screen.blit(text_theta1, (10, 10))
    screen.blit(text_theta2, (10, 30))
    screen.blit(text_theta3, (10, 50))

    pygame.display.flip()

pygame.quit()
sys.exit()
