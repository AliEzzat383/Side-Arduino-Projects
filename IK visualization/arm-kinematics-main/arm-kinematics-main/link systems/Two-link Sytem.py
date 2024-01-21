import sys
import math
import pygame
import numpy as np
from scipy.optimize import fsolve

def inverse_kinematics_N1(L1=100, theta=0, x=0, y=0):
    offsetx = - (L1 * math.cos(theta))
    offsety = - (L1 * math.sin(theta))
    return math.degrees(math.atan2(y ,x ))

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
    theta_1_deg = np.degrees(theta_1)
    theta_2_deg = np.degrees(theta_2)
    if theta_1_deg > 180:
        theta_1_deg = theta_1_deg - 360
    if theta_2_deg > 180:
        theta_2_deg = theta_2_deg - 360
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
a2 = 50
x1 = 0
y1 = 0
x2 = 0
y2 = 0
theta1 = 0
theta2 = 0

# Font setup
font = pygame.font.SysFont(None, 24)

# Main loop
# Inside the main loop
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

        # Calculate inverse kinematics for mouse position
        theta1, theta2 = inverse_kinematics_N2(a1, a2, mouse_x - width // 2, (mouse_y - height // 2))
        
        if theta1 <= 0 :
            # Calculate positions of points
            x1 = a1 * math.cos(math.radians(theta1)) + width // 2
            y1 = a1 * math.sin(math.radians(theta1)) + height // 2
            x2 = x1 + a2 * math.cos(math.radians(theta1) + math.radians(theta2))
            y2 = y1 + a2 * math.sin(math.radians(theta1) + math.radians(theta2))
            
        elif theta1 > 0:
            theta1 =  theta1 * -1
            theta2 = inverse_kinematics_N1(a1, theta1, mouse_x - width // 2, mouse_y - height // 2)
            x1 = a1 * math.cos(math.radians(theta1)) + width // 2
            y1 = a1 * math.sin(math.radians(theta1)) + height // 2
            x2 = x1 + a2 * math.cos(math.radians(theta2)) 
            y2 = y1 + a2 * math.sin(math.radians(theta2)) 
        


    screen.fill(white)

    # Draw links and joints
    pygame.draw.line(screen, black, (width // 2, height // 2), (x1, y1), 5)
    pygame.draw.line(screen, black, (x1, y1), (x2, y2), 5)
    pygame.draw.circle(screen, blue, (width // 2, height // 2), 8)
    pygame.draw.circle(screen, black, (int(x1), int(y1)), 8)
    pygame.draw.circle(screen, red, (int(x2), int(y2)), 8)

    # Display text on screen
    text_x = font.render(f"x: {mouse_x - width // 2}", True, black)
    text_y = font.render(f"y: {mouse_y - height // 2}", True, black)
    text_theta1 = font.render(f"Theta1: {(theta1):.2f} degrees", True, black)
    text_theta2 = font.render(f"Theta2: {(theta2):.2f} degrees", True, black)

    screen.blit(text_x, (10, 50))
    screen.blit(text_y, (10, 70))
    screen.blit(text_theta1, (10, 10))
    screen.blit(text_theta2, (10, 30))

    pygame.display.flip()

pygame.quit()
sys.exit()