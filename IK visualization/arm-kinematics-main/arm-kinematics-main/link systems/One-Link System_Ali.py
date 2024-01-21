import sys
import math
import pygame
import numpy as np

def inverse_kinematics_N1(x, y):
    theta = math.atan2(y,x)
    return math.degrees(theta)

# Initialize Pygame
pygame.init()

# Screen dimensions
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('One-Link System Animation')

# Colors
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
blue = (0, 0, 255)

# Link lengths
a1 = 100
x1 = 0
y1 = 0
theta1 = 0

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
        theta1 = inverse_kinematics_N1(mouse_x - width // 2, (mouse_y - height // 2))

        # Convert angles to radians
        theta1_rad = math.radians(theta1)

        # Calculate positions of points
        x1 = a1 * math.cos(theta1_rad) + width // 2
        y1 = a1 * math.sin(theta1_rad) + height // 2


    screen.fill(white)

    # Draw links and joints
    pygame.draw.line(screen, black, (width // 2, height // 2), (x1, y1), 5)
    pygame.draw.circle(screen, blue, (width // 2, height // 2), 8)
    pygame.draw.circle(screen, black, (int(x1), int(y1)), 8)
 

    # Display text on screen
    text_x = font.render(f"x: {mouse_x - width // 2}", True, black)
    text_y = font.render(f"y: {mouse_y - height // 2}", True, black)
    text_theta1 = font.render(f"Theta1: {(theta1):.2f} degrees", True, black)


    screen.blit(text_x, (10, 50))
    screen.blit(text_y, (10, 70))
    screen.blit(text_theta1, (10, 10))


    pygame.display.flip()

pygame.quit()
sys.exit()
