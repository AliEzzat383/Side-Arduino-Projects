import keyboard
import time
import serial  # Import the pyserial library
import pygame

# Initialize three values and step sizes for increment/decrement
value1 = 0
value2 = 0
value3 = 0
angle = 1
base = value1 * angle
shoulder = value2 * angle
elbow = value3 * angle
step = 10
delay = 500  # Delay in milliseconds

# Initialize previous values to track changes
previous_value1 = value1
previous_value2 = value2
previous_value3 = value3

# Initialize Pygame
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

print("Press the UP arrow key to increment value1, DOWN arrow key to decrement value1.")
print("Press the LEFT arrow key to increment value2, RIGHT arrow key to decrement value2.")
print("Press 'w' to increment value3, 's' to decrement value3.")
print("Press ESC to exit.")

# Initialize the serial connection
ser = serial.Serial('COM3', 115200)  # Replace 'COM3' with the correct COM port and baud rate
running = True

while running:
    # Calculate base, shoulder, and elbow based on the current values
    base = value1 * angle
    shoulder = value2 * angle
    elbow = value3 * angle

    # Check if any of the values have changed
    if value1 != previous_value1 or value2 != previous_value2 or value3 != previous_value3:
        print(f"Value1: {value1}, Value2: {value2}, Value3: {value3}")
        # Send base, shoulder, and elbow to the serial port, separated by commas
        ser.write(f"{base},{shoulder},{elbow}\n".encode())

    previous_value1 = value1
    previous_value2 = value2
    previous_value3 = value3

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value1 += step
        if value1 > 180:
            value1 = 180  # Limit value1 to a maximum of 180
    elif keys[pygame.K_DOWN]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value1 -= step
        if value1 < 0:
            value1 = 0  # Ensure value1 doesn't go below zero
    elif keys[pygame.K_a]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value2 += step
        if value2 > 180:
            value2 = 180  # Limit value2 to a maximum of 180
    elif keys[pygame.K_d]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value2 -= step
        if value2 < 0:
            value2 = 0  # Ensure value2 doesn't go below zero
    elif keys[pygame.K_w]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value3 += step
        if value3 > 180:
            value3 = 180  # Limit value3 to a maximum of 180
    elif keys[pygame.K_s]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value3 -= step
        if value3 < 0:
            value3 = 0  # Ensure value3 doesn't go below zero
    elif keys[pygame.K_ESCAPE]:
        running = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
# Close the serial connection when done
ser.close()
