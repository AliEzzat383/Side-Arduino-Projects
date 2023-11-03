import keyboard
import time
import serial  # Import the pyserial library
import pygame

# Initialize five values for controlling the arm
value1 = 90  # base
value2 = 90  # shoulder
value3 = 0   # elbow
value4 = 90  # wrist
value5 = 30  # clamp
angle = 1
step = 10
delay = 500  # Delay in milliseconds

# Initialize previous values to track changes
previous_value1 = value1
previous_value2 = value2
previous_value3 = value3
previous_value4 = value4
previous_value5 = value5

# Initialize Pygame
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

print("Press the UP arrow key to increment value2 (shoulder), DOWN arrow key to decrement value2.")
print("Press the LEFT arrow key to increment value1 (base), RIGHT arrow key to decrement value1.")
print("Press 'w' to increment value3 (elbow), 's' to decrement value3.")
print("Press 'a' to increment value5 (clamp), 'd' to decrement value5 (clamp).")
print("Press 'f' to increment value4 (wrist), 'j' to decrement value4 (wrist).")
print("Press ESC to exit.")

# Initialize the serial connection
ser = serial.Serial('COM3', 115200)  # Replace 'COM3' with the correct COM port and baud rate
running = True

while running:
    # Calculate angles based on the current values
    base = value1 * angle
    shoulder = value2 * angle
    elbow = value3 * angle
    wrist = value4 * angle
    clamp = value5

    # Check if any of the values have changed
    if (
        value1 != previous_value1
        or value2 != previous_value2
        or value3 != previous_value3
        or value4 != previous_value4
        or value5 != previous_value5
    ):
        print(f"Value1: {value1}, Value2: {value2}, Value3: {value3}, Value4: {value4}, Value5: {value5}")
        # Send the values to the serial port, separated by commas
        ser.write(f"{base},{shoulder},{elbow},{wrist},{clamp}\n".encode())

    previous_value1 = value1
    previous_value2 = value2
    previous_value3 = value3
    previous_value4 = value4
    previous_value5 = value5

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value2 += step
        if value2 > 180:
            value2 = 180  # Limit value2 to a maximum of 180
    elif keys[pygame.K_DOWN]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value2 -= step
        if value2 < 0:
            value2 = 0  # Ensure value2 doesn't go below zero
    elif keys[pygame.K_LEFT]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value1 += step
        if value1 > 180:
            value1 = 180  # Limit value1 to a maximum of 180
    elif keys[pygame.K_RIGHT]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value1 -= step
        if value1 < 0:
            value1 = 0  # Ensure value1 doesn't go below zero
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
    elif keys[pygame.K_a]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value5 += step
        if value5 > 180:
            value5 = 180  # Limit value5 to a maximum of 180
    elif keys[pygame.K_d]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value5 -= step
        if value5 < 0:
            value5 = 0  # Ensure value5 doesn't go below zero
    elif keys[pygame.K_f]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value4 += step
        if value4 > 180:
            value4 = 180  # Limit value4 to a maximum of 180
    elif keys[pygame.K_j]:
        time.sleep(delay / 1000.0)  # Delay in seconds
        value4 -= step
        if value4 < 0:
            value4 = 0  # Ensure value4 doesn't go below zero
    elif keys[pygame.K_ESCAPE]:
        running = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
# Close the serial connection when done
ser.close()
