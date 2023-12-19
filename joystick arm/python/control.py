import keyboard
import time
import serial
import pygame

value1 = 90  # base
value2 = 90  # shoulder
value3 = 0   # elbow
value4 = 90  # wrist_y
value5 = 0   # wrist_x
value6 = 30  # clamp
angle = 1
step = 30  # Initial step value
delay = 500  # Delay in milliseconds

previous_value1 = value1
previous_value2 = value2
previous_value3 = value3
previous_value4 = value4
previous_value5 = value5
previous_value6 = value6

pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

print("Press the LEFT arrow key to increment value1 (base), RIGHT arrow key to decrement value1.")
print("Press the UP arrow key to increment value2 (shoulder), DOWN arrow key to decrement value2.")
print("Press 'w' to increment value3 (elbow), 's' to decrement value3.")
print("Press 'a' to increment value4 (wrist_y), 'd' to decrement value4 (wrist_y).")
print("Press 'f' to increment value5 (wrist_x), 'j' to decrement value5 (wrist_x).")
print("Press 'g' to increment value6 (clamp), 'h' to decrement value6 (clamp).")
print("Press 'i' to increase step size, 'k' to decrease step size.")
print("Press ESC to exit.")

ser = serial.Serial('COM3', 115200)
running = True

while running:
    base = value1 * angle
    shoulder = value2 * angle
    elbow = value3 * angle
    wrist_y = value4 * angle
    wrist_x = value5 * angle
    clamp = value6

    if (
        value1 != previous_value1
        or value2 != previous_value2
        or value3 != previous_value3
        or value4 != previous_value4
        or value5 != previous_value5
        or value6 != previous_value6
    ):
        print(f"Value1: {value1}, Value2: {value2}, Value3: {value3}, Value4: {value4}, Value5: {value5}, Value6: {value6}, Step: {step}")
        ser.write(f"{base},{shoulder},{elbow},{wrist_y},{wrist_x},{clamp}\n".encode())

    previous_value1 = value1
    previous_value2 = value2
    previous_value3 = value3
    previous_value4 = value4
    previous_value5 = value5
    previous_value6 = value6

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        time.sleep(delay / 1000.0)
        value2 += step
        if value2 > 180:
            value2 = 150
    elif keys[pygame.K_DOWN]:
        time.sleep(delay / 1000.0)
        value2 -= step
        if value2 < 0:
            value2 = 30
    elif keys[pygame.K_LEFT]:
        time.sleep(delay / 1000.0)
        value1 += step
        if value1 > 180:
            value1 = 180
    elif keys[pygame.K_RIGHT]:
        time.sleep(delay / 1000.0)
        value1 -= step
        if value1 < 0:
            value1 = 0
    elif keys[pygame.K_w]:
        time.sleep(delay / 1000.0)
        value3 += step
        if value3 > 180:
            value3 = 180
    elif keys[pygame.K_s]:
        time.sleep(delay / 1000.0)
        value3 -= step
        if value3 < 0:
            value3 = 0
    elif keys[pygame.K_a]:
        time.sleep(delay / 1000.0)
        value4 += step
        if value4 > 180:
            value4 = 180
    elif keys[pygame.K_d]:
        time.sleep(delay / 1000.0)
        value4 -= step
        if value4 < 0:
            value4 = 0
    elif keys[pygame.K_f]:
        time.sleep(delay / 1000.0)
        value5 += step
        if value5 > 180:
            value5 = 180
    elif keys[pygame.K_j]:
        time.sleep(delay / 1000.0)
        value5 -= step
        if value5 < 0:
            value5 = 0
    elif keys[pygame.K_g]:
        time.sleep(delay / 1000.0)
        value6 += step
        if value6 > 90:
            value6 = 90
    elif keys[pygame.K_h]:
        time.sleep(delay / 1000.0)
        value6 -= step
        if value6 < 30:
            value6 = 30
    elif keys[pygame.K_i]:
        time.sleep(delay / 1000.0)
        step += 10
        if step > 90:
            step = 90
    elif keys[pygame.K_k]:
        time.sleep(delay / 1000.0)
        step -= 10
        if step < 10:
            step = 10
    elif keys[pygame.K_ESCAPE]:
        running = False

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
ser.close()
