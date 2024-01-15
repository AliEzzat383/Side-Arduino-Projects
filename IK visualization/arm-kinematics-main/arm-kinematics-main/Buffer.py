import time
import serial

path = []

ser = serial.Serial('COM3', 115200)

with open('output.csv', 'r') as csvfile:
    lines = csvfile.readlines()
    for line in lines:
        values = [float(item) for item in line.strip().split(',')]
        path.append(values)

for ang in path:
    # print(ang)
    time.sleep(2)
    ser.write(f"{round(ang[0])},{round(ang[1])},{round(ang[2])},{round(ang[3])},{round(ang[4])},{60}\n".encode())
    # print(f"{round(ang[0])},{round(ang[1])},{round(ang[2])},{round(ang[3])},{round(ang[4])},{60}\n".encode())

ser.close()
