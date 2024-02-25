import time
import serial
path = [ #experimental values
[0,60,120,0,90,80],
[0,60,120,0,90,80],
[0,60,120,0,90,50],
[0,90,140,0,75,48],
[90,90,140,0,75,52],
[90,60,90,0,60,52],
[90,60,120,0,120,80],
]
ser = serial.Serial('COM3', 115200)
for ang in path:
    print(ang)
    time.sleep(1.5)
    ser.write(f"{round(ang[0])},{round(ang[1])},{round(ang[2])},{round(ang[3])},{round(ang[4])},{round(ang[5])}\n".encode())
