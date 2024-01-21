import time
import serial
path = [ #values obtained from IK
[0,32,135,0,58,80],
[0,32,144,0,58,80],
[0,32,144,0,58,50],
[0,73,146,0,75,48],
[90,73,146,0,75,52],
[90,40,144,0,58,52],
[90,40,135,0,45,80],
]
ser = serial.Serial('COM3', 115200)
for ang in path:
    print(ang)
    time.sleep(1.5)
    ser.write(f"{round(ang[0])},{round(ang[1])},{round(ang[2])},{round(ang[3])},{round(ang[4])},{round(ang[5])}\n".encode())
