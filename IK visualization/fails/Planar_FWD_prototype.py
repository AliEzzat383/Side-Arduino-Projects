import math

a1 = 1
a2 = 1
x = 2
y = 0
R = math.sqrt(x**2 + y**2)
N = (a1**2 + a2**2 - R**2) / (2 * a1 * a2)
theta2 = 180 - math.atan2(abs(math.sqrt(1 - N**2)) , N)
theta1 = math.atan2(y,x) - math.atan2((a1+a2*math.cos(theta2)) , (a2 * math.sin(theta2)))
print(theta1 , theta2)

theta1 = 0
theta2 = 0

x1 = a1 * math.cos(theta1)
y1 = a1 * math.sin(theta1)
x2 = x1 + a2 * math.cos(theta1 + theta2)
y2 = y1 + a2 * math.sin(theta1 + theta2)

print(x2, y2)