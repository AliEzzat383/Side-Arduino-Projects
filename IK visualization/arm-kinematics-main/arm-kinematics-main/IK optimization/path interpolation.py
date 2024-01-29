import csv
import select
import sys
import serial
import math
import pygame
import numpy as np
from scipy.optimize import minimize
import time
#just as a reminder of experimental initial guesses
        # initial_guess = [5.06, -67.00, -12.00]
        # initial_guess = [-63.00,122.45,-12.00]
        # initial_guess = [0,-90,30]
        # initial_guess = [-90,120,0]
        # initial_guess = [30,-60,50]
class arm:
    def __init__(self, a0, a1, a2, a3):
        # Initialize Pygame
        pygame.init()
        
        self.clock = pygame.time.Clock()
        # Screen dimensions
        self.width, self.height = 1000, 1000
        self.Cx,self.Cy = self.width // 2 ,self.height // 2
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Three-Link Robotic Arm System Animation')

        # Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)

        # Link lengths
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.link_lengths=[a1,a2,a3]
        self.num_joints = len(self.link_lengths)


        # Font setup
        self.font = pygame.font.SysFont(None, 24)

        # Initialize variables for positions of destinations and angles

        self.xy1  = pygame.math.Vector2(self.Cx,self.Cy)
        self.xy2  = pygame.math.Vector2(self.Cx,self.Cy)
        self.xy3  = pygame.math.Vector2(self.Cx,self.Cy)
        
        self.theta1, self.theta2, self.theta3 = 0, 0, 0
        self.theta_gripper = 55
        self.source = self.xy3
        self.destination  = pygame.math.Vector2(self.Cx,self.Cy)
        # self.target_point =[] 
        self.str = "0,90,0,90,90,60".encode()
        self.angles = []
    def inverse_kinematics_N4(self, x_t, y_t, initial_guess=None,boundaries=None):
        target_position = np.array([x_t, y_t])
        # Update initial guess based on whether it's the first point
        if not hasattr(self, 'first_point_selected') or self.first_point_selected:
            initial_guess = [30,-60,50]
            boundaries = [(-np.radians(180), 0), (0, np.radians(145)), (-np.radians(90), np.radians(90))]
            self.first_point_selected = False
            print("first point")
        else:
            # Use the output angles from the previous point as the initial guess
            initial_guess = [self.theta1, self.theta2, self.theta3]
            boundaries = [(-np.radians(180), 0), (0, np.radians(145)), (np.radians(45), np.radians(60))]
            print("next point")
            # self.first_point_selected = False
        def forward_kinematics(angles):
            x = 0
            y = 0
            for i in range(self.num_joints):
                x += self.link_lengths[i] * np.cos(np.sum(angles[:i+1]))
                y += self.link_lengths[i] * np.sin(np.sum(angles[:i+1]))
            return np.array([x, y]) 
        
        def error_function(angles, target_position):
            end_effector_position = forward_kinematics(angles)
            return np.linalg.norm(end_effector_position - target_position)

        result = minimize(
            fun=lambda angles: error_function(angles, target_position),
            x0=initial_guess,
            method='SLSQP',
            bounds=boundaries,
            options={'disp': False},
            tol= 1.49e-7,
        )
        print(result.success)
        print(result.message)
        print(target_position)
        theta_1, theta_2, theta_3 = result.x

        # converting from radian to degree
        theta_1_deg = np.degrees(theta_1 % (2 * np.pi))
        theta_2_deg = np.degrees(theta_2 % (2 * np.pi))
        theta_3_deg = np.degrees(theta_3 % (2 * np.pi))
        # keep angles in range of 180
        if abs(theta_1_deg) > 180:
            theta_1_deg = int(theta_1_deg - math.copysign(360, theta_1_deg))
        if abs(theta_2_deg) > 180:
            theta_2_deg = int(theta_2_deg - math.copysign(360, theta_2_deg))
        if abs(theta_3_deg) > 180:
            theta_3_deg = int(theta_3_deg - math.copysign(360, theta_3_deg))
        initial_guess=[theta_1_deg,theta_2_deg,theta_3_deg]
        return theta_1_deg, theta_2_deg, theta_3_deg 

    def draw(self):
            self.screen.fill(self.white) 
            pygame.draw.line(self.screen, self.black, ((-0.5*self.a0)+(self.Cx), self.a0+(self.Cy)), ((0.5*self.a0)+(self.Cx), self.a0+(self.Cy)), 5)
            pygame.draw.line(self.screen, self.black, (self.Cx, self.a0+(self.Cy)), (self.Cx, self.Cy), 5)
            pygame.draw.line(self.screen, self.black, (self.Cx, self.Cy), (self.xy1[0], self.xy1[1]), 5)
            pygame.draw.line(self.screen, self.black, (self.xy1[0], self.xy1[1]), (self.xy2[0], self.xy2[1]), 5)
            pygame.draw.line(self.screen, self.black, (self.xy2[0], self.xy2[1]), (self.xy3[0], self.xy3[1]), 5)
            pygame.draw.circle(self.screen, self.blue, (self.Cx, self.Cy), 8)
            pygame.draw.circle(self.screen, self.black, (int(self.xy1[0]), int(self.xy1[1])), 8)
            pygame.draw.circle(self.screen, self.red, (int(self.xy2[0]), int(self.xy2[1])), 8)
            pygame.draw.circle(self.screen, self.red, (int(self.xy3[0]), int(self.xy3[1])), 8)
            pygame.draw.circle(self.screen, self.red, (self.destination), 8) 
            # pygame.draw.circle(self.screen, self.black, (self.source), 8) 
            # pygame.draw.line(self.screen, self.black, (self.source[0],self.source[1]), (self.destination[0],self.destination[1]), 5)

            # Display text on screen
            text_x = self.font.render(f"x: {self.destination[0] - self.Cx}", True, self.black)
            text_y = self.font.render(f"y: {-(self.destination[1] - (self.a0+self.Cy))}", True, self.black)
            text_theta1 = self.font.render(f"Theta1: {self.theta1:.2f} degrees", True, self.black)
            text_theta2 = self.font.render(f"Theta2: {self.theta2:.2f} degrees", True, self.black)
            text_theta3 = self.font.render(f"Theta3: {self.theta3:.2f} degrees", True, self.black)

            self.screen.blit(text_x, (10, 70))
            self.screen.blit(text_y, (10, 90))
            self.screen.blit(text_theta1, (10, 10))
            self.screen.blit(text_theta2, (10, 30))
            self.screen.blit(text_theta3, (10, 50))
            pygame.display.flip()
    
    def grip(self,state):
        if state:
            self.theta_gripper = 45
        else:
            self.theta_gripper = 80

    def move(self):
            if pygame.mouse.get_pressed()[0]:
                pointx, pointy = pygame.mouse.get_pos()
                if pointy > (self.Cy) + self.a0:
                    pointy = (self.Cy) + self.a0
                # Update initial guess based on whether it's the first point
                # # Update destination and calculate inverse kinematics
                #    self.destination = pointx,pointy
                #    self.target_point =[self.destination[0]-self.Cx,self.destination[1]-self.Cy]
                # to test specific points
                # ser = serial.Serial('COM3', 115200)
                for i in range(0,102,5):
                    pos = [159 + i, 81]
                    self.destination[0] = pos[0] + self.Cx
                    self.destination[1] = self.a0 + self.Cy - pos[1]
                    self.target_point = [self.destination[0] - self.Cx, self.destination[1] - self.Cy]
                    # print(self.target_point)  
                    self.theta1, self.theta2, self.theta3 = self.inverse_kinematics_N4(self.target_point[0],self.target_point[1])
                    # Update initial guess for the next point
                    # initial_guess = [self.theta1, self.theta2, self.theta3]
                    # calculate joint locations for plotting (forward kinematics)
                    self.xy1[0] = self.a1 * np.cos(np.radians(self.theta1)) + self.Cx
                    self.xy1[1] = self.a1 * np.sin(np.radians(self.theta1)) + self.Cy
                    self.xy2[0] = self.xy1[0] + self.a2 * np.cos(np.radians(self.theta1) + np.radians(self.theta2))
                    self.xy2[1] = self.xy1[1] + self.a2 * np.sin(np.radians(self.theta1) + np.radians(self.theta2))
                    self.xy3[0] = self.xy2[0] + self.a3 * np.cos(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))
                    self.xy3[1] = self.xy2[1] + self.a3 * np.sin(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))   
                    self.draw()
                    #edit angles for physical arm configuration and joint limitations
                    self.theta1=abs(round(self.theta1,0))
                    self.theta2= round(self.theta2,0)
                    self.theta3=round(self.theta3,0)   
                    self.theta1=self.theta1
                    self.theta2=self.theta2
                    self.theta3 = (-self.theta3 + 90) % 360
                    if i> 20 :
                        self.grip(True)
                    #send angles to arduino driver
                    self.str = f'0,{self.theta1},{self.theta2},0,{self.theta3},{self.theta_gripper}\n'#.encode()
                    print(self.str)
                    # ser = serial.Serial('COM3', 115200)
                    time.sleep(0.5)
                    # ser.write(self.str.encode())
                    self.angles.append(self.str)
                    self.writefile('output.csv')
                    # ser.close()
                
                for j in range(0,91,15):
                    pos = [159 + i, 81-j]
                    self.destination[0] = pos[0] + self.Cx
                    self.destination[1] = self.a0 + self.Cy - pos[1]
                    self.target_point = [self.destination[0] - self.Cx, self.destination[1] - self.Cy]
                    # print(self.target_point)  
                    self.theta1, self.theta2, self.theta3 = self.inverse_kinematics_N4(self.target_point[0],self.target_point[1])
                    # Update initial guess for the next point
                    # initial_guess = [self.theta1, self.theta2, self.theta3]
                    # calculate joint locations for plotting (forward kinematics)
                    self.xy1[0] = self.a1 * np.cos(np.radians(self.theta1)) + self.Cx
                    self.xy1[1] = self.a1 * np.sin(np.radians(self.theta1)) + self.Cy
                    self.xy2[0] = self.xy1[0] + self.a2 * np.cos(np.radians(self.theta1) + np.radians(self.theta2))
                    self.xy2[1] = self.xy1[1] + self.a2 * np.sin(np.radians(self.theta1) + np.radians(self.theta2))
                    self.xy3[0] = self.xy2[0] + self.a3 * np.cos(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))
                    self.xy3[1] = self.xy2[1] + self.a3 * np.sin(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))   
                    self.draw()
                    #edit angles for physical arm configuration and joint limitations
                    self.theta1=abs(round(self.theta1,0))
                    self.theta2= round(self.theta2,0)
                    self.theta3=round(self.theta3,0)   
                    self.theta1=self.theta1
                    self.theta2=self.theta2
                    self.theta3 = (-self.theta3 + 90) % 360
                    if j>=90:
                        self.grip(False)#release
                    #send angles to arduino driver
                    self.str = f'0,{self.theta1},{self.theta2},0,{self.theta3},{self.theta_gripper}\n'#.encode()
                    print(self.str)
                    time.sleep(0.5)
                    # ser = serial.Serial('COM3', 115200)
                    # time.sleep(2)
                    # ser.write(self.str.encode())
                    self.angles.append(self.str)
                    self.writefile('output.csv')
                    # ser.close()

                
            
    def writefile(self,filename):
    
        with open(filename,"w",newline='') as file:
            for row in self.angles:
                file.write(row)
        file.close()         
    
    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.move()
            pygame.display.flip()  # Move display.flip() outside the move() method
            self.clock.tick(60)  # Adjust 60 to your desired frame rate

        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    arm_instance = arm(90,130,120,125)
    arm_instance.run()