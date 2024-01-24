import csv
import select
import sys
import serial
import math
import pygame
import numpy as np
from scipy.optimize import fsolve
import time
class arm:
    def __init__(self, a0, a1, a2, a3):
        # Initialize Pygame
        pygame.init()
        
        self.clock = pygame.time.Clock()
        # Screen dimensions
        self.width, self.height = 1000, 1000
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

        # Font setup
        self.font = pygame.font.SysFont(None, 24)

        # Initialize variables for positions of destinations and angles

        self.xy1  = pygame.math.Vector2(self.width // 2,self.height // 2)
        self.xy2  = pygame.math.Vector2(self.width // 2,self.height // 2)
        self.xy3  = pygame.math.Vector2(self.width // 2,self.height // 2)
        
        self.theta1, self.theta2, self.theta3 = 0, 0, 0
        self.source = self.xy3
        self.destination  = pygame.math.Vector2(self.width // 2,self.height // 2)
        self.str = "0,90,0,90,90,60".encode()


    def inverse_kinematics_N3(self, L1, L2, L3, x, y, initial_guess=None):
        def equations(variables):
            theta_1, theta_2, theta_3 = variables
            eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
            eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
            return [eq1, eq2, 0]
        if initial_guess is None:
            initial_guess = [5.06,-67.00,-12.00]

        result = fsolve(equations, initial_guess)

        theta_1, theta_2, theta_3 = result

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
        # theta 1 joint limitations
        if theta_1_deg > 0:
            theta_1_deg=-theta_1_deg
        # theta 2 joint limitations
        if theta_2_deg > 90 :
            theta_2_deg = 90
        if theta_2_deg <-90:
            theta_2_deg = -90
        # theta 3 joint limitations
        if theta_3_deg > 90:
            theta_3_deg = 90
        if theta_3_deg < -90:
            theta_3_deg = -90

        return theta_1_deg, theta_2_deg, theta_3_deg
    def draw(self):
            self.screen.fill(self.white) 
            pygame.draw.line(self.screen, self.black, ((-0.5*self.a0)+(self.width // 2), self.a0+(self.height // 2)), ((0.5*self.a0)+(self.width // 2), self.a0+(self.height // 2)), 5)
            pygame.draw.line(self.screen, self.black, (self.width // 2, self.a0+(self.height // 2)), (self.width // 2, self.height // 2), 5)
            pygame.draw.line(self.screen, self.black, (self.width // 2, self.height // 2), (self.xy1[0], self.xy1[1]), 5)
            pygame.draw.line(self.screen, self.black, (self.xy1[0], self.xy1[1]), (self.xy2[0], self.xy2[1]), 5)
            pygame.draw.line(self.screen, self.black, (self.xy2[0], self.xy2[1]), (self.xy3[0], self.xy3[1]), 5)
            pygame.draw.circle(self.screen, self.blue, (self.width // 2, self.height // 2), 8)
            pygame.draw.circle(self.screen, self.black, (int(self.xy1[0]), int(self.xy1[1])), 8)
            pygame.draw.circle(self.screen, self.red, (int(self.xy2[0]), int(self.xy2[1])), 8)
            pygame.draw.circle(self.screen, self.red, (int(self.xy3[0]), int(self.xy3[1])), 8)
            pygame.draw.circle(self.screen, self.red, (self.destination), 8) 
            # pygame.draw.circle(self.screen, self.black, (self.source), 8) 
            # pygame.draw.line(self.screen, self.black, (self.source[0],self.source[1]), (self.destination[0],self.destination[1]), 5)

            # Display text on screen
            text_x = self.font.render(f"x: {self.destination[0] - self.width // 2}", True, self.black)
            text_y = self.font.render(f"y: {self.destination[1] - self.height // 2}", True, self.black)
            text_theta1 = self.font.render(f"Theta1: {self.theta1:.2f} degrees", True, self.black)
            text_theta2 = self.font.render(f"Theta2: {self.theta2:.2f} degrees", True, self.black)
            text_theta3 = self.font.render(f"Theta3: {self.theta3:.2f} degrees", True, self.black)

            self.screen.blit(text_x, (10, 70))
            self.screen.blit(text_y, (10, 90))
            self.screen.blit(text_theta1, (10, 10))
            self.screen.blit(text_theta2, (10, 30))
            self.screen.blit(text_theta3, (10, 50))
            pygame.display.flip()
    def move(self):
        
        if pygame.mouse.get_pressed()[0]: 
           pointx,pointy = pygame.mouse.get_pos()
           if pointy > (self.height // 2) + self.a0 :
               pointy = (self.height // 2) + self.a0
            # Update initial guess based on whether it's the first point
           if not hasattr(self, 'first_point_selected') or self.first_point_selected:
               initial_guess = [5.06, -67.00, -12.00]
               self.first_point_selected = False
           else:
               # Use the output angles from the previous point as the initial guess
               initial_guess = [self.theta1, self.theta2, self.theta3]

        # Update destination and calculate inverse kinematics
           self.destination = pointx,pointy
           self.theta1, self.theta2, self.theta3 = self.inverse_kinematics_N3(self.a1, self.a2, self.a3, self.destination[0]-self.width // 2,self.destination[1]-self.height // 2,initial_guess=initial_guess)
        # Update initial guess for the next point
           initial_guess = [self.theta1, self.theta2, self.theta3]
        # calculate joint locations for plotting (forward kinematics)
           self.xy1[0] = self.a1 * np.cos(np.radians(self.theta1)) + self.width // 2
           self.xy1[1] = self.a1 * np.sin(np.radians(self.theta1)) + self.height // 2
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
           self.theta2=self.theta2+90
           self.theta3 = (-self.theta3 + 90) % 360
        #send angles to arduino driver
           self.str = f'0,{self.theta1},{self.theta2},0,{self.theta3},80\n'#.encode()
           print(self.str)

           ser = serial.Serial('COM3', 115200)
           time.sleep(2)
           ser.write(self.str.encode())
           ser.close()
               
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
    arm_instance = arm(100,130,120,125)
    arm_instance.run()