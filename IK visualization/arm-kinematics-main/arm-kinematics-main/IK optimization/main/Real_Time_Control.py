import csv
import select
import os
import sys
import serial
import math
import pygame
import numpy as np
from scipy.optimize import minimize
import time

class arm:
    def __init__(self, a0, a1, a2, a3):
        # Initialize Pygame
        pygame.init()
        script_name = os.path.basename(__file__)
        self.clock = pygame.time.Clock()
        # Screen dimensions
        self.width, self.height = 1000, 1000
        self.Cx,self.Cy = self.width // 2 ,self.height // 2
        self.Cy = 0.7*self.Cy
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(script_name)

        # Colors
        self.colors = {
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            'red': (255, 0, 0),
            'blue': (0, 0, 255),
            'light_sky_blue': pygame.Color('lightskyblue3'),
            'dodger_blue': pygame.Color('dodgerblue2'),
        }

        # Link lengths
        self.a0,self.a1,self.a2,self.a3 = a0,a1,a2,a3
        self.link_lengths=[a1,a2,a3]
        self.num_joints = len(self.link_lengths)
        # Font setup
        self.font = pygame.font.SysFont(None, 24)


        # Initialize variables for positions of destinations and angles
        #plotting vectors
        self.xy0 = pygame.math.Vector2(self.Cx,self.Cy)          # the joint points
        self.xy1 = pygame.math.Vector2(self.Cx,self.Cy)          # the joint points
        self.xy2 = pygame.math.Vector2(self.Cx,self.Cy)          # the joint points
        self.xy3 = pygame.math.Vector2(self.Cx,self.Cy)          # the joint points
        self.destination  = pygame.math.Vector2(self.Cx,self.Cy) # the selected point
        self.R = 0                                               #top view ploting radius

        #joint variables
        self.theta0, self.theta1, self.theta2, self.wrist, self.theta3 = 0, -90, 90, 0, 0
        self.theta_gripper = 75
        self.str = "90,90,0,90,90,60".encode() # initialization doesn't affect homing , homing taken from driver
        self.angles = [90,90,0,90]
        
        #point variables
        self.endeff = self.xy3
        self.target_point =[0,0,0] 

        #user point variables
        self.end_reach  = 160    #zf
        self.end_yaw    =  0    #final yaw angle
        self.end_height =  30    #xf

        #control variables
        self.step = 30  # Initial self.step value
        self.previous_x = self.end_reach
        self.previous_yaw = self.end_yaw
        self.previous_z = self.end_height
        self.previous_grip = self.theta_gripper

    def inverse_kinematics_N4(self, x_t, z_t, boundaries=None):
        # r_t = self.get_hypo(x_t,y_t)
        target_position = np.array([x_t, z_t])
        # Update initial guess based on whether it's the first point
        if not hasattr(self, 'first_point_selected') or self.first_point_selected:
            # initial_guess = [-100,60,-80]
            # initial_guess = [90,0,90]
            initial_guess=[0,0,0]
            boundaries = [(-np.radians(180), np.radians(0)), (0, np.radians(145)), (-np.radians(90), np.radians(90))]
            # print("first point")

        else:
            # Use the output angles from the previous point as the initial guess
            initial_guess = [self.theta1, self.theta2, self.theta3]
            # initial_guess=[0,0,0]
            boundaries = [(-np.radians(180), 0), (0, np.radians(145)), (np.radians(0), np.radians(90))]
            # boundaries = [(0,self.theta1), (0, self.theta2), (np.radians(0), self.theta3)]
            # print("next point")
            self.first_point_selected = False
        def forward_kinematics(angles):
            x = 0
            z = 0
            for i in range(self.num_joints):
                x += self.link_lengths[i] * np.cos(np.sum(angles[:i+1]))
                z += self.link_lengths[i] * np.sin(np.sum(angles[:i+1]))
            return np.array([x, z]) 
        
        def error_function(angles, target_position):
            end_effector_position = forward_kinematics(angles)
            return np.linalg.norm(end_effector_position - target_position)

        # if(True):
        #         theta_1 = -90
        #         theta_2 = 0
        #         theta_3 = 0
        # else:
        result = minimize(
            fun=lambda angles: error_function(angles, target_position),
            x0=initial_guess,
            method='SLSQP',
            bounds=boundaries,
            options={'disp': False},
            tol= 1.49e-10,
        )
        # theta_0 = math.atan2(y_t,x_t) 
        # print(result.success)
        # print(result.message)
        theta_1, theta_2, theta_3 = result.x

        # converting from radian to degree
        # theta_0_deg = np.degrees(theta_0 % (2 * np.pi))
        theta_1_deg = np.degrees(theta_1 % (2 * np.pi))
        theta_2_deg = np.degrees(theta_2 % (2 * np.pi))
        theta_3_deg = np.degrees(theta_3 % (2 * np.pi))
        # keep angles in range of 180
        # if abs(theta_0_deg) > 180:
        #     theta_0_deg = int(theta_0_deg - math.copysign(360, theta_0_deg))
        if abs(theta_1_deg) > 180:
            theta_1_deg = int(theta_1_deg - math.copysign(360, theta_1_deg))
        if abs(theta_2_deg) > 180:
            theta_2_deg = int(theta_2_deg - math.copysign(360, theta_2_deg))
        if abs(theta_3_deg) > 180:
            theta_3_deg = int(theta_3_deg - math.copysign(360, theta_3_deg))

        return theta_1_deg, theta_2_deg, theta_3_deg

    def adjusted_angles(self):
        for attr in ['theta0','theta1', 'theta2', 'theta3']:
            setattr(self, attr, round(getattr(self, attr), 0))
        self.angles[0] = self.theta0
        self.angles[1] = abs(self.theta1)
        self.angles[2] = self.theta2
        self.angles[3] = (-self.theta3 + 90) % 360
    
    def grip(self, state):
        self.theta_gripper = 45 if state else 80

    def get_hypo(self, a, b):
        return(abs(math.sqrt((a)**2+(b)**2)))

    def forward_kinematics(self):
        self.R = self.end_reach
        self.xy0[0] = self.R * math.cos(np.radians(self.theta0)) + self.Cx
        self.xy0[1] = self.R * math.sin(np.radians(self.theta0)) + self.Cy

        self.xy1[0] = self.a1 * np.cos(np.radians(self.theta1)) + self.Cx
        self.xy1[1] = self.a1 * np.sin(np.radians(self.theta1)) + self.Cy
        self.xy2[0] = self.xy1[0] + self.a2 * np.cos(np.radians(self.theta1) + np.radians(self.theta2))
        self.xy2[1] = self.xy1[1] + self.a2 * np.sin(np.radians(self.theta1) + np.radians(self.theta2))
        self.xy3[0] = self.xy2[0] + self.a3 * np.cos(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))
        self.xy3[1] = self.xy2[1] + self.a3 * np.sin(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))
        # print(f'{self.xy1} ,{self.xy2} ,{self.xy3}')
    
    def draw(self):
        self.screen.fill(self.colors['white'])
        pygame.draw.line(self.screen, self.colors['black'], ((-0.5*self.a0)+(self.Cx), self.a0+(self.Cy)), ((0.5*self.a0)+(self.Cx), self.a0+(self.Cy)), 5)
        pygame.draw.line(self.screen, self.colors['black'], (self.Cx, self.a0+(self.Cy)), (self.Cx, self.Cy), 5)
        pygame.draw.line(self.screen, self.colors['black'], (self.Cx, self.Cy), (self.xy1[0], self.xy1[1]), 5)
        pygame.draw.line(self.screen, self.colors['black'], (self.xy1[0], self.xy1[1]), (self.xy2[0], self.xy2[1]), 5)
        pygame.draw.line(self.screen, self.colors['black'], (self.xy2[0], self.xy2[1]), (self.xy3[0], self.xy3[1]), 5)

        pygame.draw.circle(self.screen, self.colors['blue'], (self.Cx, self.Cy), 8)
        pygame.draw.circle(self.screen, self.colors['black'], (int(self.xy1[0]), int(self.xy1[1])), 8)
        pygame.draw.circle(self.screen, self.colors['red'], (int(self.xy2[0]), int(self.xy2[1])), 8)
        pygame.draw.circle(self.screen, self.colors['red'], (int(self.xy3[0]), int(self.xy3[1])), 8)
        pygame.draw.circle(self.screen, self.colors['red'], (self.destination), 8) 
        
        Rmax = 1 * (self.a1 + self.a2 + self.a3)
        pygame.draw.line(self.screen, self.colors['black'], (self.Cx, (self.Cy + Rmax)), (self.xy0[0], self.xy0[1] + Rmax), 8)
        pygame.draw.circle(self.screen, self.colors['blue'], (self.Cx, int(self.Cy + Rmax)), 8)
        pygame.draw.circle(self.screen, self.colors['red'], (int(self.xy0[0]), int(self.xy0[1] + Rmax)), 8)

        # Display text on screen
        text_coord = self.font.render(f"input end reach , end y and end height seperated by commas", True, self.colors['black'])
        text_x = self.font.render(f"x: {self.end_reach}", True, self.colors['black'])
        text_y = self.font.render(f"yaw: {self.end_yaw}°",True, self.colors['black'])
        text_z = self.font.render(f"z: {self.end_height}", True, self.colors['black'])
        text_theta0 = self.font.render(f"Theta0: {self.theta0:.2f} degrees", True, self.colors['black'])
        text_theta1 = self.font.render(f"Theta1: {self.theta1:.2f} degrees", True, self.colors['black'])
        text_theta2 = self.font.render(f"Theta2: {self.theta2:.2f} degrees", True, self.colors['black'])
        text_theta3 = self.font.render(f"Theta3: {self.theta3:.2f} degrees", True, self.colors['black'])


        self.screen.blit(text_coord, (10, 170))
        self.screen.blit(text_theta0, (10, 10))
        self.screen.blit(text_theta1, (10, 30))
        self.screen.blit(text_theta2, (10, 50))
        self.screen.blit(text_theta3, (10, 70))
        self.screen.blit(text_x, (10, 90))
        self.screen.blit(text_y, (10, 110))
        self.screen.blit(text_z, (10, 130))
        
        pygame.display.flip()

    def control(self):
        #function intended output
        # x = self.end_reach  # x coordinate
        # y = self.end_yaw  # yaw angle
        # z = self.end_height   # z coordinate
        # g = self.theta_gripper  # gripper angle
        # print(f"x: {self.end_reach}, y: {self.end_yaw}, z: {self.end_height}")
        # function local variables 
        delay = 500  # Delay in milliseconds

        if (self.end_reach != self.previous_x or self.end_yaw != self.previous_yaw or self.end_height != self.previous_z or self.theta_gripper != self.previous_grip):
            print(f"x: {self.end_reach}, y: {self.end_yaw}, z: {self.end_height}, gripper: {self.theta_gripper}, Step: {self.step}")
            # pass
        self.previous_x = self.end_reach
        self.previous_yaw = self.end_yaw
        self.previous_z = self.end_height
        self.previous_grip = self.theta_gripper
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            time.sleep(delay / 1000.0)
            self.end_yaw += self.step
            if self.end_yaw > 180:
                self.end_yaw = 180
        elif keys[pygame.K_s]:
            time.sleep(delay / 1000.0)
            self.end_yaw -= self.step
            if self.end_yaw < 0:
                self.end_yaw = 0
        
        elif keys[pygame.K_RIGHT]:
            time.sleep(delay / 1000.0)
            self.end_reach += self.step
            # self.end_height -= self.step/2
            if self.end_reach > 375:
                self.end_reach = 375
        elif keys[pygame.K_LEFT]:
            time.sleep(delay / 1000.0)
            self.end_reach -= self.step
            # self.end_height += self.step/2
            if self.end_reach < 0:
                self.end_reach = 0

        elif keys[pygame.K_UP]:
            time.sleep(delay / 1000.0)
            self.end_height += self.step
            # self.end_reach -= self.step/2
            if self.end_height > 475:
                self.end_height = 475
        elif keys[pygame.K_DOWN]:
            time.sleep(delay / 1000.0)
            self.end_height -= self.step
            # self.end_reach += self.step/2
            if self.end_height < 10:
                self.end_height = 10

        elif keys[pygame.K_a]:
            time.sleep(delay / 1000.0)
            self.theta_gripper += 10
            if self.theta_gripper > 80:
                self.theta_gripper = 80
        elif keys[pygame.K_d]:
            time.sleep(delay / 1000.0)
            self.theta_gripper -= 10
            if self.theta_gripper < 45:
                self.theta_gripper = 45
        elif keys[pygame.K_i]:
            time.sleep(delay / 1000.0)
            self.step += 10
            if self.step > 90:
                self.step = 90
        elif keys[pygame.K_k]:
            time.sleep(delay / 1000.0)
            self.step -= 10
            if self.step < 10:
                self.step = 10

    def move(self):
        # Handle input events
        ser = serial.Serial('COM3', 115200,dsrdtr=True)
        # self.end_reach,self.end_yaw,self.end_height = 160,30,30              
        boundaries = [(-np.radians(180), 0), (0, np.radians(145)), (-np.radians(90), np.radians(90))]
        pos = [self.end_reach, self.end_yaw, self.end_height] # here for easy editing
        if self.end_reach == 0:
            self.destination = [(pos[0] + self.Cx), (self.a0 + self.Cy - pos[2])]
        self.target_point = [pos[0], pos[1], self.a0 - pos[2]]
        # print(pos)
        self.theta0 = self.target_point[1]     
        self.theta1, self.theta2, self.theta3 = self.inverse_kinematics_N4(self.target_point[0], self.target_point[2], boundaries=boundaries)
        # calculate joint locations for plotting (forward kinematics)
        self.forward_kinematics()
        self.draw()
        # edit angles for physical arm configuration and joint limitations
        self.adjusted_angles()
        # send angles to arduino driver
        # self.grip(True)
        self.str = f'{self.angles[0]},{self.angles[1]},{self.angles[2]},{self.wrist},{self.angles[3]},{self.theta_gripper}\n'  # .encode()
        
        # print(self.str)

        # self.angles.append(self.str)
        # self.writefile('output.csv',self.angles)
        time.sleep(1.5)
        ser.write(self.str.encode())

        # self.grip(False)
        # self.str = f'{self.theta0},{self.theta1},{self.theta2},{self.wrist},{self.theta3},{self.theta_gripper}\n'  # .encode()
        # time.sleep(2)
        # ser.write(self.str.encode())
        

        # ser.close()
        self.draw()

    def tutorial(self):
        print("Press the LEFT arrow key to increment x, RIGHT arrow key to decrement x.")
        print("Press the UP arrow key to increment y, DOWN arrow key to decrement y.")
        print("Press 'w' to increment z, 's' to decrement z.")
        print("Press 'a' to increment gripper, 'd' to decrement gripper.")
        print("Press 'i' to increase self.step size, 'k' to decrease self.step size.")
        # print("Press ESC to exit.")

    def run(self):
        self.forward_kinematics()
        self.draw()
        # arduino_home = serial.Serial('COM3', 115200)
        # arduino_home.write(self.str)
        # arduino_home.close()
        self.tutorial()
        running = True
        while running:
            self.control()
            self.move()
            pygame.display.flip()
            self.clock.tick(60)  # Adjust 60 to your desired frame rate

        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    arm_instance = arm(100,130,120,125)
    arm_instance.run()