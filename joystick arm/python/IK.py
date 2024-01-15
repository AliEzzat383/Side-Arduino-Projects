import sys
import math
import pygame
import numpy as np
from scipy.optimize import fsolve
import time
class arm:
    def __init__(self, a1, a2, a3):
        # Initialize Pygame
        pygame.init()
        
        self.clock = pygame.time.Clock()
        # Screen dimensions
        self.width, self.height = 800, 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Three-Link System Animation')

        # Colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)

        # Link lengths
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
        self.distance = 0  
        self.directions = pygame.math.Vector2(0,0)
        self.points = []

    def inverse_kinematics_N3(self,L1, L2, L3, x, y):
        def equations(variables):
            theta_1, theta_2, theta_3 = variables
            eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
            eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
            return [eq1, eq2, 0]  # Adding a placeholder equation to match the number of variables

        initial_guess = [0.0, 0.0, 0.0]  # Initial guess for theta_1, theta_2, and theta_3
        theta_1, theta_2, theta_3 = fsolve(equations, initial_guess)

        theta_1_deg = np.degrees(theta_1 % (2 * np.pi))
        theta_2_deg = np.degrees(theta_2 % (2 * np.pi))
        theta_3_deg = np.degrees(theta_3 % (2 * np.pi))
        if abs(theta_1_deg) > 180:
            theta_1_deg = int(theta_1_deg - math.copysign(360 ,theta_1_deg))
        if abs(theta_2_deg) > 180:
            theta_2_deg = int(theta_2_deg - math.copysign(360 ,theta_2_deg))
        if abs(theta_1_deg) > 180:
            theta_3_deg = int(theta_3_deg - math.copysign(360 ,theta_3_deg))
        
        if theta_1_deg > 0:
            theta_1_deg = theta_1_deg - 180
            theta_2_deg = -theta_2_deg
            theta_3_deg = -theta_3_deg
            

        return theta_1_deg, theta_2_deg, theta_3_deg
    def draw(self):
            self.screen.fill(self.white) 
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
        self.angles = []
        print(0,",",self.theta1,",",self.theta2,",",self.theta3,",",0,",",60)
        self.angles.append(0,",",self.theta1,",",self.theta2,",",self.theta3,",",0,",",60)
        if pygame.mouse.get_pressed()[0]: 
           pointx,pointy = pygame.mouse.get_pos()
        #    if pointx < self.width // 2 :
        #        pointx = self.width // 2
           if pointy > self.height // 2 :
               pointy = self.height // 2
           self.destination = pointx,pointy
        #    self.calculate_distance()
        point = self.destination
        self.theta1, self.theta2, self.theta3 = self.inverse_kinematics_N3(self.a1, self.a2, self.a3, point[0]-self.width // 2,point[1]-self.height // 2)
        self.xy1[0] = self.a1 * np.cos(np.radians(self.theta1)) + self.width // 2
        self.xy1[1] = self.a1 * np.sin(np.radians(self.theta1)) + self.height // 2
        self.xy2[0] = self.xy1[0] + self.a2 * np.cos(np.radians(self.theta1) + np.radians(self.theta2))
        self.xy2[1] = self.xy1[1] + self.a2 * np.sin(np.radians(self.theta1) + np.radians(self.theta2))
        self.xy3[0] = self.xy2[0] + self.a3 * np.cos(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))
        self.xy3[1] = self.xy2[1] + self.a3 * np.sin(np.radians(self.theta1) + np.radians(self.theta2) + np.radians(self.theta3))
        self.draw()
        if abs(self.theta3) > 180:
            self.theta3 = self.theta3 - math.copysign(180,self.theta3)
        print(0,",",self.theta1,",",self.theta2,",",self.theta3,",",0,",",60)
        # time.sleep(0.01)
    def calculate_distance(self):
        distance = int(math.hypot(self.destination[0] - self.source[0], self.destination[1] - self.source[1]))
        if distance > 0:
            # Get number of points 
            num_points = (distance)

            # Calculate spacing
            spacing = distance / (num_points)

            self.points = [self.source]
            for i in range(1, num_points):
                pt_x = self.source[0] + i * spacing * (self.destination[0] - self.source[0]) / distance
                pt_y = self.source[1] + i * spacing * (self.destination[1] - self.source[1]) / distance 
                self.points.append((pt_x, pt_y))
            self.points.append(self.destination)
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
    arm_instance = arm(100, 70, 50)
    arm_instance.run()
