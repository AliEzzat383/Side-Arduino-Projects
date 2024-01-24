def inverse_kinematics_N3(self,L1, L2, L3, x, y):
        def equations(variables):
                    theta_1, theta_2, theta_3 = variables
                    eq1 = x - L1 * np.cos(theta_1) - L2 * np.cos(theta_1 + theta_2) - L3 * np.cos(theta_1 + theta_2 + theta_3)
                    eq2 = y - L1 * np.sin(theta_1) - L2 * np.sin(theta_1 + theta_2) - L3 * np.sin(theta_1 + theta_2 + theta_3)
                    return eq1**2 + eq2**2

        # Set constraints for theta values to be in the range 0 to 180 degrees
        constraints = (
            {'type': 'ineq', 'fun': lambda x: x[0]},  # theta_1 >= 0
            {'type': 'ineq', 'fun': lambda x: 180 - x[0]},  # theta_1 <= 180
            {'type': 'ineq', 'fun': lambda x: x[1]},  # theta_2 >= 0
            {'type': 'ineq', 'fun': lambda x: 180 - x[1]},  # theta_2 <= 180
            {'type': 'ineq', 'fun': lambda x: x[2]},  # theta_3 >= 0
            {'type': 'ineq', 'fun': lambda x: 180 - x[2]}  # theta_3 <= 180
        )

        # Initial guess for theta_1, theta_2, and theta_3
        initial_guess = [45, 45, 45]

        # Use minimize with constraints and bounds
        result = minimize(equations, initial_guess, constraints=constraints, bounds=[(0, 180)] * 3)

        # Extract optimal theta values
        theta_1, theta_2, theta_3 = result.x

        #mathematical values calculated for plot visualization
        #to map radian angles to degrees in range 0 to 360
        theta_1_deg = np.degrees(theta_1 % (2 * np.pi))
        theta_2_deg = np.degrees(theta_2 % (2 * np.pi))
        theta_3_deg = np.degrees(theta_3 % (2 * np.pi))
        #to limit links from self collision
        if abs(theta_1_deg) > 180:
            theta_1_deg = int(theta_1_deg - math.copysign(360 ,theta_1_deg))
        if abs(theta_1_deg) > 180:
            theta_2_deg = int(theta_2_deg - math.copysign(360 ,theta_2_deg))
        if abs(theta_1_deg) > 180:
            theta_3_deg = int(theta_3_deg - math.copysign(360 ,theta_3_deg))
        # joint limitations    
        if theta_1_deg > 0:                                                 # to avoid lower elbow config under ground level
            theta_1_deg = theta_1_deg - 180
            # theta_2_deg = theta_2_deg-math.copysign(360,theta_1_deg)-90
            # theta_3_deg = theta_3_deg-math.copysign(360,theta_2_deg)

        # if abs(self.xy3[0]-self.destination[0])>100 or abs(self.xy3[1]!=self.destination[1])>100:
        #     print(f'theta_1: {round(theta_1_deg,0)}     theta_2: {round(theta_2_deg,0)}     theta_3: {round(theta_3_deg,0)} \n')

        return theta_1_deg, theta_2_deg, theta_3_deg