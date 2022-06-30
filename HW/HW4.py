# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import time

def clamp(x, min, max):
    if x < min:
        x = min
    if x > max:
        x = max

    return x

class MyRobot1(RCJSoccerRobot):
    def run(self):
        dt = 10e6
        intergral_position = 0
        intergral_theta = 0

        ball_data = None

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                '''else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue'''

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
    
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                #print("heading",math.degrees(heading))
                
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                
                t1 = time.time()

                kp = 10
                kd = 0
                ki = 0.05

                angle_error = math.atan2(ball_data["direction"][0], ball_data["direction"][1]) - (math.pi/2)
                postion_error = 1/ball_data['strength'] * 10

                proportional_theta = kp * angle_error
                proportional_position = kp * postion_error

                intergral_theta += ki * angle_error * dt
                intergral_theta = clamp(intergral_theta, -10, 10)

                intergral_position += ki * postion_error * dt
                intergral_position = clamp(intergral_position, -10, 10)

                derivative_theta = (kd * angle_error) / dt
                derivative_theta = clamp(derivative_theta, -10, 10)

                derivative_position = (kd * postion_error) / dt
                derivative_position = clamp(derivative_position, -10, 10)

                u_theta = proportional_theta + intergral_theta + derivative_position
                u_postion = proportional_position + intergral_position + derivative_position
                print(u_theta, u_postion)

                self.left_motor.setVelocity(clamp(u_theta * -1 + u_postion, -10, 10))
                self.right_motor.setVelocity(clamp(u_theta + u_postion, -10, 10))

                t2 = time.time()
                dt = t2 - t1 
                time.sleep(dt-0.000003)

