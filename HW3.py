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
        intergral_x = 0
        intergral_y = 0
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                '''if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
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
                
                

                '''# Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise
                if direction == 0:
                    left_speed = 7
                    right_speed = 7
                else:
                    left_speed = direction * 4
                    right_speed = direction * -4

                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)

                # Send message to team robots
                self.send_data_to_team(self.player_id)'''
                t1 = time.time()
                
                x_desired = -0.09
                y_desired = -0.050

                kp = 10
                ki = 5
                kd = 5

                #control x y     
                error_x = robot_pos[1] - x_desired
                error_y = robot_pos[0] - y_desired

                #P & PI
                proportional_x = kp * error_x
                proportional_y = kp * error_y

                intergral_x += ki * error_x * dt
                intergral_x = clamp(intergral_x, -10, 10)
                intergral_y += ki * error_y * dt
                intergral_y = clamp(intergral_y, -10, 10)

                #P & PD
                derivative_x = (kd * error_x) / dt
                derivative_x = clamp(derivative_x, -10, 10)

                derivative_y = (kd * error_y) / dt
                derivative_y = clamp(derivative_y, -10, 10)

                u_x = proportional_x + intergral_x + derivative_x
                u_y = proportional_y + intergral_y + derivative_y

                if (abs(error_x) > 0.01):
                    self.left_motor.setVelocity(u_x)
                    self.right_motor.setVelocity(u_x)
                else:
                    if error_y < 0:
                        self.left_motor.setVelocity(1)
                        self.right_motor.setVelocity(-1) 

                        if(abs(math.degrees(heading) - 90) < 1): 
                            if(abs(error_y) > 0.01):
                                self.left_motor.setVelocity(u_y)
                                self.right_motor.setVelocity(u_y) 
                            else: 
                                self.left_motor.setVelocity(0)
                                self.right_motor.setVelocity(0) 
                    else:
                        self.left_motor.setVelocity(-1)
                        self.right_motor.setVelocity(1) 

                        if(abs(math.degrees(heading) - 270) < 1): 
                            if(abs(error_y) > 0.01):
                                self.left_motor.setVelocity(u_y)
                                self.right_motor.setVelocity(u_y) 
                            else: 
                                self.left_motor.setVelocity(0)
                                self.right_motor.setVelocity(0) 
                
                t2 = time.time()
                dt = t2 - t1 
                time.sleep(dt-0.000003)