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
    intergral = 0
    
    def run(self):
        dt = 10e-6
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
                desired = -0.08
                kp = 0.5
                kd = 5
                ki = 100
                error = robot_pos[1] - desired

                proportional = error * kp

                derivative = (kd * error) / dt
                derivative = clamp(derivative, -10, 10)

                self.intergral += ki * error * dt
                self.intergral = clamp(self.intergral, -10, 10)

                u = clamp(proportional + self.intergral + derivative, -10, 10)

                print(proportional, derivative, self.intergral)
                
                if abs(u) > 0.1:
                    self.left_motor.setVelocity(u)
                    self.right_motor.setVelocity(u)
                else:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                
                t2 = time.time()
                dt = t2 - t1 
                time.sleep(dt-0.000003)