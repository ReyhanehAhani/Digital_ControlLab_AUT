# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
    
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                #print("heading",math.degrees(heading))
                
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                
                
                x_desired = -0.09
                y_desired = -0.020
                kp = 20

                angle_error = math.atan2(ball_data["direction"][0], ball_data["direction"][1]) - (math.pi/2)

                postion_error = 1/ball_data['strength'] * 5
                self.left_motor.setVelocity(postion_error * kp + angle_error * -1 * kp)
                self.right_motor.setVelocity(postion_error * kp + angle_error * kp)


