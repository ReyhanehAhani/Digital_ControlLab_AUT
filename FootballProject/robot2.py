# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import enum

from PIDController import *


def clamp(x, min, max):
    if x < min:
        x = min
    if x > max:
        x = max
    return x


class MessageType(enum.IntEnum):
    BALL_NOT_REACHABLE = 1
    BALL_BY_TEAMMATE = 2
    BALL_BY_OPPONENT = 3


class MyRobot2(RCJSoccerRobot):
    PositionController = PID(40, 0, 0, 0, 0)
    AngleCotrnoller = PID(40, 0, 0, 0, 0)

    def run(self):
        ball_data = None
        dt = None
        heading_stage = 0
        position_stage = 0

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                if data['waiting_for_kickoff']:
                    heading_stage = 0
                    position_stage = 0

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()['robot_id']  # noqa: F841
                    if team_data == MessageType.BALL_BY_OPPONENT:
                        # Handel grabbing the ball
                        get_ball = True
                    if team_data == MessageType.BALL_BY_TEAMMATE:
                        # Don't grab the ball
                        get_ball = False
                    if team_data == MessageType.BALL_NOT_REACHABLE:
                        # Return with ball data in next packets
                        pass

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    self.send_data_to_team(MessageType.BALL_NOT_REACHABLE)
                    ball_data = None

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

                compass_values = self.compass.getValues()

                t1 = time.time()
                T = 0.000003
                # Write scripts of the robot here


                if abs(robot_pos[0]) > 0.01 and (heading_stage == 0 or position_stage == 0):
                    print("if section")
                    if abs(heading - math.pi/2) > 0.1 and heading_stage == 0: 
                        print("0")
                        u = self.AngleCotrnoller.update(heading - math.pi/2, dt)
                        self.left_motor.setVelocity(-u)
                        self.right_motor.setVelocity(u)

                    elif abs(robot_pos[0]) > 0.01 and position_stage == 0: 
                        print("1")
                        u = self.PositionController.update(-robot_pos[0], dt)
                        self.left_motor.setVelocity(u)
                        self.right_motor.setVelocity(u)
                elif abs(robot_pos[1] - 0.65) > 0.01:
                    print("else section")
                    if abs(heading) > 0.1:
                        print("2")
                        u = self.AngleCotrnoller.update(heading, dt)
                        self.left_motor.setVelocity(-u)
                        self.right_motor.setVelocity(u)
                        heading_stage = 1

                    elif abs(robot_pos[1] - 0.65) > 0.01:
                        print("3")
                        u = self.PositionController.update(robot_pos[1] - 0.65, dt)
                        self.left_motor.setVelocity(u)
                        self.right_motor.setVelocity(u)
                        position_stage = 1
                    
                    elif ball_data: 
                        e_theta = math.atan2(
                            ball_data["direction"][0], ball_data["direction"][1]) - (math.pi / 2)
                        e_position = 1 / ball_data['strength'] * 2.5

                        if abs(e_theta) > 0.1: 
                            print("4")
                            u = self.AngleCotrnoller.update(e, dt)
                            self.left_motor.setVelocity(-u)
                            self.right_motor.setVelocity(u)
                        else: 
                            print("5")
                            self.left_motor.setVelocity(10)
                            self.right_motor.setVelocity(10)
                        
                t2 = time.time()
                dt = t2 - t1
                #time.sleep(dt - T)
