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
    PositionController = PID(45, 0, 0.005, 0, 0)
    AngleCotrnoller = PID(90, 20, 0.00005, 10, -10)

    BallPositionController = PID(10, 5, 0.0005, 10, -10)
    BallAngleCotrnoller = PID(20, 10, 0.00075, 10, -10)
    GoalPositionCotrnoller = PID(15, 0, 0.002, 0, 0)
    GoalAngleCotrnoller = PID(5, 30, 0.000045, 20, -20)

    def run(self):
        ball_data = None
        dt = None
        heading_stage = 0
        position_stage = 0
        inplace = False
        inplace_tick = 0

        loop_tick = 0

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                if data['waiting_for_kickoff']:
                    heading_stage = 0
                    position_stage = 0
                    loop_tick = 0
                    inplace = False

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

                if ball_data and ball_data['strength'] > 10 and inplace:
                    heading_stage = 0
                    position_stage = 0
                    loop_tick = 0
                    e_ball_position = 1 / ball_data['strength'] * 2.5
                    e_ball_theta = math.atan2(
                        ball_data["direction"][0], ball_data["direction"][1]) - (math.pi / 2)
                    e_goal_distance = math.sqrt(
                        (robot_pos[0])**2 + (-8 + robot_pos[1])**2)

                    v_ball_position = self.BallPositionController.update(
                        e_ball_position, dt)
                    v_ball_theta = self.BallAngleCotrnoller.update(
                        e_ball_theta, dt)
                    v_goal_theta = self.GoalAngleCotrnoller.update(heading, dt)
                    v_goal_position = self.GoalPositionCotrnoller.update(
                        e_goal_distance, dt)

                    vr = 0
                    vl = 0

                    if abs(v_ball_position) > 0.01:
                        w = v_ball_theta
                        v = v_ball_position
                        if abs(w) < 0.1: 
                            w = 0
                            self.BallPositionController.p_error = 50
                            self.BallPositionController.i_gain = 0
                        else: 
                            self.BallPositionController.p_error = 10
                            self.BallPositionController.i_gain = 5

                            v = self.BallPositionController.update(e_ball_position, dt)
                        vr = (2 * v + 0.085 * w) / 0.04
                        vl = (2 * v - 0.085 * w) / 0.04
                    elif abs(heading) > 0.1:
                        if heading > 0:
                            vr = -v_goal_theta
                            vl = v_goal_theta
                        else: 
                            vr = v_goal_theta
                            vl = -v_goal_theta
                    else:
                        vl = 10
                        vr = 10

                    self.left_motor.setVelocity(vl)
                    self.right_motor.setVelocity(vr)

                elif abs(robot_pos[0]) > 0.01 and (heading_stage == 0 or position_stage == 0):
                    if abs(heading - math.pi / 2) > 0.1 and heading_stage == 0:
                        u = self.AngleCotrnoller.update(
                            heading - math.pi / 2, dt)
                        self.left_motor.setVelocity(-u)
                        self.right_motor.setVelocity(u)

                    elif abs(robot_pos[0]) > 0.01 and position_stage == 0:
                        u = self.PositionController.update(-robot_pos[0], dt)
                        if u > 0:
                            u += 3
                        elif u < 0: 
                            u -= 3

                        self.left_motor.setVelocity(u)
                        self.right_motor.setVelocity(u)
                elif abs(robot_pos[1] - 0.55) > 0.01:
                    if abs(heading) > 0.1:
                        u = self.AngleCotrnoller.update(heading, dt)
                        self.left_motor.setVelocity(-u)
                        self.right_motor.setVelocity(u)
                        heading_stage = 1

                    elif abs(robot_pos[1] - 0.55) > 0.01:
                        u = self.PositionController.update(
                            robot_pos[1] - 0.55, dt)
                        
                        if u > 0:
                            u += 3
                        elif u < 0: 
                            u -= 3

                        self.left_motor.setVelocity(u)
                        self.right_motor.setVelocity(u)
                        position_stage = 1
                        if abs(robot_pos[1] - 0.55) < 0.1:
                            inplace = True
                        else: 
                            inplace = False
                else:
                    inplace = True
                    if loop_tick < 20:
                        self.left_motor.setVelocity(5)
                        self.right_motor.setVelocity(5)
                    else:
                        self.left_motor.setVelocity(-5)
                        self.right_motor.setVelocity(-5)

                t2 = time.time()
                dt = t2 - t1
                # time.sleep(dt - T)

                if inplace:
                    if abs(robot_pos[1] - 0.55) > 0.5:
                        inplace = False
                    inplace_tick += 1

                    if inplace_tick == 60:
                        heading_stage = 0
                        position_stage = 0
                        loop_tick = 0
                        inplace = False

                loop_tick += 1
                if loop_tick == 40:
                    loop_tick = 0
