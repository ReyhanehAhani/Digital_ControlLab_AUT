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


class MyRobot1(RCJSoccerRobot):
    BallPositionController = PID(2, 5, 0.0005, 10, -10)
    BallAngleCotrnoller = PID(20, 10, 0.00075, 10, -10)
    GoalPositionCotrnoller = PID(15, 0, 0.002, 0, 0)
    GoalAngleCotrnoller = PID(5, 30, 0.000045, 20, -20)

    def run(self):
        ball_data = None
        dt = None
        heading_stage = 0
        heading_stage_counter = 0
        has_shooted = False

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841

                    if data['waiting_for_kickoff']:
                        heading_stage = 0

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

                if not ball_data:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                direction = utils.get_direction(ball_data["direction"])

                t1 = time.time()
                T = 0.000003

                # Write scripts of the robot here
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

                if heading_stage == 1:
                    self.BallPositionController.d_gain = 0.005
                else: 
                    self.BallPositionController.d_gain = 0.0005

                if abs(e_ball_position) > 0.2:
                    self.send_data_to_team(MessageType.BALL_BY_OPPONENT)
                if abs(e_ball_position) > 0.011 and heading_stage == 0:
                    w = v_ball_theta
                    v = v_ball_position
                    vr = (2 * v + 0.085 * w) / 0.04
                    vl = (2 * v - 0.085 * w) / 0.04
                elif abs(heading) > 0.1:
                    heading_stage = 1
                    if heading > 0:
                        vr = -v_goal_theta
                        vl = v_goal_theta
                    else: 
                        vr = v_goal_theta
                        vl = -v_goal_theta
                else:
                    heading_stage = 1
                    vl = v_goal_position
                    vr = v_goal_position

                self.left_motor.setVelocity(vl)
                self.right_motor.setVelocity(vr)

                t2 = time.time()
                dt = t2 - t1
                time.sleep(dt - T)

                if heading_stage != 0:
                    heading_stage_counter += 1
                    if heading_stage_counter == 40:
                        heading_stage_counter = 0
                        heading_stage = 0
