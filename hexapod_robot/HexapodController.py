#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import messages
import math

from messages import *

DELTA_DISTANCE = 0.12
C_TURNING_SPEED = 5
C_AVOID_SPEED = 10


def dot(a: Vector3, b: Vector3):
    return a.x * b.x + a.y * b.y + a.z * b.z


class HexapodController:
    def __init__(self):
        self.go_straight = Twist(linear=Vector3(x=1.0))
        self.do_nothing = Twist()

    def goto(self, goal: Pose, odometry: Odometry, collision):
        """Method to steer the robot towards the goal position given its current 
           odometry and collision status
        Args:
            goal: Pose of the robot goal
            odometry: Perceived odometry of the robot
            collision: bool of the robot collision status
        Returns:
            cmd: Twist steering command
        """
        if collision:
            return None

        if odometry is None or goal is None:
            return self.do_nothing

        robot = odometry.pose
        goal = goal

        heading = robot.orientation.to_Euler()[0]

        robot_to_goal = goal.position - robot.position

        distance = robot_to_goal.norm()
        if distance < DELTA_DISTANCE:
            return None

        to_target = math.atan2(robot_to_goal.y, robot_to_goal.x)

        dphi = (to_target - heading) / math.pi

        if dphi < -1:
            dphi += 2
        elif dphi > 1:
            dphi -= 2

        linear = distance
        angular = dphi * C_TURNING_SPEED

        return Twist(Vector3(x=linear), Vector3(z=angular))

    def goto_reactive(self, goal, odometry, collision, laser_scan):
        """Method to steer the robot towards the goal position while avoiding
           contact with the obstacles given its current odometry, collision
           status and laser scan data
        Args:
            goal: Pose of the robot goal
            odometry: Perceived odometry of the robot
            collision: bool of the robot collision status
            laser_scan: LaserScan data perceived by the robot
        Returns:
            cmd: Twist steering command
        """
        if collision:
            return None

        if odometry is None or goal is None:
            return self.do_nothing

        robot = odometry.pose
        goal = goal

        heading = robot.orientation.to_Euler()[0]

        robot_to_goal = goal.position - robot.position

        distance = robot_to_goal.norm()
        if distance < DELTA_DISTANCE:
            return None

        to_target = math.atan2(robot_to_goal.y, robot_to_goal.x)

        dphi = (to_target - heading) / math.pi

        if dphi < -1:
            dphi += 2
        elif dphi > 1:
            dphi -= 2

        linear = distance
        angular = dphi * C_TURNING_SPEED

        return Twist(Vector3(x=linear), Vector3(z=angular))
