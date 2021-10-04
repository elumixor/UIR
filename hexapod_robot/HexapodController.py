#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# import messages

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

    def goto_reactive(self, goal, odometry, collision, laser_scan: LaserScan):
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

        distances = laser_scan.distances
        range_min = laser_scan.range_min
        range_max = laser_scan.range_max

        half = len(distances) // 2

        left = distances[:half]
        right = distances[half:]

        scan_left = float("inf")
        scan_right = float("inf")

        for d in left:
            if d < range_min or d > range_max:
                continue

            if d < scan_left:
                scan_left = d

        for d in right:
            if d < range_min or d > range_max:
                continue

            if d < scan_right:
                scan_right = d

        repulsive_force = 1 / scan_left - 1 / scan_right

        print(f"[{goal.position.x}, {goal.position.y}] :: [{robot.position.x:.3f}, {robot.position.y:.3f}] :: {scan_left:.3f} :: {scan_right:.3f}")

        angular_speed_navigation_component = dphi * C_TURNING_SPEED
        angular_speed_avoidance_component = repulsive_force * C_AVOID_SPEED
        angular_speed = angular_speed_navigation_component + angular_speed_avoidance_component

        linear = distance

        return Twist(Vector3(x=linear), Vector3(z=angular_speed))
