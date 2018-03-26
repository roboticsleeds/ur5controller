#!/usr/bin/env python

# Copyright (C) 2017 The University of Leeds
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import IPython
import numpy
from openravepy import IkFilterOptions
from openravepy import matrixFromAxisAngle
from openravepy import PlanningError
from ur5_factory import UR5_Factory

__author__ = "Rafael Papallas"
__authors__ = ["Rafael Papallas"]
__copyright__ = "Copyright (C) 2018, The University of Leeds"
__credits__ = ["Rafael Papallas", "Dr. Mehmet Dogar"]
__email__ = "Rafael: r.papallas@leeds.ac.uk  |  Mehmet: m.r.dogar@leeds.ac.uk"
__license__ = "GPLv3"
__maintainer__ = "Rafael Papallas"
__status__ = "Production"
__version__ = "0.0.1"


class Demo:
    def __init__(self):
        ur5_factory = UR5_Factory()

        # The create_ur5_and_env() method takes also the following optional
        # arguments: is_simulation, has_ridgeback, gripper_name,
        # has_force_torque_sensor, env_path, viewer_name, urdf_path, and
        # srdf_path. See ur5_factory.py class for more details.
        self.env, self.robot = ur5_factory.create_ur5_and_env()

        self.move_robot_to_start_transform()

    def move_robot_to_start_transform(self):
        # Move manipulator to the start transform (just above the table
        # surface)
        start_transform = numpy.eye(4)
        start_transform[0, 3] = 0.80917
        start_transform[1, 3] = 0.05212
        start_transform[2, 3] = 0.81

        ik_solution = self.robot.manipulator.FindIKSolution(start_transform, IkFilterOptions.CheckEnvCollisions)

        self.robot.base_manipulation.MoveManipulator(goal=ik_solution)

    def is_trajectory_safe(self, trajectory_object):
        waypoints = trajectory_object.GetAllWaypoints2D()
        waypoint_sums = [0, 0, 0, 0, 0, 0]
        for i in range(0, len(waypoints) - 1):
            for j in range(0, 6):
                difference = abs(waypoints[i][j] - waypoints[i+1][j])
                waypoint_sums[j] += difference

        waypoint_below_threshold = [x < numpy.pi/3 for x in waypoint_sums]
        return all(waypoint_below_threshold)

    def execute_trajectory_object(self, trajectory_object):
        try:
            self.robot.GetController().SetPath(trajectory_object)
            self.robot.WaitForController(0)
        except planning_error, e:
            print "ERROR: There was an error when executing the trajectory."
            print e

    def rotate_hand(self, rotation_matrix):
        current_hand_transform = self.robot.end_effector_transform
        goal_transform = numpy.dot(current_hand_transform, rotation_matrix)
        ik_solution = self.robot.manipulator.FindIKSolution(goal_transform, IkFilterOptions.CheckEnvCollisions)

        if ik_solution is not None:
            try:
                trajectory_object = self.robot.base_manipulation.MoveManipulator(goal=ik_solution, execute=False, outputtrajobj=True)
            except PlanningError, e:
                print "There was a planning error"
                print e

            if trajectory_object and self.is_trajectory_safe(trajectory_object):
                self.execute_trajectory_object(trajectory_object)
            else:
                print "The trajectory failed the safety check."
        else:
            print "No IK Solution found."

    def move_hand(self, x_offset, y_offset):
        current_hand_transform = self.robot.end_effector_transform
        current_hand_transform[0, 3] += x_offset
        current_hand_transform[1, 3] += y_offset

        ik_solution = self.robot.manipulator.FindIKSolution(current_hand_transform, IkFilterOptions.CheckEnvCollisions)

        if ik_solution is not None:
            try:
                trajectory_object = self.robot.base_manipulation.MoveManipulator(goal=ik_solution, execute=False, outputtrajobj=True)
            except PlanningError, e:
                print "There was a planning error"
                print e

            if trajectory_object and self.is_trajectory_safe(trajectory_object):
                self.execute_trajectory_object(trajectory_object)
            else:
                print "The trajectory failed the safety check."
        else:
            print "No IK Solution found."

    def control_robot_with_the_keyboard(self):
        OFFSET = 0.07

        print ""
        print "w: Move arm forward."
        print "s: Move arm backwards."
        print "a: Move arm left."
        print "d: Move arm right."
        print "q: Rotate end-effector anticlockwise."
        print "e: Rotate end-effector clockwise."
        print "o: Open end-effector."
        print "c: Close end-effector."
        print ""
        print "Type 'exit' to stop the demo and enter in IPython environment."

        while True:
            action = raw_input("Menu Action: ")

            if action == "exit": # EXIT Demo
                break
            if action == "q": # Rotate Anti-clockwise
                anticlockwise = matrixFromAxisAngle([0, 0, numpy.pi/6])
                self.rotate_hand(anticlockwise)
            elif action == "e": # Rotate Clockwise
                clockwise = matrixFromAxisAngle([0, 0, -numpy.pi/6])
                self.rotate_hand(clockwise)
            elif action == "o": # Open Gripper
                self.robot.open_gripper()
            elif action == "c": # Close Gripper
                self.robot.close_gripper()
            elif action in ["w", "s", "a", "d"]:
                x_offset = 0
                y_offset = 0

                if action == "w": x_offset = OFFSET   # Move Forward
                if action == "s": x_offset = -OFFSET  # Move Backwards
                if action == "a": y_offset = -OFFSET  # Move Left
                if action == "d": y_offset = OFFSET   # Move Right

                self.move_hand(x_offset, y_offset)


if __name__ == "__main__":
    demo = Demo()
    demo.control_robot_with_the_keyboard()

    IPython.embed()
