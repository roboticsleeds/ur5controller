#!/usr/bin/env python

# Copyright (C) 2018 The University of Leeds
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
#
# Author: Rafael Papallas (http://papallas.me)

__author__ = "Rafael Papallas"
__authors__ = ["Rafael Papallas"]
__copyright__ = "Copyright (C) 2018, The University of Leeds"
__credits__ = ["Rafael Papallas", "Dr. Mehmet Dogar"]
__email__ = "Rafael: r.papallas@leeds.ac.uk  |  Mehmet: m.r.dogar@leeds.ac.uk"
__license__ = "GPLv3"
__maintainer__ = "Rafael Papallas"
__status__ = "Production"
__version__ = "0.0.1"

from openravepy import Robot
from openravepy import interfaces
from openravepy import databases
from openravepy import IkParameterization


class UR5_Robot(Robot):
    def __init__(self):
        self.robot_name = "UR5"
        self._OPENRAVE_GRIPPER_MAX_VALUE = 0.87266444
        self._ROBOT_GRIPPER_MAX_VALUE = 255

        self.manipulator = self.SetActiveManipulator(self.GetManipulators()[0])
        self.task_manipulation = interfaces.TaskManipulation(self)
        self.base_manipulation = interfaces.BaseManipulation(self)

        # Needed for "find a grasp" function (not parsed using or_urdf hence
        # needs manual setting)
        self.GetManipulators()[0].SetChuckingDirection([1.0])
        self.GetManipulators()[0].SetLocalToolDirection([1.0, 0, 0])

        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self, iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            print "The IKModel is now being generated. Please be patient, this will take a while (sometimes up to 30 minutes)..."
            self.ikmodel.autogenerate()

    @property
    def end_effector_transform(self):
        """End-Effector's current transform property."""
        return self.manipulator.GetTransform()

    def set_gripper_openning(self, value):
        if value < 0 or value > 255:
            raise ValueError("Gripper value should be between 0 and 255.")

        # This conversion is required because SetDesired() controller
        # requires the value to be in terms of a value between 0 and
        # 0.87266444 (OpenRAVE model limit values) and then is converting
        # it to a value between 0-255 for the low-level gripper driver.
        # To make this method more user-friendly, the value is expected
        # to be between 0 and 255, then we map it down to 0 to 0.87266444
        # and send it to the gripper controller.
        model_value = self._OPENRAVE_GRIPPER_MAX_VALUE / self._ROBOT_GRIPPER_MAX_VALUE * abs(value);
        dof_values = self.GetDOFValues()
        dof_values[3] = model_value

        self.GetController().SetDesired(dof_values)

    def open_gripper(self, kinbody=None):
        """
        Will release fingers (i.e open end-effector).

        Args:
            kinbody: Optionally, provide an OpenRAVE KinBody that will
                     be used to release it from the end-effector.
        """
        self.task_manipulation.ReleaseFingers(target=kinbody)
        self.WaitForController(0)

    def close_gripper(self):
        """Will close fingers of the end-effector until collision."""
        self.task_manipulation.CloseFingers()
        self.WaitForController(0)

    def execute_trajectory_and_wait_for_controller(self, trajectory):
        self.GetController().SetPath(trajectory)
        self.WaitForController(0)

