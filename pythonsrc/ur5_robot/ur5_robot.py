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

from openravepy import Robot
from openravepy import RaveCreateMultiController
from openravepy import RaveCreateController
from openravepy import interfaces
from openravepy import databases
from openravepy import IkParameterization


class UR5_Robot(Robot):
    def __init__(self, is_simulation):
        self.robot_name = "UR5"

        if not is_simulation:
            self.multicontroller = RaveCreateMultiController(self.env, "")
            self.SetController(self.multicontroller)

            robot_controller = RaveCreateController(self.env, 'ur5controller')
            hand_controller = RaveCreateController(self.env, 'robotiqcontroller')

            self.multicontroller.AttachController(robot_controller, [2, 1, 0, 4, 5, 6], 0)
            self.multicontroller.AttachController(hand_controller, [3], 0)

        self.manipulator = self.SetActiveManipulator(self.GetManipulators()[0])
        self.task_manipulation = interfaces.TaskManipulation(self)
        self.base_manipulation = interfaces.BaseManipulation(self)

        # Needed for "find a grasp" function (not parsed using or_urdf hence
        # needs manual setting)
        self.GetManipulators()[0].SetChuckingDirection([1.0])
        self.GetManipulators()[0].SetLocalToolDirection([1.0, 0, 0])

        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self, iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            print "The IKModel is now being generated. Please being patient, this will take a while (sometimes up to 30 minutes)..."
            self.ikmodel.autogenerate()

    @property
    def end_effector_transform(self):
        return self.manipulator.GetTransform()
