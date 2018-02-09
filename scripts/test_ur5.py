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
#
# Author: Rafael Papallas (http://papallas.me)

from openravepy import *
import IPython

def create_ur5(env, urdf_path=None, srdf_path=None):
    if urdf_path is None:
        urdf_path = "package://ur5controller/ur5_description/ur5.urdf"
    if srdf_path is None:
        srdf_path = "package://ur5controller/ur5_description/ur5.srdf"

    module = RaveCreateModule(env, 'urdf')
    with env:
        name = module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
        robot = env.GetRobot(name)

    multicontroller = RaveCreateMultiController(env, "")
    robot.SetController(multicontroller)

    robot_controller = RaveCreateController(env,'ur5controller')
    hand_controller = RaveCreateController(env, 'robotiqcontroller')

    multicontroller.AttachController(robot_controller, [2, 1, 0, 4, 5, 6], 0)
    multicontroller.AttachController(hand_controller, [3], 0)

    manip = robot.SetActiveManipulator(robot.GetManipulators()[0])
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,
                                                                 iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # Required for or_rviz to work with the robot's interactive marker.
    ik_solver = RaveCreateIkSolver(env, ikmodel.getikname())
    manip.SetIkSolver(ik_solver)

    # Needed for find a grasp function (not parsed using or_urdf hence needs manual setting)
    robot.GetManipulators()[0].SetChuckingDirection([1.0])
    robot.GetManipulators()[0].SetLocalToolDirection([1.0, 0, 0])

    # Add the robot to the environment
    env.Add(robot, True)


if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')

    # At this point, the robot should load in the viewer and replicate the
    # current configuration of the physical robot.
    ur5_robot = create_ur5(env)

    # Some simple manipulation task to check that the controller can control the
    # physical robot.

    # manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
    # manipprob.MoveManipulator(goal=[-1.5708, 0, 0, 0, 0, 0]) # call motion planner with goal joint angles
    # robot.WaitForController(0) # wait

    IPython.embed()
