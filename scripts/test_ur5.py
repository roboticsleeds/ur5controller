# Copyright (C) 2017 Rafael Papallas
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

from openravepy import *
import os
import IPython


env = Environment()
env.SetViewer('qtcoin')
module = RaveCreateModule(env, 'urdf')


with env:
    # Get the path of SRDF and URDF from the current working directory.
    current_working_directory = os.getcwd()
    srdf_path = "/home/rafael/catkin_ws/src/ur5controller/ur5_model/ur5.srdf"
    urdf_path = "/home/rafael/catkin_ws/src/ur5controller/ur5_model/ur5.urdf"

    name = module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
    robot = env.GetRobot(name)

# Add the robot to the environment
env.Add(robot,True)

# ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
# if not ikmodel.load():
#     ikmodel.autogenerate()


# this binds UR5 controller to the robot
robot_controller = RaveCreateController(env,'ur5controller')
hand_controller = RaveCreateController(env, 'robotiqcontroller')
#
multicontroller = RaveCreateMultiController(env, "")
robot.SetController(multicontroller)
#
multicontroller.AttachController(robot_controller, robot_controller.GetControlDOFIndices(), 0)
multicontroller.AttachController(hand_controller, hand_controller.GetControlDOFIndices(), 0)

# manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
# manipprob.MoveManipulator(goal=[-1.5708, 0, 0, 0, 0, 0]) # call motion planner with goal joint angles
# robot.WaitForController(0) # wait

IPython.embed()
