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
import time
from numpy import *
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



    # Needed for find a grasp function (not parsed using or_urdf hence needs manual setting)
    robot.GetManipulators()[0].SetChuckingDirection([1.0])
    robot.GetManipulators()[0].SetLocalToolDirection([1.0, 0, 0])


    multicontroller = RaveCreateMultiController(env, "")
    robot.SetController(multicontroller)

    robot_controller = RaveCreateController(env,'ur5controller')
    hand_controller = RaveCreateController(env, 'robotiqcontroller')

    multicontroller.AttachController(robot_controller, [2, 1, 0, 4, 5, 6], 0)
    multicontroller.AttachController(hand_controller, [3], 0)

    # Add the robot to the environment
    env.Add(robot, True)
    return robot


if __name__ == "__main__":
    env = Environment()
    env.Load('test_env.xml')
    env.SetViewer('qtcoin')

    # At this point, the robot should load in the viewer and replicate the
    # current configuration of the physical robot.
    robot = create_ur5(env)

    manip = robot.SetActiveManipulator(robot.GetManipulators()[0])
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,
                                                                 iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # Required for or_rviz to work with the robot's interactive marker.
    ik_solver = RaveCreateIkSolver(env, ikmodel.getikname())
    manip.SetIkSolver(ik_solver)

    # time.sleep(3)

    robot.SetTransform(array([[  1.26915621e-01,   9.91913517e-01,   1.18245318e-07, 6.25871897e-01],
                              [ -9.91913517e-01,   1.26915621e-01,  -1.04079755e-07, -7.71137774e-02],
                              [ -1.18245294e-07,  -1.04079782e-07,   1.00000000e+00, 7.75881767e-01],
                              [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]]))

    # robot.SetDOFValues(array([ 1.91162348, -0.97853786,  2.21942425,  0.        , -0.90290195,
    #     2.39421415,  0.12924275]))

    # Some simple manipulation task to check that the controller can control the
    # physical robot.

    # time.sleep(3)

    while True:
        try:
            action = float(raw_input("direction: "))
            if action == 8.0:   # Forward
                xOffset = -0.05
                yOffset = 0.0
            elif action == 2.0: # Backwards
                xOffset = 0.1
                yOffset = 0.0
            elif action == 4.0: # Left
                xOffset = 0.0
                yOffset = -0.05
            elif action == 6.0: # Right
                xOffset = 0.0
                yOffset = 0.05
            else:
                xOffset = 0.0
                yOffset = 0.0

            manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs

            t = robot.GetManipulators()[0].GetTransform()
            x_initial = t[0][3]
            y_initial = t[1][3]

            x_goal = x_initial - xOffset
            y_goal = y_initial - yOffset

            x = x_goal - x_initial
            y = y_goal - y_initial

            direction = [x, y, 0] / linalg.norm([x, y, 0])
            step_size = 0.01
            max_steps = (linalg.norm([x, y, 0]) / step_size) + 1
            min_steps = max_steps / 2
            start_transform = t

            try:
                traj = manipprob.MoveHandStraight(direction=direction,
                                           starteematrix=start_transform,
                                           stepsize=step_size,
                                           minsteps=min_steps,
                                           maxsteps=max_steps,
                                           outputtraj=True,
                                           )
                #print traj
            except planning_error,e:
                print e
        except KeyboardInterrupt:
            break

    # manipprob.MoveToHandPosition(matrices=[current_hand_transform], seedik=10)
    # robot.WaitForController(0)

    # manipprob.MoveManipulator(goal=[0, -1.57, 0, 0, 0, 0]) # call motion planner with goal joint angles
    # robot.WaitForController(0) # wait
    #
    # task_manipulation = interfaces.TaskManipulation(robot)
    # task_manipulation.CloseFingers()

    IPython.embed()
