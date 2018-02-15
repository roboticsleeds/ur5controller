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

    # multicontroller = RaveCreateMultiController(env, "")
    # robot.SetController(multicontroller)
    #
    # robot_controller = RaveCreateController(env,'ur5controller')
    # hand_controller = RaveCreateController(env, 'robotiqcontroller')
    #
    # multicontroller.AttachController(robot_controller, [7, 6, 0, 9, 10, 11], 0)
    # multicontroller.AttachController(hand_controller, [8], 0)

    manip = robot.SetActiveManipulator(robot.GetManipulators()[0])
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # Required for or_rviz to work with the robot's interactive marker.
    ik_solver = RaveCreateIkSolver(env, ikmodel.getikname())
    manip.SetIkSolver(ik_solver)

    # Add the robot to the environment
    env.Add(robot, True)
    return robot

def move_hand_straight(start_transform, x_offset, y_offset):
    x_initial = start_transform[0][3]
    y_initial = start_transform[1][3]

    x_goal = x_initial - x_offset
    y_goal = y_initial - y_offset

    x = x_goal - x_initial
    y = y_goal - y_initial

    direction = [x, y, 0] / linalg.norm([x, y, 0])
    step_size = 0.01
    max_steps = (linalg.norm([x, y, 0]) / step_size) + 1
    min_steps = max_steps / 2

    try:
        manipprob.MoveHandStraight(direction=direction,
                                   starteematrix=start_transform,
                                   stepsize=step_size,
                                   minsteps=min_steps,
                                   maxsteps=max_steps)
        robot.WaitForController(0)
    except planning_error, e:
        print e

def rotate_hand(rotation_matrix):
    current_hand_transform = robot.GetManipulators()[0].GetTransform()
    goal_transform = numpy.dot(current_hand_transform, rotation_matrix)
    ik_solution = manip.FindIKSolution(goal_transform, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    try:
        manipprob.MoveManipulator(goal=ik_solution)
    except planning_error, e:
        print e

if __name__ == "__main__":
    env = Environment()
    env.Load('test_env.xml')
    env.SetViewer('qtcoin')

    # At this point, the robot should load in the viewer and replicate the
    # current configuration of the physical robot.
    robot = create_ur5(env)

    robot.SetTransform(array([[ 9.99945700e-01, -1.04210156e-02, -1.24229304e-09, 3.03276211e-01],
                              [ 1.04210156e-02,  9.99945700e-01, -6.45894224e-12, -1.31079838e-01],
                              [ 1.24229289e-09, -6.48736357e-12,  1.00000000e+00, 2.52894759e-02],
                              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]))

    robot.SetDOFValues(array([ 1.53822374, 0, 0, 0, 0, 0, -0.81608755, 1.58696198, 0, -0.73290283, 1.61193109, 0.0235535]))

    print "===================================================================="
    print "                         UR5 CONTROLLER DEMO"
    print "===================================================================="
    print ""
    print "Controlling UR5 Robot in 2D over a table."
    print ""
    print "=========="
    print "   MENU"
    print "=========="
    print ""
    print " q     w    e "
    print ""
    print " a     +    d "
    print ""
    print "       s      "
    print ""
    print "w -> moves the arm forward."
    print "s -> moves the arm backwards."
    print "a -> moves the arm left."
    print "d -> moves the arm right."
    print "q -> rotate end-effector anti-clockwise."
    print "e -> rotate end-effector clockwise."
    print "o -> Opens the end-effector."
    print "c -> Closes the end-effector."
    print ""
    print "'exit' to stop the demo."

    manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
    task_manipulation = interfaces.TaskManipulation(robot)
    manip = robot.SetActiveManipulator(robot.GetManipulators()[0])
    OFFSET = 0.07

    while True:
        action = raw_input("Menu Action: ")
        if action == "exit":  # EXIT Demo
            break
        if action == "q":     # Rotate Anti-clockwise
            rotation_matrix = matrixFromAxisAngle([0, 0, -numpy.pi/6])
            rotate_hand(rotation_matrix)
        elif action == "e":   # Rotate Clockwise
            rotation_matrix = matrixFromAxisAngle([0, 0, numpy.pi/6])
            rotate_hand(rotation_matrix)
        elif action == "o":   # Open Gripper
            pass
        elif action == "c":   # Close Gripper
            task_manipulation.CloseFingers()
        elif action in ["w", "s", "a", "d"]:
            start_transform = robot.GetManipulators()[0].GetEndEffectorTransform()

            if action == "w":   # Forward
                x_offset = OFFSET
                y_offset = 0
            elif action == "s": # Backwards
                x_offset = -OFFSET
                y_offset = 0
            elif action == "a": # Left
                x_offset = 0
                y_offset = OFFSET
            elif action == "d": # Right
                x_offset = 0
                y_offset = -OFFSET

            current_hand_transform = robot.GetManipulators()[0].GetTransform()
            x = numpy.identity(4)

            x[0, 3] += x_offset
            x[1, 3] += y_offset

            goal_transform = numpy.dot(current_hand_transform, x)
            ik_solution = manip.FindIKSolution(goal_transform, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
            try:
                manipprob.MoveManipulator(goal=ik_solution)
            except planning_error, e:
                print e

            # move_hand_straight(start_transform, x_offset, y_offset)

    IPython.embed()
