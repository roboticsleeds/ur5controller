from openravepy import *
import numpy
import os

env = Environment()
env.SetViewer('qtcoin')
module = RaveCreateModule(env, 'urdf')

with env:
    # Get the path of SRDF and URDF from the current working directory.
    current_working_directory = os.getcwd()
    srdf_path = current_working_directory + "/ur5_model/ur5.srdf"
    urdf_path = current_working_directory + "/ur5_model/ur5.urdf"

    name = module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
    robot = env.GetRobot(name)

# Add the robot to the environment
env.Add(robot,True)

# this binds UR5 controller to the robot
probotcontroller = RaveCreateController(env,'ur5controller')
robot.SetController(probotcontroller)

manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.5]) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
