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

print(robot.GetTransform())

# position: [-7.104050955497598e-05, 6.690283570076616e-05, 0.0004613414283554107, 5.438328769979961e-05, 2.811695286197846e-05, 1.649085228461189e-05]
print("Test 1")
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
manipprob.MoveManipulator(goal=[-0.791174970479432e-05, -0.570702650942672, 0.00016744036894245085, -1.5707646492896767, 2.1162518248907247e-05, 0.20651503552483e-06]) # call motion planner with goal joint angles
robot.WaitForController(0) # wait

print("Test")
# array([[ 1.,  0.,  0.,  0.],
#        [ 0.,  1.,  0.,  0.],
#        [ 0.,  0.,  1.,  0.5],
#        [ 0.,  0.,  0.,  1.]])
