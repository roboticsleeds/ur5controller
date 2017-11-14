from openravepy import *
import numpy

env = Environment()
env.SetViewer('qtcoin')
module = RaveCreateModule(env, 'urdf')

with env:
    name = module.SendCommand('LoadURI /home/rafael/Documents/ur5_openrave/ur5.urdf /home/rafael/Documents/ur5_openrave/ur5.srdf')
    robot = env.GetRobot(name)

# Add the robot to the environment
env.Add(robot,True)

# this binds UR5 controller to the robot
probotcontroller = RaveCreateController(env,'ur5controller')
robot.SetController(probotcontroller)
