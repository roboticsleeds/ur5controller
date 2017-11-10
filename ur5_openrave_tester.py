from openravepy import *
import time

# Genral/Global settings
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
urdf_module = RaveCreateModule(env, 'urdf')

with env:
    arg = 'LoadURI ur5.urdf ur5.srdf'
    bh_name = urdf_module.SendCommand(arg)
    bh_body = env.GetKinBody(bh_name)

