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
import time

# Genral/Global settings
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
urdf_module = RaveCreateModule(env, 'urdf')

with env:
    arg = 'LoadURI ur5.urdf ur5.srdf'
    bh_name = urdf_module.SendCommand(arg)
    bh_body = env.GetKinBody(bh_name)
