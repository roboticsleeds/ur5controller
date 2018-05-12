#!/usr/bin/env python

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

import IPython
from ur5_factory import UR5_Factory

__author__ = "Rafael Papallas"
__authors__ = ["Rafael Papallas"]
__copyright__ = "Copyright (C) 2018, The University of Leeds"
__credits__ = ["Rafael Papallas", "Dr. Mehmet Dogar"]
__email__ = "Rafael: r.papallas@leeds.ac.uk  |  Mehmet: m.r.dogar@leeds.ac.uk"
__license__ = "GPLv3"
__maintainer__ = "Rafael Papallas"
__status__ = "Production"
__version__ = "0.0.1"

if __name__ == "__main__":
  ur5_factory = UR5_Factory()

  # The create_ur5_and_env() method takes also the following optional
  # arguments: is_simulation, has_ridgeback, gripper_name,
  # has_force_torque_sensor, env_path, viewer_name, urdf_path, and
  # srdf_path. See ur5_factory.py class for more details.
  env, robot = ur5_factory.create_ur5_and_env(is_simulation=True, has_force_torque_sensor=False)

  IPython.embed()
