#!/usr/bin/env python
"""
URDF File for UR5 Parser.

The URDF file requires some hard-coded file URIs that have to be of the format
file:///home/rafael/Desktop/ this file will take care of generating a valid
such file with such URIs by just providing the path.

The program will ask you to provide a valid path of the meshes directory for
UR5 of the format: /home/rafael/Desktop and then will take care to generate
the ur5.urdf file with the full URI path.

           Copyright (C)  2017/2018 Rafael Papallas

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import os

path = raw_input("Please provide the full address of meshes directory " + \
                 "(please don't use relative path but full path e.g " + \
                 "/home/rafael/Desktop/...) or EMPTY to use current directory: ")

if path is "":
    # Will return the current working direcotry
    path = os.getcwd()

full_path = "file://" + path

# If the last character of the full path is '/' then remove it.
if full_path[:-1] is "/": full_path[:-1] = ""

# If the path does not start with / add it
if full_path[7] is not "/": full_path = full_path[:6] + '/' + full_path[6:]

# Open the template path and generate a new file that includes the full valid
# paths.
with open("ur5_template.urdf", "rt") as fin:
    with open("ur5.urdf", "wt") as fout:
        for line in fin:
            fout.write(line.replace('{}', full_path))
