//  Copyright (C) 2017 Rafael Papallas
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "ros/ros.h"
#include "plugindefs.h"
#include <openrave/plugin.h>

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream &sinput);

/**
	This is an interface for the UR5 controller.
*/
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string &interfaceName, std::istream &sinput,
                                          EnvironmentBasePtr penv) {
    switch (type)
    {
        case PT_Controller:
            if (interfaceName == "ur5controller")
            {
                if (!ros::isInitialized())
                {
                    int argc = 0;
                    std::string node_name = "ur5controller";
                    ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
                    RAVELOG_INFO("Starting ROS node '%s'.\n", node_name.c_str());
                }
                else
                {
                    RAVELOG_INFO("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
                }

                return CreateUr5Controller(penv, sinput);
            }
            break;
        default:
            break;
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_Controller].push_back("Ur5Controller");
}

OPENRAVE_PLUGIN_API void DestroyPlugin() {}
