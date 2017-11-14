#include "ros/ros.h"
#include "plugindefs.h"
#include <openrave/plugin.h>

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream& sinput);

/**
	This is an interface for the UR5 controller.
*/
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type)
    {
      case PT_Controller:
        if( interfacename == "ur5controller")
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

          return CreateUr5Controller(penv,sinput);
        }
        break;
      default:
          break;
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
  info.interfacenames[PT_Controller].push_back("Ur5Controller");
}

OPENRAVE_PLUGIN_API void DestroyPlugin(){}
