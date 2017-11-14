#include "plugindefs.h"
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

using namespace std;
using namespace OpenRAVE;

/**
	This is a controller for the UR5 robot. The main purpose of this class is to
	subscribe to ROS topic that publishes the joint angles of the robot. By listening
	to those values we can make the OpenRAVE robot model change in real-time as with
	the robot or vice versa.

	@author Rafael Papallas
	@version 1.0 14/11/2017
*/
class Ur5Controller : public ControllerBase
{
	public:
		Ur5Controller(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv)
		{
			__description = ":Interface Authors: Rafael Papallas, Mehmet Dogar";

			_nControlTransformation = 0;
			_penv = penv;
			_paused = false;
			_initialized = false;
		}

		/**
			Callback when the joint values changes. Responsible to change the robot
			model in OpenRAVE.

			@param msg the message sent by the subscriber (i.e the new joint values)
		*/
		void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
		  if (_paused)
		  {
		  	return;
		  }

			// Create a joint vector for angles and assign the new values from message.
		  std::vector <double> joint_angles;
		  for (unsigned int i = 0; i < (msg->position).size(); i++)
		  {
		    joint_angles.push_back((msg->position).at(i));
		  }

			// Set DOF Values of the joint angles just received from message to the
			// robot in OpenRAVE.
      OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
      _probot->SetDOFValues(joint_angles,
														KinBody::CLA_CheckLimitsSilent);
		}

		/**
			Initialiser when the robot is attached to the controller.

			Subscribes to the topic /joint_states that listens to changes to the
			robot joints and calls a callback to act on the new values.
		*/
		virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
		{
			_probot = robot;

      if ( !!_probot )
      {
	      _nControlTransformation = nControlTransformation;
      }

			int dofindices_array[] = {1,2,3,4,5,6};

			for (int i = 0; i<=5;i++)
			{
				_dofindices.push_back(dofindices_array[i]);
			}

			_pn = new ros::NodeHandle();
			_joint_angles_sub = _pn->subscribe("/joint_states", 1, &Ur5Controller::JointStateCallback, this);

			return true;
		}

		virtual void Reset(int options)
		{
		}

		virtual const std::vector<int>& GetControlDOFIndices() const
		{
			return _dofindices;
		}

		virtual int IsControlTransformation() const
		{
			return _nControlTransformation;
		}

		virtual bool SetDesired(const std::vector<OpenRAVE::dReal>& values, TransformConstPtr trans)
		{
			return true;
		}

		virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
		{
			return false;
		}

		virtual void SimulationStep(OpenRAVE::dReal fTimeElapsed)
		{
			ros::spinOnce();
		}

		virtual bool IsDone()
		{
			return false;
		}

		virtual OpenRAVE::dReal GetTime() const
		{
			return 0;
		}

		virtual RobotBasePtr GetRobot() const
		{
			return _probot;
		}


	private:
		bool _initialized;
		RobotBasePtr _probot;
		std::vector<int> _dofindices;
		int _nControlTransformation;
		bool _paused;
		ros::Subscriber _joint_angles_sub;

		ros::NodeHandle* _pn;
		EnvironmentBasePtr _penv;

};

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream& sinput)
{
	return ControllerBasePtr(new Ur5Controller(penv,sinput));
}
