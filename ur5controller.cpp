#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/JointValue.h"
#include "brics_actuator/JointPositions.h"
#include "plugindefs.h"

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
			//RAVELOG_ERROR("Test 1");
		}

		/**
			Callback when the joint values changes. Responsible to change the robot
			model in OpenRAVE.

			@param msg the message sent by the subscriber (i.e the new joint values)
		*/
		void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
			//RAVELOG_ERROR("Test 2a");
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

			//RAVELOG_ERROR("Test 2b");
		}

		/**
			Initialiser when the robot is attached to the controller.

			Subscribes to the topic /joint_states that listens to changes to the
			robot joints and calls a callback to act on the new values.
		*/
		virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
		{
			_probot = robot;

			//RAVELOG_ERROR("Test 3a");


      if ( !!_probot )
      {
				_dofindices = dofindices;
	      _nControlTransformation = nControlTransformation;
      }

			// int dofindices_array[] = {1,2,3,4,5,6};
			//
			// for (int i = 0; i<=5;i++)
			// {
			// 	_dofindices.push_back(dofindices_array[i]);
			// }

			////RAVELOG_ERROR("Test 2");

			//RAVELOG_ERROR("Test 3b");


			_pn = new ros::NodeHandle();
			_joint_angles_sub = _pn->subscribe("/joint_states", 1, &Ur5Controller::JointStateCallback, this);
			_move_arm_pub = _pn->advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);


			//RAVELOG_ERROR("Test 3c");

			_traj = RaveCreateTrajectory(_penv,"");
			_traj->Init(robot->GetConfigurationSpecification());

			////RAVELOG_ERROR("Test 4");

			// _traj->Init(_probot->GetActiveConfigurationSpecification());

			// Class attribute indicating that the classs has been intialised.
			_initialized = true;

			////RAVELOG_ERROR("Test 5");

			//RAVELOG_ERROR("Test 3d");

			return true;
		}

		virtual void Reset(int options)
		{
			// _traj.reset();
			//RAVELOG_ERROR("Test 4");
		}

		virtual const std::vector<int>& GetControlDOFIndices() const
		{
			//RAVELOG_ERROR("Test 5");
			return _dofindices;
		}

		virtual int IsControlTransformation() const
		{
			//RAVELOG_ERROR("Test 6");
			return _nControlTransformation;
		}

		virtual bool SetDesired(const std::vector<OpenRAVE::dReal>& values, TransformConstPtr trans)
		{
			//RAVELOG_ERROR("Test 7");
			return true;
		}

		virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
		{
			//RAVELOG_ERROR("Test 8a");
			// _traj.reset();
			if (ptraj != NULL)
			{
				_traj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
				_traj->Clone(ptraj, Clone_Bodies);
			}
			//RAVELOG_ERROR("Test 8b");
			return true;
		}

		double IsSameArmConfig(vector<double> &config1, vector<double> &config2)
		{
			//RAVELOG_ERROR("Test 9a");
				for (unsigned int i = 0; i < config1.size(); i++)
				{
						if (std::fabs( config1[i] - config2[i] ) > 0.01)
						{
								return false;
						}
				}
				//RAVELOG_ERROR("Test 9b");
				return true;
		}

		bool IsArmAtConfig(vector<double> &config)
		{
				//RAVELOG_ERROR("Test 10a");
				std::vector<double> current_arm_config;
				static const int arr[] = {0, 1, 2, 3, 4, 5};
				vector<int> dofindices (arr, arr + sizeof(arr) / sizeof(arr[0]) );
				_probot->GetDOFValues(current_arm_config, dofindices);

				//RAVELOG_ERROR("Test 10b");
				return IsSameArmConfig(current_arm_config,config);
		}

		// bool MoveArm(ostream& sout, istream& sinput){
		// 		std::string valuesString;
		// 		sinput >> valuesString;
		//
		// 		ROS_INFO("input %s", valuesString.c_str());
		//
		// 		vector<std::string> valuestokens;
		// 		boost::split(valuestokens, valuesString, boost::is_any_of("\t,"));
		//
		// 		vector<double> values;
		//
		// 		for (size_t i = 0; i < valuestokens.size(); ++i)
		// 		{
		// 				values.push_back(boost::lexical_cast<double>(valuestokens[i]));
		// 		}
		//
		// 		bool ret_val = MoveArm(values);
		//
		// 		if (ret_val){
		// 				sout << "True";
		// 				return true;
		// 		}
		// 		sout << "False";
		// 		return false;
		// }

		bool MoveArm(vector<double> values)
		{
			  //RAVELOG_ERROR("Test 11a");
				if (ros::ok() && !_paused){

						// for (size_t i = 0; i < _offset.size(); ++i)
						// {
						// 		values[i] += _offset.at(i);
						// }
					 trajectory_msgs::JointTrajectory traj;
					 trajectory_msgs::JointTrajectoryPoint points_n;

           traj.header.stamp = ros::Time::now();
           traj.header.frame_id = "base_link";
           traj.joint_names.resize(6);
           traj.points.resize(1);

					 traj.points[0].positions.resize(6);

           traj.joint_names[0] ="elbow_joint";
           traj.joint_names[1] ="shoulder_lift_joint";
           traj.joint_names[2] ="shoulder_pan_joint";
					 traj.joint_names[3] ="wrist_1_joint";
					 traj.joint_names[4] ="wrist_2_joint";
					 traj.joint_names[5] ="wrist_3_joint";

           for(int j = 0; j < 6; j++) {
						 traj.points[0].positions[j] = values[j];
           }

					 traj.points[0].time_from_start = ros::Duration(1);

						ROS_INFO("the values are %f %f %f %f %f", values[0], values[1], values[2], values[3], values[4]);
						_move_arm_pub.publish(traj);
						ros::spinOnce();

						// Store last values for later.
            _last_arm_command = values;

						// Store last values for later.
						// _last_arm_command = values;
						//RAVELOG_ERROR("Test 11b");
						return true;
				}
				////RAVELOG_ERROR("Test 12");
				return false;
		}

		bool MoveArmTowards(vector<double> &config)
		{
			//RAVELOG_ERROR("Test 12a");
				if (IsArmAtConfig(config))
				{
						return true; // already there.
				}

				if (_last_arm_command.empty() || !IsSameArmConfig(_last_arm_command,config))
				{
						MoveArm(config);
						_last_arm_command = config;
				}

				//RAVELOG_ERROR("Test 12b");

				return false;
		}

		virtual void SimulationStep(dReal fTimeElapsed)
		{
			//RAVELOG_ERROR("Test 13a");
			//////RAVELOG_ERROR("Test 14a");
			if(!_initialized)
			{
				return;
			}

			//RAVELOG_ERROR("Test 13b");

			//if(_traj != NULL) {
			// 	//RAVELOG_ERROR("Test ", _traj->GetNumWaypoints());
			// 	TrajectoryBasePtr ptraj;
			// 	// ptraj = RaveCreateTrajectory(GetEnv(), _traj->GetXMLId());
			// 	ptraj->Clone(_traj, Clone_Bodies);
			// 	//RAVELOG_ERROR("Test ", ptraj->GetNumWaypoints());
			// }
			//return;
			// TrajectoryBasePtr ptraj = _traj;
			// ptraj->Clone(_traj, Clone_Bodies);
			//////
			//
			//RAVELOG_ERROR("Test ", _traj->GetNumWaypoints());
			// ////////RAVELOG_ERROR("Test 14c");
			if (_traj->GetNumWaypoints() > 0)
			{
				//RAVELOG_ERROR("Test 13c");

				vector<dReal> waypoint;

				_traj->GetWaypoint(0, waypoint);

				////RAVELOG_ERROR("Test 14d");
				//RAVELOG_ERROR("Test 13d");
				static const int arr[] = {0, 1, 2, 3, 4, 5};
				std::vector< int > arm_indices(arr,arr+sizeof(arr)/sizeof(arr[0]));
				std::vector< dReal > arm_goal(6);
				bool arm_at_waypoint = true;

				if( _traj->GetConfigurationSpecification().ExtractJointValues(arm_goal.begin(),waypoint.begin(),_probot,arm_indices) )
				{
						arm_at_waypoint = MoveArmTowards(arm_goal);
				}
				//RAVELOG_ERROR("Test 13e");
				if (arm_at_waypoint)
				{
						_traj->Remove(0,1); // Remove the reached (first) waypoint. Now the next waypoint is the first waypoint.
				}
			}
			//RAVELOG_ERROR("Test 13f");
			ros::spinOnce();
		}

		virtual bool IsDone()
		{
			////RAVELOG_ERROR("Test 15");
			return _traj->GetNumWaypoints() == 0;
		}

		virtual OpenRAVE::dReal GetTime() const
		{
			//////RAVELOG_ERROR("Test 16");
			return 0;
		}

		virtual RobotBasePtr GetRobot() const
		{
			//////RAVELOG_ERROR("Test 17");
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
		TrajectoryBasePtr _traj;

		ros::Publisher _move_arm_pub;
		std::vector<double> _last_arm_command;

};

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream& sinput)
{
	return ControllerBasePtr(new Ur5Controller(penv,sinput));
}
