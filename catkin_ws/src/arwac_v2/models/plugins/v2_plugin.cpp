#include <stdio.h>
#include <math.h>
#include <string>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
using namespace std;

//Podcar Gazebo-to-ROS plugin.   Simulates sensors and recieves wheel commands.
//Build with:  cmake . ; make    (NB not catkin -- just including it to get ROS libraries in)

namespace gazebo
{
  class MyPlugin : public ModelPlugin
  {
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			int argc=0;
			char** argv;		
			ros::init(argc, argv, "GazeboPlugin");

			this->model = _parent;   //*er to my physical model, that I'm controlling
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyPlugin::OnUpdate, this, _1));      //GAZEBO (not ROS) callback
			this->cmdSub  = _n.subscribe("cmd_vel",  1, &MyPlugin::wheelSpeedCallback,  (MyPlugin*)this);  
			this->_pub_gnd  = _n.advertise<nav_msgs::Odometry>("odometry/groundTruth", 1000);   //TODO more acurate to sim the IMU and GPS ten fuse in EKF later on
			
			this-> timeOut = 2;
			this-> lastCmdTime = time(NULL);
			this-> timedOut = false;

			this->fR = this->model->GetJoint("wheel_front_right_joint");
			this->fL = this->model->GetJoint("wheel_front_left_joint");
			this->bR = this->model->GetJoint("wheel_back_right_joint");
			this->bL = this->model->GetJoint("wheel_back_left_joint");

			this->angleController = common::PID(2, 0, 1);

			this->model->GetJointController()->SetPositionPID(fL->GetScopedName(), this->angleController);
			this->model->GetJointController()->SetPositionPID(fR->GetScopedName(), this->angleController);
		}

		public: void wheelSpeedCallback(const std_msgs::Float64& msg){
			this-> timedOut = false;
			this->lastCmdTime = time(NULL);
			
			bL->SetVelocity(0, msg.data/0.203015); //divide by wheel radius
			bR->SetVelocity(0, msg.data/0.203015);
			fL->SetVelocity(0, msg.data/0.203015);
			fR->SetVelocity(0, msg.data/0.203015);
		}

		public: void brake(){
			bL->SetVelocity(0, 0);   
			bR->SetVelocity(0, 0);  
			fL->SetVelocity(0, 0);   
			fR->SetVelocity(0, 0);  
		}

		public: void OnUpdate(const common::UpdateInfo & /*_info*/){
			//cout << "update " << endl;

			std::time_t timeNow = time(NULL);
			if(timeNow >= this->lastCmdTime+this->timeOut && !this->timedOut){
				this-> timedOut = true;
				this->brake();
			}

			math::Pose pose = this->model->GetWorldPose();
			nav_msgs::Odometry odomOut;
			odomOut.header.stamp = ros::Time::now(); 
			odomOut.header.frame_id = "map";
            odomOut.pose = pose;
			this->_pub_gnd.publish(odomOut);
		}

		public: ros::NodeHandle _n;
		protected: ros::Subscriber cmdSub;
		protected: ros::Publisher _pub_gnd;
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		
		private: std::time_t lastCmdTime;
		private: std::time_t timeOut;
		private: bool timedOut;

		public: common::PID angleController;

		private: physics::JointPtr fR;
		private: physics::JointPtr fL;
		private: physics::JointPtr bR;
		private: physics::JointPtr bL;
	};

	GZ_REGISTER_MODEL_PLUGIN(MyPlugin)   //tell Gazebo I'm here
}
