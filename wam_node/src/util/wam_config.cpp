#include "wam_config.h"

#include "assert.h"

WamConfig::WamConfig (ros::NodeHandle nh, size_t DOF) {

	ROS_INFO("Loading WAM config parameters.");

	if (nh.hasParam("planning/DOF")) {
		nh.getParam("planning/DOF", plan_DOF);
	} else {
		ROS_ERROR("You must specify planning DOFs.");
	}

	if (nh.hasParam("joint_controller/position")) {
		nh.getParam("joint_controller/position/Kp",	Jp_Kp);
		nh.getParam("joint_controller/position/Kd",	Jp_Kd);
		nh.getParam("joint_controller/position/Ki",	Jp_Ki);
		nh.getParam("joint_controller/position/i_lim",	Jp_uLim);
		nh.getParam("joint_controller/position/u_lim",	Jp_iLim);
		assert(Jp_Kd.size()==DOF);
		assert(Jp_Kd.size()==DOF);
		assert(Jp_Ki.size()==DOF);
		assert(Jp_uLim.size()==DOF);
		assert(Jp_iLim.size()==DOF);
		use_default_JpGains=false;		
	} else {
		use_default_JpGains=true;
		ROS_WARN("No joint-position gains specified. Using default values");
	}
	if (nh.hasParam("joint_controller/velocity")) {
		nh.getParam("joint_controller/velocity/Kp",	Jv_Kp);
		nh.getParam("joint_controller/velocity/Kd",	Jv_Kd);
		nh.getParam("joint_controller/velocity/Ki",	Jv_Ki);
		nh.getParam("joint_controller/velocity/i_lim",	Jv_uLim);
		nh.getParam("joint_controller/velocity/u_lim",	Jv_iLim);
		assert(Jv_Kd.size()==DOF);
		assert(Jv_Kd.size()==DOF);
		assert(Jv_Ki.size()==DOF);
		assert(Jv_uLim.size()==DOF);
		assert(Jv_iLim.size()==DOF);
		use_default_JpGains=false;	
	} else {
		use_default_JvGains=true;
		ROS_WARN("No joint-velocity gains specified. Using default values");
	}
}

