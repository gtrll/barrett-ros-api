#pragma once

#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

class WamConfig
{

public:
	size_t plan_DOF;
	std::vector<double> Jp_Kp, Jp_Kd, Jp_Ki, Jp_uLim, Jp_iLim;
	std::vector<double> Jv_Kp, Jv_Kd, Jv_Ki, Jv_uLim, Jv_iLim;
	bool use_default_JpGains;
	bool use_default_JvGains;

private: 

public:
	// Default Constructor
	WamConfig() {};

	// Load params from config file
	WamConfig(ros::NodeHandle nh, size_t DOF);
};

