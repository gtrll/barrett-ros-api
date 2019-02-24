#pragma once

#include <deque>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class ForceTorqueFilter {

	public:

		ForceTorqueFilter(uint win_size);
		~ForceTorqueFilter() {};

	protected:

		ros::NodeHandle nh_;
		std::string ft_topic_;
		std::string filter_topic_;
		std::string frame_id_;
		ros::Subscriber ft_sub_;
		ros::Publisher ft_pub_;

		bool init_;

		uint win_size_;
		std::deque <geometry_msgs::Wrench> msg_buff_;

		geometry_msgs::Wrench w_offset_;
		geometry_msgs::Wrench getMean_();
		geometry_msgs::Wrench subWrench_(const geometry_msgs::Wrench& w1, const geometry_msgs::Wrench& w2);

	public: 

		virtual void listenerCB (const geometry_msgs::WrenchStamped::ConstPtr& msg);
		virtual void publish();
};