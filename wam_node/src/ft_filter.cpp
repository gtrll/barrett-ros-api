#include "ft_filter.h"
	
ForceTorqueFilter::ForceTorqueFilter (uint win_size) : win_size_(win_size){

	nh_ = ros::NodeHandle("~");

	nh_.getParam("ft_topic", ft_topic_);
	nh_.getParam("filter_topic", filter_topic_);

	ft_sub_ = nh_.subscribe(ft_topic_, 1, &ForceTorqueFilter::listenerCB, this);
	// wait for queue to populate
	ros::Duration(1.0).sleep();

	nh_.getParam("frame_id", frame_id_);

	ft_pub_ = nh_.advertise<geometry_msgs::WrenchStamped> (filter_topic_,1);

	init_=true;
	
}

void ForceTorqueFilter::listenerCB (const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	// collect wrench samples in FIFO queue
	msg_buff_.push_back(msg->wrench);
	if (msg_buff_.size() > win_size_) msg_buff_.pop_front();
}

void ForceTorqueFilter::publish() {

	geometry_msgs::WrenchStamped msg_out;

	// Get initial offset
	if (init_) {
		w_offset_ = getMean_();
		init_=false;
	}

	// Subtract offset from mean
	msg_out.wrench = subWrench_(getMean_(), w_offset_);

	msg_out.header.frame_id = frame_id_;
	msg_out.header.stamp = ros::Time::now();

	ft_pub_.publish(msg_out);
}


geometry_msgs::Wrench ForceTorqueFilter::getMean_() {

	/* TODO: add mutex (pre-C++11) */

	geometry_msgs::Wrench w_mean;

	for (size_t i=0; i<msg_buff_.size(); i++) {
		w_mean.force.x += msg_buff_.at(i).force.x;
		w_mean.force.y += msg_buff_.at(i).force.y;
		w_mean.force.z += msg_buff_.at(i).force.z;
		w_mean.torque.x += msg_buff_.at(i).torque.x;
		w_mean.torque.y += msg_buff_.at(i).torque.y;
		w_mean.torque.z += msg_buff_.at(i).torque.z;
	}	


	w_mean.force.x  = w_mean.force.x / msg_buff_.size();
	w_mean.force.y  = w_mean.force.y / msg_buff_.size();
	w_mean.force.z  = w_mean.force.z / msg_buff_.size();
	w_mean.torque.x = w_mean.torque.x / msg_buff_.size();
	w_mean.torque.y = w_mean.torque.y / msg_buff_.size();
	w_mean.torque.z = w_mean.torque.z / msg_buff_.size();

	return w_mean;
}

// Subtract wrenches
geometry_msgs::Wrench ForceTorqueFilter::subWrench_ (const geometry_msgs::Wrench& w1, const geometry_msgs::Wrench& w2) {

	geometry_msgs::Wrench w_out;

	w_out.force.x  = w1.force.x  - w2.force.x ;
	w_out.force.y  = w1.force.y  - w2.force.y ;
	w_out.force.z  = w1.force.z  - w2.force.z ;
	w_out.torque.x = w1.torque.x - w2.torque.x ;
	w_out.torque.y = w1.torque.y - w2.torque.y ;
	w_out.torque.z = w1.torque.z - w2.torque.z ;

	return w_out;
}



///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

	ros::init(argc, argv, "ft_filter");

	ForceTorqueFilter ft_filter(50);
	
	ros::Rate pub_rate(250);

	while (ros::ok()) {
		ros::spinOnce();
		ft_filter.publish();
		pub_rate.sleep();
	}
}