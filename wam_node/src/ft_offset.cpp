#include "ft_filter.h"


// Subtract offset only
class ForceTorqueOffset : public ForceTorqueFilter {

	public:

		ForceTorqueOffset(uint win_size);
		~ForceTorqueOffset() {};

	private:
		geometry_msgs::Wrench w_;

	public: 

		void listenerCB (const geometry_msgs::WrenchStamped::ConstPtr& msg);
		void publish();
};
	

ForceTorqueOffset::ForceTorqueOffset (uint win_size) : ForceTorqueFilter(win_size) {}


void ForceTorqueOffset::listenerCB (const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	// collect wrench samples in FIFO queue
	if (init_) {
		msg_buff_.push_back(msg->wrench);
		if (msg_buff_.size() > win_size_) msg_buff_.pop_front();
	}

	w_ = msg->wrench;
}

void ForceTorqueOffset::publish() {

	geometry_msgs::WrenchStamped msg_out;

	// Get initial offset
	if (init_) {
		w_offset_ = getMean_();
		init_=false;
	}

	// Subtract offset from mean
	msg_out.wrench = subWrench_(w_, w_offset_);

	msg_out.header.frame_id = frame_id_;
	msg_out.header.stamp = ros::Time::now();

	ft_pub_.publish(msg_out);
}



///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

	ros::init(argc, argv, "ft_offset");

	ForceTorqueOffset ft_offset(50);
	
	ros::Rate pub_rate(250);

	while (ros::ok()) {
		ros::spinOnce();
		ft_offset.publish();
		pub_rate.sleep();
	}
}