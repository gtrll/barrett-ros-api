#include <ros/ros.h>

class ForceTorqueFilter {

	public:

		~ForceTorqueFilter() {};

	private:

		ros::NodeHandle nh_;
		std::string ft_topic_;
		ros::Subscriber ft_sub_;

	public: 

		ForceTorqueFilter () {

			nh_.param<std::string>("ft_topic", ft_topic_, "/ft_sensor/raw");
			ft_sub_ = nh_.subscribe(ft_topic, 100, filterCallback, this)

		}

};
	

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

	ros::init(argc, argv, "ft_filter");

	ForceTorqueFilter ft_filter;
	
	ros::Rate pub_rate(250);

	while (ros::ok()) {
		ros::spinOnce();
		pub_rate.sleep();
	}
}