
#include <ros/ros.h>
#include "wam_common/CartPointTraj.h"
#include "geometry_msgs/PoseStamped.h"

std::vector<std::vector<double> > linspace(std::vector<double> pointStart, std::vector<double> pointGoal, const double& T, const double& dt) {

    std::vector<std::vector<double> > trajectory;

    size_t num= (size_t) (T / dt);
    for (size_t i=0; i < num; i++) {
    	std::vector<double> p_t;
    	double t = dt * i;
    	p_t.push_back(t);

    	for (size_t j=0; j < 3; j++) {
    		float d = pointStart[j] + (t / T) * (pointGoal[j]-pointStart[j]); 
    		p_t.push_back(d);
    	}
    	trajectory.push_back(p_t);
    }
    std::vector<double> p_goal;
    p_goal.push_back(T);
    p_goal.push_back(pointGoal[0]);
    p_goal.push_back(pointGoal[1]);
    p_goal.push_back(pointGoal[2]);
    trajectory.push_back(p_goal);

    return trajectory;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wam_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wam_common::CartPointTraj>("/wam/cart_traj_cmd",10);

	ros::Duration(1.0).sleep();

	double dt = 0.02; // time interval (s)
	double T = 5.0; //trajectory duration (s)


	// Get current cartes. pose
	boost::shared_ptr<geometry_msgs::PoseStamped const> cposePtr;
	cposePtr = ros::topic::waitForMessage <geometry_msgs::PoseStamped> ("/wam/pose",ros::Duration(10));

	// std::cout << "Pose Message received." << std::endl;
	// std::cout << "Position: " << std::endl;
	// std::cout << "x : " << cposePtr->pose.position.x << std::endl;
	// std::cout << "y : " << cposePtr->pose.position.y << std::endl;
	// std::cout << "z : " << cposePtr->pose.position.z << std::endl;
	// std::cout << "Orientation: " << std::endl;
	// std::cout << "x : " << cposePtr->pose.orientation.x << std::endl;
	// std::cout << "y : " << cposePtr->pose.orientation.y << std::endl;
	// std::cout << "z : " << cposePtr->pose.orientation.z << std::endl;
	// std::cout << "w : " << cposePtr->pose.orientation.w << std::endl;

	std::vector<double> pointStart;
	pointStart.push_back(cposePtr->pose.position.x);
	pointStart.push_back(cposePtr->pose.position.y);
	pointStart.push_back(cposePtr->pose.position.z);

	std::vector<double> pointGoal;
	pointGoal.push_back(cposePtr->pose.position.x);
	pointGoal.push_back(cposePtr->pose.position.y+0.5);
	pointGoal.push_back(cposePtr->pose.position.z);

	std::vector<std::vector<double> > pointTraj = linspace(pointStart,pointGoal,T,dt)

	wam_common::CartPointTraj cpTraj;
	for (size_t i=0; i < pointTraj.size(); i++){
		cpTraj.time.push_back(pointTraj[i][0]);
		cpTraj.x.push_back(pointTraj[i][1]);
		cpTraj.y.push_back(pointTraj[i][2]);
		cpTraj.z.push_back(pointTraj[i][3]);

		std::cout << i << "," << 
		pointTraj[i][0] << "," << 
		pointTraj[i][1] << "," << 
		pointTraj[i][2] << "," << 
		pointTraj[i][3] << "," << std::endl;
	}

	pub.publish(cpTraj);

	ros::Duration(1.0).sleep();

	ros::spinOnce();
}