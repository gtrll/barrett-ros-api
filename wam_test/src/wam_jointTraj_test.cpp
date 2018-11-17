
#include <ros/ros.h>
#include "wam_common/JointTraj.h"
#include "sensor_msgs/JointState.h"

std::vector<std::vector<double> > linspace(std::vector<double> pointStart, std::vector<double> pointGoal, const double& T, const double& dt) {

    std::vector<std::vector<double> > trajectory;

    size_t num= (size_t) (T / dt);
    for (size_t i=0; i < num; i++) {
    	std::vector<double> p_t;
    	double t = dt * i;
    	p_t.push_back(t);

    	for (size_t j=0; j < 7; j++) {
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
    p_goal.push_back(pointGoal[3]);
    p_goal.push_back(pointGoal[4]);
    p_goal.push_back(pointGoal[5]);
    p_goal.push_back(pointGoal[6]);
    trajectory.push_back(p_goal);

    return trajectory;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wam_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<wam_common::JointTraj>("/wam/joint_traj_cmd",10);

	ros::Duration(1.0).sleep();

	double dt = 0.5; // time interval (s)
	// double dt = 0.02; // time interval (s)
	double T = 3.0; //trajectory duration (s)

	// Get current cartes. pose
	boost::shared_ptr<sensor_msgs::JointState const> jpPtr;
	jpPtr = ros::topic::waitForMessage <sensor_msgs::JointState> ("/wam/joint_states",ros::Duration(10));

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
	pointStart.push_back(jpPtr->position[0]);
	pointStart.push_back(jpPtr->position[1]);
	pointStart.push_back(jpPtr->position[2]);
	pointStart.push_back(jpPtr->position[3]);
	pointStart.push_back(jpPtr->position[4]);
	pointStart.push_back(jpPtr->position[5]);
	pointStart.push_back(jpPtr->position[6]);

	std::vector<double> pointGoal;
	pointGoal.push_back(-0.39598);
	pointGoal.push_back(-1.72859);
	pointGoal.push_back(0.793012);
	pointGoal.push_back(1.830379);
	pointGoal.push_back(-0.73093);
	pointGoal.push_back(-0.78612);
	pointGoal.push_back(0.040789);

	std::vector<std::vector<double> > pointTraj = linspace(pointStart,pointGoal,T,dt);

	wam_common::JointTraj jpTraj;
	for (size_t i=0; i < pointTraj.size(); i++){
		jpTraj.time.push_back(pointTraj[i][0]);
		jpTraj.j1.push_back(pointTraj[i][1]);
		jpTraj.j2.push_back(pointTraj[i][2]);
		jpTraj.j3.push_back(pointTraj[i][3]);
		jpTraj.j4.push_back(pointTraj[i][4]);
		jpTraj.j5.push_back(pointTraj[i][5]);
		jpTraj.j6.push_back(pointTraj[i][6]);
		jpTraj.j7.push_back(pointTraj[i][7]);

		std::cout << i << "," << 
		pointTraj[i][0] << "," << 
		pointTraj[i][1] << "," << 
		pointTraj[i][2] << "," << 
		pointTraj[i][3] << "," << 
		pointTraj[i][4] << "," << 
		pointTraj[i][5] << "," << 
		pointTraj[i][6] << "," << 
		pointTraj[i][7] << "," << std::endl;
	}

	pub.publish(jpTraj);

	ros::Duration(1.0).sleep();

	ros::spinOnce();

}