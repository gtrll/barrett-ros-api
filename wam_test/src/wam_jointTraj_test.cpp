
#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
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
	ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/wam/joint_traj_cmd",10);

	ros::Duration(1.0).sleep();

	size_t DOF = 7;
	double dt = 0.5; // time interval (s)
	double T = 5.0; //trajectory duration (s)

	// Get current joint position
	boost::shared_ptr<sensor_msgs::JointState const> jpPtr;
	jpPtr = ros::topic::waitForMessage <sensor_msgs::JointState> ("/wam/joint_states",ros::Duration(10));

	// Set start to current
	std::vector<double> pointStart;
	for (size_t j=0; j<DOF; j++) 
		pointStart.push_back(jpPtr->position[j]);
	

	// Set goal joint pos
	std::vector<double> pointGoal;
	pointGoal.push_back(-0.39598);
	pointGoal.push_back(-1.72859);
	pointGoal.push_back(0.793012);
	pointGoal.push_back(1.830379);
	pointGoal.push_back(-0.73093);
	pointGoal.push_back(-0.78612);
	pointGoal.push_back(0.040789);

	// Get linear sequence of waypoints
	std::vector<std::vector<double> > pointTraj = linspace(pointStart,pointGoal,T,dt);

	// Populate msg
	trajectory_msgs::JointTrajectory jpTraj;
	jpTraj.points.resize(pointTraj.size());

	for (size_t i=0; i < pointTraj.size(); i++){
		jpTraj.points[i].positions.resize(DOF);
		jpTraj.points[i].time_from_start = ros::Duration(pointTraj[i][0]);
		for (size_t j=0; j<DOF; j++)
			jpTraj.points[i].positions[j]=pointTraj[i][j+1]; // time is first index
		// std::cout << jpTraj.points[i].time_from_start.toSec() << ", " <<
		// jpTraj.points[i].positions[0] << ", " <<
		// jpTraj.points[i].positions[1] << ", " <<
		// jpTraj.points[i].positions[2] << ", " <<
		// jpTraj.points[i].positions[3] << ", " <<
		// jpTraj.points[i].positions[4] << ", " <<
		// jpTraj.points[i].positions[5] << ", " <<
		// jpTraj.points[i].positions[6] << std::endl;
	}

	pub.publish(jpTraj);

	ros::Duration(1.0).sleep();

	ros::spinOnce();

}