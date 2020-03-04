// for math
#include <cmath>

// for ros
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

// for MPC
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <vector>

// for readcsv
// #include "utils.h"

class Controller
{
public:
	void observe(const nav_msgs::Odometry::ConstPtr& msg);
	void get_path(const nav_msgs::Path::ConstPtr& msg);
	void get_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);
	ackermann_msgs::AckermannDriveStamped control();
	bool verbose = false;
	// vector<double> path_x = {};
	// vector<double> path_y = {};
	std::vector<double> path_x = vector<double>(32);
	std::vector<double> path_y = vector<double>(32)	;
	std::vector<double> path_goal = vector<double>(2);
	Controller(){};
    Controller(bool verbose){
		this -> verbose = verbose;
	}
    

private:
	MPC mpc;
	double x, 
	y, 
	th, 
   	vel, 
   	vth,
   	a = 0, 
   	sta=0;
	int curr = 0;
	

};

