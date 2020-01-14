// for math
#include <cmath>
// for ros
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <ros/package.h>


// for MPC
#include "MPC.h"
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>

// for readcsv
#include "utils.h"


class Controller
{
		public:
				void observe(const nav_msgs::Odometry::ConstPtr& msg);
				ackermann_msgs::AckermannDriveStamped control();
				bool verbose = false;
				// vector<double> path_x = {};
				// vector<double> path_y = {};
				vector<double> path_x = {};
				vector<double> path_y = {};
				vector<double> path_goal = {};
				Controller(){
					if(true){
						read_csv_path(ros::package::getPath("rccar_ros_mpc")+"/src/path.csv", path_x, path_y, path_goal);
					}
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
				Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);
				double polyeval(Eigen::VectorXd coeffs, double x);
				Eigen::VectorXd coeffs;

};



double Controller::polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (auto i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

Eigen::VectorXd Controller::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
												int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (auto i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (auto j = 0; j < xvals.size(); j++) {
		for (auto i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

void Controller::observe(const nav_msgs::Odometry::ConstPtr& msg)
{
		// position
		x = msg->pose.pose.position.x;
		y = msg->pose.pose.position.y;
		
		// pose
		tf::Quaternion q(msg->pose.pose.orientation.x, 
										 msg->pose.pose.orientation.y, 
										 msg->pose.pose.orientation.z, 
										 msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		th = yaw;

		// velocity
		vel = msg->twist.twist.linear.x;
		vth = msg->twist.twist.angular.z;

		// if (verbose){
		// 		ROS_INFO("Seq: [%d]", msg->header.seq);
		// 		ROS_INFO("Position-> x: [%f], y: [%f]", x, y);
		// 		ROS_INFO("Orientation-> theta: [%f]", th);
		// 		ROS_INFO("Vel-> Linear: [%f], Angular: [%f]\n", vel, vth);
		// }
		
}

ackermann_msgs::AckermannDriveStamped Controller::control(){
		// set up msg
		ackermann_msgs::AckermannDriveStamped _ackermann_msg = ackermann_msgs::AckermannDriveStamped();
		_ackermann_msg.header.frame_id = "base_link";
		_ackermann_msg.header.stamp = ros::Time::now();
		vector<double> ptsx(path_x.begin()+curr, path_x.begin() + std::min((int)path_x.size()-1, curr+6));
		vector<double> ptsy(path_y.begin()+curr, path_y.begin() + std::min((int)path_y.size()-1, curr+6));
		// deal with path
		if (/*curr == path_x.size()-1  ||*/ pow( x - path_goal.at(0), 2)+ pow( y - path_goal.at(1), 2) < 0.3){
			// reached
			_ackermann_msg.drive.steering_angle = 0;
			_ackermann_msg.drive.speed = 0;
			_ackermann_msg.drive.acceleration = 0;//throttle_value;
		}
		else{	
			if(verbose){
				for (auto i=0; i<ptsx.size(); i++){
				ROS_INFO("[%f],[%f]\n", ptsx.at(i),ptsy.at(i));
				}
			}
			
			
			// if (pow( x - ptsx.front(),2) + pow( y - ptsy.front(), 2) < 0.3){
			if (cos(th) * (ptsx.front() - x) + sin(th) * (ptsy.front()-y) < 0 && curr < path_x.size()-4 ){
				curr ++;
			}
			else{
				// solve MPC		
				double px = x;
				double py = y;
				double psi = th;
				double v = vel;

				double str = sta;
				double throttle = a;
				double xgoal = ptsx.at(0);
				double ygoal = ptsy.at(0);
				vector<double> goal = {(xgoal-px)*cos(psi)+(ygoal-py)*sin(psi), (ygoal-py)*cos(psi)-(xgoal-px)*sin(psi) };
				for(auto i=0;i<ptsx.size();i++){
						double diffx = ptsx[i]-px;
						double diffy = ptsy[i]-py;
						ptsx[i] = diffx * cos(psi) + diffy * sin(psi);
						ptsy[i] = diffy * cos(psi) - diffx * sin(psi);
				}

				if(ptsx.size()>=4){
					Eigen::VectorXd ptsxV = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
					Eigen::VectorXd ptsyV = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
					coeffs = polyfit(ptsxV, ptsyV, 3);
				}
				Eigen::VectorXd goalV = Eigen::VectorXd::Map(goal.data(), goal.size());
				// std::cout << coeffs << std::endl;
				Eigen::VectorXd state(6);
				
				double px_l = v * dt;
				double py_l = 0.0;
				double psi_l = v * str / Lf * dt;
				double v_l = v + throttle*dt;
				double cte_l =  polyeval(coeffs, 0) + v * CppAD::sin(atan(coeffs[1])) * dt;
				double epsi_l = -atan(coeffs[1])+psi_l;
				state << px_l, py_l, psi_l, v_l, cte_l, epsi_l;
				std::vector<double> r;
				r = mpc.Solve(state, coeffs, goalV);

				double steer_value = r[0]; /// (deg2rad(25)*Lf);
				double throttle_value = r[1]; //r[1]*(1-fabs(steer_value))+0.1;
				double velocity_value = vel + throttle_value * dt;  
				// vector<double> mpc_x_vals;
				// vector<double> mpc_y_vals;

				// for ( int i = 2; i < r.size(); i++ ) {
				// 		if ( i % 2 == 0 ) {
				// 				mpc_x_vals.push_back( r[i] ); // for x
				// 				// std::cout<<"x"<<r[i]<<",";
				// 		} else {
				// 				mpc_y_vals.push_back( r[i] );// for y
				// 				// std::cout<<"y"<<r[i]<<std::endl;
				// 		}
				// }
				ROS_INFO("sta: [%f], v:[%f], a:[%f]", steer_value, velocity_value, throttle_value);
				ROS_INFO("x: [%f], y:[%f], th:[%f]", x, y, th);

				
				_ackermann_msg.drive.steering_angle = steer_value;
				_ackermann_msg.drive.speed = velocity_value;
				_ackermann_msg.drive.acceleration = throttle_value;//throttle_value;
			}
		}
		

		
		return _ackermann_msg;
}


int main(int argc, char **argv)
{
		ros::init(argc, argv, "odom_listener");
		ros::NodeHandle n;
		Controller controller;
		ros::Subscriber state = n.subscribe("/odom", 1, &Controller::observe, &controller);
		ros::Publisher control = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
		ros::Rate loop_rate(10);

		while (ros::ok())
		{       
				
				control.publish(controller.control());

				ros::spinOnce();

				loop_rate.sleep();
	
		}
	//ros::spin();

	return 0;
}