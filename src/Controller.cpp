#include "Controller.hpp"


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

}


void Controller::get_path(const nav_msgs::Path::ConstPtr& msg)
{
	std::vector<geometry_msgs::PoseStamped> poses = msg->poses;
	// std::cout<<poses.size()<<std::endl;
	int k = 2;
	int length = poses.size()>(std::size_t)(N*k) ? N: poses.size(), start = 0;
	
	double min = 1e10;

	double xi = 0., yi = 0., tmp = 0.;
	for(unsigned int i = 0; i< poses.size(); i++){
		xi = poses.at(i).pose.position.x;
		yi = poses.at(i).pose.position.y;
		tmp = (x-xi)*(x-xi) + (y-yi) * (y-yi);
		if(tmp < min){
			start = i;
			min = tmp;
		}
		if((std::size_t)(start+length*k) > poses.size()){
				length = (poses.size() - start) / k;
		}

		// if(start > curr){
		// 	curr = start;
		// }
		curr = start;
		path_goal.at(0) = poses.back().pose.position.x;
		path_goal.at(1) = poses.back().pose.position.y;
	}

	
	
	path_x = std::vector<double>(length);
	path_y = std::vector<double>(length); 
	
	for (int i = 0; i < length; i++)
	{
		path_x.at(i) = poses.at(i*k+curr).pose.position.x;
		path_y.at(i) = poses.at(i*k+curr).pose.position.y;
	}
}

ackermann_msgs::AckermannDriveStamped Controller::control(){
		// set up msg
		ackermann_msgs::AckermannDriveStamped _ackermann_msg = ackermann_msgs::AckermannDriveStamped();
		_ackermann_msg.header.frame_id = "base_link";
		_ackermann_msg.header.stamp = ros::Time::now();

		// vector<double> ptsx(path_x.begin()+curr, path_x.begin() + std::min((int)path_x.size()-1, curr+6));
		// vector<double> ptsy(path_y.begin()+curr, path_y.begin() + std::min((int)path_y.size()-1, curr+6));
		
		// deal with path
		if (pow( x - path_goal.at(0), 2)+ pow( y - path_goal.at(1), 2) < 0.1*0.1 || path_x.size()<1){
			// reached
			// curr = 0;
			_ackermann_msg.drive.steering_angle = 0;
			_ackermann_msg.drive.speed = 0;
			_ackermann_msg.drive.acceleration = 0;//throttle_value;
		}

		
		else{	

			std::vector<double> ptsx(N, path_x.back());
			std::vector<double> ptsy(N, path_y.back());
			// vector<double> ptsy = std::vector<double>(N);
			int length = ((path_x.size()-1) < (N-1)) ? (path_x.size()-1) : (N-1);
			for( int i = 0; i < length ; i++){
				ptsx.at(i) = path_x.at(i);
				ptsy.at(i) = path_y.at(i);
			}
			// solve MPC		
			double px = x;
			double py = y;
			double psi = th;
			double v = vel;

			double str = sta;
			double throttle = a;
			for(unsigned int i = 0; i < ptsx.size(); i++){
					double diffx = ptsx[i]-px;
					double diffy = ptsy[i]-py;
					ptsx[i] = diffx * cos(psi) + diffy * sin(psi);
					ptsy[i] = diffy * cos(psi) - diffx * sin(psi);
			}
			// int horizon = ptsx.size();
			Eigen::VectorXd ptsxV = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
			Eigen::VectorXd ptsyV = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
			
			Eigen::VectorXd state(4);
			
			double px_l = v * dt;
			double py_l = 0.0;
			double psi_l = v * str / Lf * dt;
			double v_l = v + throttle*dt;
			
			state << px_l, py_l, psi_l, v_l; //cte_l, epsi_l;
			std::vector<double> r;
			r = mpc.Solve(state, ptsxV, ptsyV);

			double steer_value = r[0]; /// (deg2rad(25)*Lf);
			double throttle_value = r[1]; //r[1]*(1-fabs(steer_value))+0.1;
			double velocity_value = vel + throttle_value * dt;  
			if(verbose){
				// for (unsigned int i = 0; i < ptsx.size(); i++){
				// 	ROS_INFO("[%f],[%f]\n", ptsx.at(i),ptsy.at(i));
				// }
				ROS_INFO("sta: [%f], v:[%f], a:[%f]", steer_value, velocity_value, throttle_value);
				ROS_INFO("x: [%f], y:[%f], th:[%f]", x, y, th);
			}
			

			
			_ackermann_msg.drive.steering_angle = steer_value;
			_ackermann_msg.drive.speed = velocity_value;
			_ackermann_msg.drive.acceleration = throttle_value;//throttle_value;
			
		}
		

		
		return _ackermann_msg;
}



