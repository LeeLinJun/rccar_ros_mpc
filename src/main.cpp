#include "Controller.hpp"

int main(int argc, char **argv)
{
		ros::init(argc, argv, "controller");
		ros::NodeHandle n;
		Controller controller(true);
		ros::Subscriber state = n.subscribe("/odom", 20, &Controller::observe, &controller);
		// ros::Subscriber path = n.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, &Controller::get_path, &controller);
		
		ros::Subscriber path = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 10, &Controller::get_path, &controller);
		//ros::Subscriber path = n.subscribe("/rrt_path", 20, &Controller::get_path, &controller);
		// ros::Subscriber goal = n.subscribe("/move_base_simple/goal", 1, &Controller::get_goal, &controller);
		ros::Publisher control = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
		ros::Rate loop_rate(20);
		while (ros::ok())
		{       
				control.publish(controller.control());
				ros::spinOnce();
				loop_rate.sleep();
	
		}
	ros::spin();

	return 0;
}
