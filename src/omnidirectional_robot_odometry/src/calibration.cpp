#include <omnidirectional_robot_odometry/calibration.h>

Calibration::Calibration() {
    this->sub_odom = new message_filters::Subscriber<nav_msgs::Odometry>(this->nh, "odom", 1);
    this->sub_pose_gt = new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh, "robot/pose", 1);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), *(this->sub_odom), *(this->sub_pose_gt));
    sync.registerCallback(&Calibration::callback, this);
}

void Calibration::callback(const nav_msgs::Odometry::ConstPtr& msg_odom, const geometry_msgs::PoseStamped::ConstPtr& msg_pose_gt)
{

    ROS_INFO("%s", msg_odom);

    ROS_INFO("%s", msg_pose_gt);

}

int main(int argc, char**argv){
	ros::init(argc, argv, "calibration");

    ROS_INFO("Sono qui");

	ros::spin();

	return 0;
}