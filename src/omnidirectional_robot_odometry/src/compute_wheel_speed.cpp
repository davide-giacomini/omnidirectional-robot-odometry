#include <omnidirectional_robot_odometry/compute_wheel_speed.h>
#include <omnidirectional_robot_odometry/util.h>

ComputeControl::ComputeControl() {
    this->subscriber_cmd_vel = this->nh.subscribe("cmd_vel", 1000, &ComputeControl::sub_callback, this);
    this->publish_wheel_speed = this->nh.advertise<omnidirectional_robot_odometry::CustomRpm>("wheels_rpm", 1000);
}

void ComputeControl::sub_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    wheels_speed my_wheels_speed = compute_wheels_speed(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);

    omnidirectional_robot_odometry::CustomRpm cust_msg;

    cust_msg.rpm_fl = my_wheels_speed.rpm_fl;
    cust_msg.rpm_fr = my_wheels_speed.rpm_fr;
    cust_msg.rpm_rl = my_wheels_speed.rpm_rl;
    cust_msg.rpm_rr = my_wheels_speed.rpm_rr;

    this->publish_wheel_speed.publish(cust_msg);
}

wheels_speed ComputeControl::compute_wheels_speed(double vel_x, double vel_y, double vel_th) {
    wheels_speed my_wheels_speed;
    // Useless, but code more readable
    double r = Util::r;
    double l = Util::l;
    double w = Util::w;

    my_wheels_speed.rpm_fl = (1/r) * ( vel_x - vel_y - vel_th*(l+w) );
    my_wheels_speed.rpm_fr = (1/r) * ( vel_x + vel_y + vel_th*(l+w) );
    my_wheels_speed.rpm_rl = (1/r) * ( vel_x + vel_y - vel_th*(l+w) );
    my_wheels_speed.rpm_rr = (1/r) * ( vel_x - vel_y + vel_th*(l+w) );

    return my_wheels_speed;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_wheel_speed");

  ComputeControl cp;

  ros::spin();

  return 0;
}