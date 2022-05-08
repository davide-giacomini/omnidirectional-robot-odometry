#include "omnidirectional_robot_odometry/compute_odometry.h"
#include <tf/transform_listener.h>

// TODO: I don't know why the method doesn't work inside the class with dynamic reconfigure. It works if declared in the .cpp file.
void choose_integration_method(int *integration_method, omnidirectional_robot_odometry::parametersConfig &config, uint32_t level);

ComputeOdometry::ComputeOdometry() {
    // Initialize subscriber and publisher
    this->sub_cmd_vel = this->nh.subscribe("cmd_vel", 1000, &ComputeOdometry::compute_odometry, this);
    this->pub_odometry = this->nh.advertise<nav_msgs::Odometry>("odom", 1000);

    // Get initial parameters and initialize pose
    this->nh.getParam("/init_pose_x", this->current_pose.x);
    this->nh.getParam("/init_pose_y", this->current_pose.y);
    this->nh.getParam("/init_pose_th", this->current_pose.th);

    // Start service for resetting odometry
    this->res_odom_service = this->nh.advertiseService("reset_odom", &ComputeOdometry::reset_odometry, this);

    // Start dynamic reconfigure service
    this->dynCallback = boost::bind(&choose_integration_method, &this->integration_method, _1, _2);
    this->dynServer.setCallback(this->dynCallback);
}

void ComputeOdometry::compute_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  this->stamp_ns[0] = this->stamp_ns[1];
  this->stamp_ns[1] = msg->header.stamp.nsec;

  if (this->stamp_ns[0] == -1) {
    return; // At least two values are needed to compute a velocity
  }


  // COMPUTE ODOMETRY

  // Velocities relative to base_link, hence to the robot frame.
  double vel_tau = msg->twist.linear.x;
  double vel_eta = msg->twist.linear.y;
  double vel_th = msg->twist.angular.z;

  // Velocities relative to odom, hence to the odometry frame.
  double vel_odom_x;
  double vel_odom_y;
  double vel_odom_th;

  if (this->integration_method == Integration::Euler) {
    vel_odom_x = vel_tau*cos(this->current_pose.th) - vel_eta*sin(this->current_pose.th);
    vel_odom_y = vel_tau*sin(this->current_pose.th) + vel_eta*cos(this->current_pose.th);
    vel_odom_th = vel_th;
  }
  else if (this->integration_method == Integration::RK) {
    double dt = (this->stamp_ns[1] - this->stamp_ns[0]) * pow(10.0, -9.0); // `dt` is in seconds.
    double direction = this->current_pose.th + ( vel_th * dt ) / 2.0;
    
    vel_odom_x = vel_tau*cos(direction) - vel_eta*sin(direction);
    vel_odom_y = vel_tau*sin(direction) + vel_eta*cos(direction);
    vel_odom_th = vel_th;
  }

  this->compute_pose(vel_odom_x, vel_odom_y, vel_odom_th);


  // PUBLISH ODOMETRY

  nav_msgs::Odometry odom_msg;
  // set header
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = msg->header.frame_id;
  // set position
  odom_msg.pose.pose.position.x = this->current_pose.x;
  odom_msg.pose.pose.position.y = this->current_pose.y;
  odom_msg.pose.pose.position.z = 0;
  // set theta
  tf2::Quaternion q;
  q.setRPY(0, 0, this->current_pose.th);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  // set covariance
  odom_msg.pose.covariance.fill(0);
  // set twist
  odom_msg.twist.twist.linear.x = vel_odom_x;
  odom_msg.twist.twist.linear.y = vel_odom_y;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = vel_th;
  // set covariance
  odom_msg.twist.covariance.fill(0);

  this->pub_odometry.publish(odom_msg);


  // BROADCAST TRANSFORM FROM ODOMETRY

  this->pub_transform(odom_msg);
}

pose ComputeOdometry::compute_pose(double vel_x, double vel_y, double vel_th){
  pose previous_pose;

  previous_pose.x = this->current_pose.x;
  previous_pose.y = this->current_pose.y;
  previous_pose.th = this->current_pose.th;

  double dt = (this->stamp_ns[1] - this->stamp_ns[0]) * pow(10.0, -9.0); // `dt` is in seconds.

  this->current_pose.x = previous_pose.x + dt*vel_x;
  this->current_pose.y = previous_pose.y + dt*vel_y;
  this->current_pose.th = previous_pose.th + dt*vel_th;

  return previous_pose;
}

void ComputeOdometry::pub_transform(const nav_msgs::Odometry msg){
  // set header
  this->trans_odom_baselink.header.stamp = msg.header.stamp;
  this->trans_odom_baselink.header.frame_id = msg.header.frame_id;
  this->trans_odom_baselink.child_frame_id = msg.child_frame_id;
  // set x,y
  this->trans_odom_baselink.transform.translation.x = msg.pose.pose.position.x;
  this->trans_odom_baselink.transform.translation.y = msg.pose.pose.position.y;
  this->trans_odom_baselink.transform.translation.z = msg.pose.pose.position.z;
  // set theta
  this->trans_odom_baselink.transform.rotation.x = msg.pose.pose.orientation.x;
  this->trans_odom_baselink.transform.rotation.y = msg.pose.pose.orientation.y;
  this->trans_odom_baselink.transform.rotation.z = msg.pose.pose.orientation.z;
  this->trans_odom_baselink.transform.rotation.w = msg.pose.pose.orientation.w;
  // send transform
  this->br.sendTransform(trans_odom_baselink);
}

bool ComputeOdometry::reset_odometry(omnidirectional_robot_odometry::ResetOdometry::Request  &req,
                                        omnidirectional_robot_odometry::ResetOdometry::Response &res) {
  // set header
  this->trans_world_odom.header.stamp = ros::Time::now();
  this->trans_world_odom.header.frame_id = "world";
  this->trans_world_odom.child_frame_id = "odom";
  // set x,y
  this->trans_world_odom.transform.translation.x = req.new_x;
  this->trans_world_odom.transform.translation.y = req.new_y;
  this->trans_world_odom.transform.translation.z = 0;
  // set theta
  tf2::Quaternion q;
  q.setRPY(0, 0, req.new_th);
  this->trans_world_odom.transform.rotation.x = q.x();
  this->trans_world_odom.transform.rotation.y = q.y();
  this->trans_world_odom.transform.rotation.z = q.z();
  this->trans_world_odom.transform.rotation.w = q.w();
  // send transform
  this->br.sendTransform(trans_world_odom);

  this->current_pose.x = 0;
  this->current_pose.y = 0;
  this->current_pose.th = 0;

  return true;
}

void /*ComputeOdometry::*/choose_integration_method(int *integration_method, omnidirectional_robot_odometry::parametersConfig &config, uint32_t level) {
    *integration_method = config.integration_method;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_odometry");

  ComputeOdometry co;
  
  ros::spin();

  return 0;
}