#include "omnidirectional_robot_odometry/compute_odometry.h"

// TODO: I don't know why the method doesn't work inside the class with dynamic reconfigure
void choose_integration_method(int *integration_method, omnidirectional_robot_odometry::parametersConfig &config, uint32_t level);

ComputeOdometry::ComputeOdometry() {
    // Initialize subscriber and publisher
    this->sub_cmd_vel = this->nh.subscribe("cmd_vel", 1000, &ComputeOdometry::compute_odometry, this);
    this->pub_odometry = this->nh.advertise<nav_msgs::Odometry>("odom", 1000);

    // Get initial parameters
    this->nh.getParam("/init_pose_x", this->init_pose_x);
    this->nh.getParam("/init_pose_y", this->init_pose_y);
    this->nh.getParam("/init_pose_th", this->init_pose_th);

    // Start service for resetting odometry
    this->res_odom_service = this->nh.advertiseService("reset_odom", &ComputeOdometry::reset_odometry, this);

    // Start dynamic reconfigure service
    //TODO -> this doesn't work
    // this->dynCallback = boost::bind(&ComputeOdometry::choose_integration_method, &this->integration_method, _1, _2);
    // this->dynServer.setCallback(this->dynCallback);
    this->dynCallback = boost::bind(&choose_integration_method, &this->integration_method, _1, _2);
    this->dynServer.setCallback(this->dynCallback);
}

/**
 * @brief Get the message coming from the topic /cmd_vel and compute the odometry,
 * publishing it to the topic /odom
 * 
 * @param msg message coming from the topic /cmd_vel
 */
void ComputeOdometry::compute_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    this->stamp_ns[0] = this->stamp_ns[1];
    this->stamp_ns[1] = msg->header.stamp.nsec;

    // During the first iteration (the older stamp stays to -1) initialize the poses and return.
    if (this->stamp_ns[0] == -1) {
    // Initialize the poses to the initial pose got in the parameter values.
    // TODO -> It can be done dynamically by getting the first values of the bag from `msg`. I did not do it because the assignment of the project required to do it statically.
    //ODOM
    this->current_pose.x = this->init_pose_x;
    this->current_pose.y = this->init_pose_y;
    this->current_pose.th = this->init_pose_th;

    return;
  }

  double vel_x = msg->twist.linear.x;
  double vel_y = msg->twist.linear.y;
  double vel_th = msg->twist.angular.z;

  if (this->integration_method == 0) {
    this->compute_euler_odometry(vel_x, vel_y, vel_th);
  }
  else if (this->integration_method == 1) {
    this->compute_rungekutta_odometry(vel_x, vel_y, vel_th);
  }

  nav_msgs::Odometry odom_msg;
  //Quaternion for theta
  tf2::Quaternion q;

  // set header
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  // set position
  odom_msg.pose.pose.position.x = this->current_pose.x;
  odom_msg.pose.pose.position.y = this->current_pose.y;
  odom_msg.pose.pose.position.z = 0;
  // set theta
  q.setRPY(0, 0, this->current_pose.th);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  // set covariance
  odom_msg.pose.covariance.fill(0);
  // set twist
  odom_msg.twist.twist.linear.x = vel_x;
  odom_msg.twist.twist.linear.y = vel_y;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = vel_th;
  // set covariance
  odom_msg.twist.covariance.fill(0);

  this->pub_odometry.publish(odom_msg);
  this->pub_transform(odom_msg);

  //TODO -> this is only a trial
  // set header
  this->trans_world_odom.header.stamp = odom_msg.header.stamp;
  this->trans_world_odom.header.frame_id = "world";
  this->trans_world_odom.child_frame_id = "odom";
  // set x,y
  this->trans_world_odom.transform.translation.x = this->init_pose_x;
  this->trans_world_odom.transform.translation.y = this->init_pose_y;
  this->trans_world_odom.transform.translation.z = 0;
  // set theta
  q.setRPY(0, 0, this->init_pose_th);
  this->trans_world_odom.transform.rotation.x = q.x();
  this->trans_world_odom.transform.rotation.y = q.y();
  this->trans_world_odom.transform.rotation.z = q.z();
  this->trans_world_odom.transform.rotation.w = q.w();
  // send transform
  this->br.sendTransform(trans_world_odom);
}

/**
 * @brief Compute odometry using Euler integration
 * 
 * @param vel_x robot linear velocity along x
 * @param vel_y robot linear velocity along y
 * @param vel_th robot angular velocity around z
 * @return the previous pose of the robot
 */
pose ComputeOdometry::compute_euler_odometry(double vel_x, double vel_y, double vel_th){
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

/**
 * @brief Compute odometry using Runge-Kutta integration
 * 
 * @param vel_x robot linear velocity along x
 * @param vel_y robot linear velocity along y
 * @param vel_th robot angular velocity around z
 * @return the previous pose of the robot
 */
pose ComputeOdometry::compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th){
    //TODO
  return this->current_pose;
}

void ComputeOdometry::pub_transform(const nav_msgs::Odometry msg){
  // set header
  this->trans_odom_baselink.header.stamp = msg.header.stamp;
  this->trans_odom_baselink.header.frame_id = msg.header.frame_id;
  this->trans_odom_baselink.child_frame_id = msg.child_frame_id;
  // set x,y
  this->trans_odom_baselink.transform.translation.x = msg.pose.pose.orientation.x;
  this->trans_odom_baselink.transform.translation.y = msg.pose.pose.orientation.y;
  this->trans_odom_baselink.transform.translation.z = msg.pose.pose.orientation.z;
  // set theta
  tf2::Quaternion q;
  q.setRPY(0, 0, msg.twist.twist.angular.z);
  this->trans_odom_baselink.transform.rotation.x = q.x();
  this->trans_odom_baselink.transform.rotation.y = q.y();
  this->trans_odom_baselink.transform.rotation.z = q.z();
  this->trans_odom_baselink.transform.rotation.w = q.w();
  // send transform
  this->br.sendTransform(trans_odom_baselink);
}

bool ComputeOdometry::reset_odometry(omnidirectional_robot_odometry::ResetOdometry::Request  &req,
                                        omnidirectional_robot_odometry::ResetOdometry::Response &res) {

    res.old_x = this->current_pose.x;
    res.old_y = this->current_pose.y;
    res.old_th = this->current_pose.th;

    this->current_pose.x = req.new_x;
    this->current_pose.y = req.new_y;
    this->current_pose.th = req.new_th;

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