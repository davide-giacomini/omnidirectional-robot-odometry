#include "omnidirectional_robot_odometry/compute_odometry.h"
#include <tf/transform_listener.h>

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
    this->current_pose.x = this->init_pose_x;
    this->current_pose.y = this->init_pose_y;
    this->current_pose.th = this->init_pose_th;
    //TODO -> I have to start from 0 in the odometry

    return;
  }

  double vel_tau = msg->twist.linear.x;
  double vel_eta = msg->twist.linear.y;
  double vel_th = msg->twist.angular.z;

  velocity vel_odom;

  if (this->integration_method == 0) {
    vel_odom = this->compute_euler_odometry(vel_tau, vel_eta, vel_th);
  }
  else if (this->integration_method == 1) {
    vel_odom = this->compute_rungekutta_odometry(vel_tau, vel_eta, vel_th);
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
  odom_msg.twist.twist.linear.x = vel_odom.x;
  odom_msg.twist.twist.linear.y = vel_odom.y;
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
  // this->trans_world_odom.header.stamp = odom_msg.header.stamp;
  // this->trans_world_odom.header.frame_id = "world";
  // this->trans_world_odom.child_frame_id = odom_msg.header.frame_id;
  // // set x,y
  // this->trans_world_odom.transform.translation.x = this->init_pose_x;
  // this->trans_world_odom.transform.translation.y = this->init_pose_y;
  // this->trans_world_odom.transform.translation.z = 0;
  // // set theta
  // q.setRPY(0, 0, this->init_pose_th);
  // this->trans_world_odom.transform.rotation.x = q.x();
  // this->trans_world_odom.transform.rotation.y = q.y();
  // this->trans_world_odom.transform.rotation.z = q.z();
  // this->trans_world_odom.transform.rotation.w = q.w();
  // // send transform
  // this->br.sendTransform(trans_world_odom);
}

/**
 * @brief Compute odometry using Euler integration
 * 
 * @param vel_x robot linear velocity along x
 * @param vel_y robot linear velocity along y
 * @param vel_th robot angular velocity around z
 * @return the previous pose of the robot
 */
velocity ComputeOdometry::compute_euler_odometry(double vel_tau, double vel_eta, double vel_th){
  velocity vel_relative_to_odom;
  pose previous_pose;

  previous_pose.x = this->current_pose.x;
  previous_pose.y = this->current_pose.y;
  previous_pose.th = this->current_pose.th;

  double dt = (this->stamp_ns[1] - this->stamp_ns[0]) * pow(10.0, -9.0); // `dt` is in seconds.

  vel_relative_to_odom.x = vel_tau*cos(previous_pose.th) - vel_eta*sin(previous_pose.th);
  vel_relative_to_odom.y = vel_tau*sin(previous_pose.th) + vel_eta*cos(previous_pose.th);

  this->current_pose.x = previous_pose.x + dt*vel_relative_to_odom.x;
  this->current_pose.y = previous_pose.y + dt*vel_relative_to_odom.y;
  this->current_pose.th = previous_pose.th + dt*vel_th;

  return vel_relative_to_odom;
}

/**
 * @brief Compute odometry using Runge-Kutta integration
 * 
 * @param vel_x robot linear velocity along x
 * @param vel_y robot linear velocity along y
 * @param vel_th robot angular velocity around z
 * @return the previous pose of the robot
 */
velocity ComputeOdometry::compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th){
    //TODO
    velocity v;
  return v;
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

/**
 * @brief This is useful to set the initial pose parameters.
 * I copied and pasted the four values of the quaternions manually directly from the bag.
 * I use the printed information for getting the yaw.
 * 
 */
void convert_initial_quaternions() {
  tf::Quaternion q1(-0.011577633209526539, -0.02075166068971157, -0.019595127552747726, 0.9995256066322327);
  tf::Matrix3x3 m1(q1);
  double roll1, pitch1, yaw1;
  m1.getRPY(roll1, pitch1, yaw1);

  tf::Quaternion q2(0.0338488332927227, -0.05557403340935707, -0.0028051661793142557, 0.997876763343811);
  tf::Matrix3x3 m2(q2);
  double roll2, pitch2, yaw2;
  m2.getRPY(roll2, pitch2, yaw2);

  tf::Quaternion q3(-0.010593205690383911, -0.024768192321062088, -0.006751589477062225, 0.9996143579483032);
  tf::Matrix3x3 m3(q3);
  double roll3, pitch3, yaw3;
  m3.getRPY(roll3, pitch3, yaw3);

  ROS_INFO("Roll 1: [%.18f]", roll1);
  ROS_INFO("Pitch 1: [%.18f]", pitch1);
  ROS_INFO("Yaw 1: [%.18f]", yaw1);

  ROS_INFO("Roll 2: [%.18f]", roll2);
  ROS_INFO("Pitch 2: [%.18f]", pitch2);
  ROS_INFO("Yaw 2: [%.18f]", yaw2);

  ROS_INFO("Roll 3: [%.18f]", roll3);
  ROS_INFO("Pitch 3: [%.18f]", pitch3);
  ROS_INFO("Yaw 3: [%.18f]", yaw3);
}

/**
 * @brief Using the yaw calculated in the `convert_initial_quaternions` function,
 * I set roll and pitch to 0 and calculate the resulting quaternions for each bag.
 * Those quaternions could be useful for the transformation tf from world to odom.
 * 
 */
void getting_initial_quaternions_from_pose() {

    tf2::Quaternion q1;
    q1.setRPY(0, 0, -0.038734904526281436);
    tf2::Quaternion q2;
    q2.setRPY(0, 0, -0.009418701789503428);
    tf2::Quaternion q3;
    q3.setRPY(0, 0, -0.012989612659950940);

    ROS_INFO("Init x bag 1: [%.18f]", q1.x());
    ROS_INFO("Init y bag 1: [%.18f]", q1.y());
    ROS_INFO("Init z bag 1: [%.18f]", q1.z());
    ROS_INFO("Init w bag 1: [%.18f]", q1.w());

    ROS_INFO("Init x bag 2: [%.18f]", q2.x());
    ROS_INFO("Init y bag 2: [%.18f]", q2.y());
    ROS_INFO("Init z bag 2: [%.18f]", q2.z());
    ROS_INFO("Init w bag 2: [%.18f]", q2.w());

    ROS_INFO("Init x bag 3: [%.18f]", q3.x());
    ROS_INFO("Init y bag 3: [%.18f]", q3.y());
    ROS_INFO("Init z bag 3: [%.18f]", q3.z());
    ROS_INFO("Init w bag 3: [%.18f]", q3.w());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_odometry");

  ComputeOdometry co;

  convert_initial_quaternions();
  getting_initial_quaternions_from_pose();
  
  ros::spin();

  return 0;
}