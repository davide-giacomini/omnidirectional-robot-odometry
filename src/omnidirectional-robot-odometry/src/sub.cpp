#include <omnidirectional-robot-odometry/sub.h>

KinematicsToOdometry::KinematicsToOdometry()
{
  this->sub = this->nh.subscribe("wheel_states", 1000, &KinematicsToOdometry::compute_velocities, this);
  this->pub_velocities = this->nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  this->pub_odometry = this->nh.advertise<nav_msgs::Odometry>("odom", 1000);

  this->nh.getParam("/init_pose_x", this->init_pose_x);
  this->nh.getParam("/init_pose_y", this->init_pose_y);
  this->nh.getParam("/init_pose_th", this->init_pose_th);
}

void KinematicsToOdometry::compute_velocities(const sensor_msgs::JointState::ConstPtr& msg) {

  this->stamp_ns[0] = this->stamp_ns[1];
  this->stamp_ns[1] = msg->header.stamp.nsec;

  if (this->stamp_ns[0] == -1) {
    // Initialize the poses to the initial pose got in the parameter values.
    // TODO -> It can be done dynamically by getting the first values of the bag from `msg`. I did not do it because the assignment of the project required so.
    this->current_pose.x = this->init_pose_x;
    this->current_pose.y = this->init_pose_y;
    this->current_pose.th = this->init_pose_th;

    return;
  }

  for (int wheel = 0; wheel < 4; wheel++) {
    this->ticks_wheels[wheel][0] = this->ticks_wheels[wheel][1];
    this->ticks_wheels[wheel][1] = msg->position[wheel];
  }


  double ang_vel_wheels1[4];
  double ang_vel_wheels2[4];

  for (int wheel = 0; wheel < 4; wheel++) {
    ang_vel_wheels1[wheel] = (this->ticks_wheels[wheel][1]-this->ticks_wheels[wheel][0])/(this->stamp_ns[1]-this->stamp_ns[0])/this->N/this->T*2*M_PI*pow(10, 9);
  }

  for (int wheel = 0; wheel < 4; wheel++) {
    ang_vel_wheels2[wheel] = msg->velocity[wheel]/60/this->T;
  }

  double vel_x = (this->r/4) * (ang_vel_wheels1[Wheel::FL] + ang_vel_wheels1[Wheel::FR] + ang_vel_wheels1[Wheel::RL]+ ang_vel_wheels1[Wheel::RR]);
  double vel_y = (this->r/4) * ( - ang_vel_wheels1[Wheel::FL] + ang_vel_wheels1[Wheel::FR] + ang_vel_wheels1[Wheel::RL] - ang_vel_wheels1[Wheel::RR]);
  double om_z = (this->r/(4*(this->l + this->w))) * ( - ang_vel_wheels1[Wheel::FL] + ang_vel_wheels1[Wheel::FR] - ang_vel_wheels1[Wheel::RL]+ ang_vel_wheels1[Wheel::RR]);

  geometry_msgs::TwistStamped pub_msg;

  pub_msg.header.stamp = msg->header.stamp;
  pub_msg.header.frame_id = msg->header.frame_id;
  pub_msg.twist.linear.x = vel_x;
  pub_msg.twist.linear.y = vel_y;
  pub_msg.twist.linear.z = 0;
  pub_msg.twist.angular.x = 0;
  pub_msg.twist.angular.y = 0;
  pub_msg.twist.angular.z = om_z;

  this->pub_velocities.publish(pub_msg);
  
  this->compute_euler_odometry(vel_x, vel_y, om_z);

  // nav_msgs::Odometry odom_msg;
  // this->pub_odometry.publish(this->current_pose);
}

pose KinematicsToOdometry::compute_euler_odometry(double vel_x, double vel_y, double vel_th){
  pose previous_pose;

  previous_pose.x = this->current_pose.x;
  previous_pose.y = this->current_pose.y;
  previous_pose.th = this->current_pose.th;

  double dt = (this->stamp_ns[1] - this->stamp_ns[0]*pow(10, -9)); // The pow is to convert it in seconds.

  this->current_pose.x = previous_pose.x + dt*vel_x;
  this->current_pose.y = previous_pose.y + dt*vel_x;
  this->current_pose.th = previous_pose.th + dt*vel_th;

  return previous_pose;
}

pose KinematicsToOdometry::compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th){
  return this->current_pose;
}

void KinematicsToOdometry::main_loop()
{
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");

  KinematicsToOdometry my_subscriber;
  my_subscriber.main_loop();

  return 0;
}
