#include <omnidirectional-robot-odometry/sub.h>

KinematicsToOdometry::KinematicsToOdometry()
{
  this->sub = this->nh.subscribe("wheel_states", 1000, &KinematicsToOdometry::compute_velocities, this);
  this->pub_velocities = this->nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
}

void KinematicsToOdometry::compute_velocities(const sensor_msgs::JointState::ConstPtr& msg) {

  this->stamp_ns[0] = this->stamp_ns[1];
  this->stamp_ns[1] = msg->header.stamp.nsec;

  if (this->stamp_ns[0] == -1) {
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
