#include <omnidirectional-robot-odometry/sub.h>

KinematicsToOdometry::KinematicsToOdometry()
{
  this->sub = this->nh.subscribe("wheel_states", 1000, &KinematicsToOdometry::compute_velocities, this);
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

  std::cout << ang_vel_wheels1[0] << std::endl;
  std::cout << ang_vel_wheels2[0] << std::endl;
  std::cout << ang_vel_wheels1[1] << std::endl;
  std::cout << ang_vel_wheels2[1] << std::endl;
  std::cout << ang_vel_wheels1[2] << std::endl;
  std::cout << ang_vel_wheels2[2] << std::endl;
  std::cout << ang_vel_wheels1[3] << std::endl;
  std::cout << ang_vel_wheels2[3] << std::endl;
  std::cout << *msg << std::endl;
  // ROS_INFO("I heard: [%lf]", msg->pose.position.x);
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
