#include <omnidirectional_robot_odometry/compute_velocities.h>

ComputeVelocities::ComputeVelocities()
{
  this->sub_wheel_vel = this->nh.subscribe("wheel_states", 1000, &ComputeVelocities::compute_velocities, this);
  this->pub_velocities = this->nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
}

void ComputeVelocities::compute_velocities(const sensor_msgs::JointState::ConstPtr& msg) {

  this->stamp_ns[0] = this->stamp_ns[1];
  this->stamp_ns[1] = msg->header.stamp.nsec;

  if (this->stamp_ns[0] == -1) {
    return;
  }

  for (int wheel = 0; wheel < 4; wheel++) {
    this->ticks_wheels[wheel][0] = this->ticks_wheels[wheel][1];
    this->ticks_wheels[wheel][1] = msg->position[wheel];
  }


  double omega_wheels[4];
  // Useless, but it makes the code more readable
  int N = Util::N;
  int T = Util::T;
  double r = Util::r;
  double l = Util::l;
  double w = Util::w;

  for (int wheel = 0; wheel < 4; wheel++) {
    omega_wheels[wheel] = (this->ticks_wheels[wheel][1]-this->ticks_wheels[wheel][0])/(this->stamp_ns[1]-this->stamp_ns[0])/N/T*2*M_PI*pow(10, 9);
  }

  double vel_x = (r/4) * (omega_wheels[Wheel::FL] + omega_wheels[Wheel::FR] + omega_wheels[Wheel::RL]+ omega_wheels[Wheel::RR]);
  double vel_y = (r/4) * ( - omega_wheels[Wheel::FL] + omega_wheels[Wheel::FR] + omega_wheels[Wheel::RL] - omega_wheels[Wheel::RR]);
  double vel_th = (r/(4*(l + w))) * ( - omega_wheels[Wheel::FL] + omega_wheels[Wheel::FR] - omega_wheels[Wheel::RL]+ omega_wheels[Wheel::RR]);

  geometry_msgs::TwistStamped pub_msg;

  pub_msg.header.stamp = msg->header.stamp;
  pub_msg.header.frame_id = msg->header.frame_id;
  pub_msg.twist.linear.x = vel_x;
  pub_msg.twist.linear.y = vel_y;
  pub_msg.twist.linear.z = 0;
  pub_msg.twist.angular.x = 0;
  pub_msg.twist.angular.y = 0;
  pub_msg.twist.angular.z = vel_th;

  this->pub_velocities.publish(pub_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_velocities");

  ComputeVelocities cv;
  
  ros::spin();

  return 0;
}
