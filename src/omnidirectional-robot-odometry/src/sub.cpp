#include <omnidirectional-robot-odometry/sub.h>

Subscriber::Subscriber()
{
  this->sub = this->n.subscribe("robot/pose", 1000, &Subscriber::chatterCallback, this);
}

void Subscriber::chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  ROS_INFO("I heard: [%lf]", msg->pose.position.x);
}

void Subscriber::main_loop()
{
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");

  Subscriber my_subscriber;
  my_subscriber.main_loop();

  return 0;
}
