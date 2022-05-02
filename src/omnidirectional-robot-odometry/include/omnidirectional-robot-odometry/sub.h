#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

class Subscriber {
public:
    Subscriber();
    void main_loop();
    void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
};