#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

enum class Wheel {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};

class KinematicsToOdometry {
public:
    KinematicsToOdometry();
    void main_loop();
    void compute_velocities(const sensor_msgs::JointState::ConstPtr& msg);
private:
    static const int N = 42;// Encoders resolution
    static const int T = 5; // 5:1 is the gear ratio

    ros::NodeHandle nh;
    ros::Subscriber sub;
    double ticks_wheels[4][2];
    double stamp_ns[2] = { -1, -1 };
};