#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "util.h"

enum Wheel {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};

struct pose {
    double x;
    double y;
    double th;
};

class ComputeVelocities {
public:
    ComputeVelocities();
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_wheel_vel;
    ros::Publisher pub_velocities;

    double ticks_wheels[4][2];
    double stamp_ns[2] = { -1, -1 };

    void compute_velocities(const sensor_msgs::JointState::ConstPtr& msg);
};