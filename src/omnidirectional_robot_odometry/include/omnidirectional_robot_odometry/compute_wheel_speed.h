#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include "omnidirectional_robot_odometry/CustomRpm.h"

enum Wheel {
    FL = (int) 0,
    FR = (int) 1,
    RL = (int) 2,
    RR = (int) 3
};

struct wheels_speed {
    double rpm_fl;
    double rpm_fr;
    double rpm_rr;
    double rpm_rl;
};

class ComputeControl {
public:
    ComputeControl();
    void main_loop();
private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber_cmd_vel;
    ros::Publisher publish_wheel_speed;

    void sub_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    wheels_speed compute_wheels_speed(double vel_x, double vel_y, double vel_th);
};