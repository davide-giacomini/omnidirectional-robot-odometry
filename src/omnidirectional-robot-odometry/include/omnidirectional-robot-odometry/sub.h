#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

enum Wheel {
    FL = (int) 0,
    FR = (int) 1,
    RL = (int) 2,
    RR = (int) 3
};

struct pose {
    double x;
    double y;
    double th;
};

class KinematicsToOdometry {
public:
    KinematicsToOdometry();
    void main_loop();
    void compute_velocities(const sensor_msgs::JointState::ConstPtr& msg);
private:
    static const int N = 42;// Encoders resolution
    static const int T = 5; // 5:1 is the gear ratio
    static constexpr double r = 0.07; // radius metres
    static constexpr double l = 0.2; // Wheel position along x
    static constexpr double w = 0.169; // Wheel position along y

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_velocities;
    ros::Publisher pub_odometry;
    double ticks_wheels[4][2];
    double stamp_ns[2] = { -1, -1 };

    double init_pose_x;
    double init_pose_y;
    double init_pose_th;
    pose current_pose;

    pose compute_euler_odometry(double vel_x, double vel_y, double vel_th);
    pose compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th);
};