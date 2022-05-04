#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

struct pose {
    double x;
    double y;
    double th;
};

class ComputeOdometry {
public:
    ComputeOdometry();
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cmd_vel;
    ros::Publisher pub_odometry;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    double stamp_ns[2] = { -1, -1 };
    double init_pose_x;
    double init_pose_y;
    double init_pose_th;
    pose current_pose;

    void compute_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg);
    pose compute_euler_odometry(double vel_x, double vel_y, double vel_th);
    pose compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th);
    void pub_transform(const nav_msgs::Odometry msg);
};