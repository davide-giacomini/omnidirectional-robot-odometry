#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
//TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
//Reset Service
#include "omnidirectional_robot_odometry/ResetOdometry.h"
//Dynamic reconfigure
#include "dynamic_reconfigure/server.h"
#include "omnidirectional_robot_odometry/parametersConfig.h"

struct pose {
    double x;
    double y;
    double th;
};

struct velocity {
    double x;
    double y;
};

class ComputeOdometry {
public:
    ComputeOdometry();
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cmd_vel;
    ros::Publisher pub_odometry;
    //TF2
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans_odom_baselink;
    geometry_msgs::TransformStamped trans_world_odom;
    //Service reset
    ros::ServiceServer res_odom_service;

    double stamp_ns[2] = { -1, -1 };
    double init_pose_x;
    double init_pose_y;
    double init_pose_th;
    pose current_pose;
    //Dynamic reconfigure
    dynamic_reconfigure::Server<omnidirectional_robot_odometry::parametersConfig> dynServer;
    dynamic_reconfigure::Server<omnidirectional_robot_odometry::parametersConfig>::CallbackType dynCallback;
    int integration_method = 0;

    void compute_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg);
    velocity compute_euler_odometry(double vel_x, double vel_y, double vel_th);
    velocity compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th);

    void pub_transform(const nav_msgs::Odometry msg);

    bool reset_odometry(omnidirectional_robot_odometry::ResetOdometry::Request  &req,
                        omnidirectional_robot_odometry::ResetOdometry::Response &res);

    // TODO -> this doesn't work with dynamic reconfigure
    // void choose_integration_method(int *integration_method, omnidirectional_robot_odometry::parametersConfig &config, uint32_t level);
};