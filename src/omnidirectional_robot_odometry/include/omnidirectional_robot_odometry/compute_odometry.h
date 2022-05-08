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

enum Integration {
    Euler = 0,
    RK = 1
};

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
    ros::Subscriber sub_cmd_vel;    // Subscriber to `cmd_vel`
    ros::Publisher pub_odometry;    // Publisher to `odom`
    //TF2
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans_odom_baselink;    // Transform from `odom` to `base_link`
    geometry_msgs::TransformStamped trans_world_odom;   // Transform from `world' to `odom`
    ros::ServiceServer res_odom_service;    // Service reset

    double stamp_ns[2] = { -1, -1 };    // Timestamp in nanoseconds, initialized to -1 for ignoring the first computation
    pose current_pose;  // Current pose with respect to the `odom` frame
    
    //Dynamic reconfigure
    dynamic_reconfigure::Server<omnidirectional_robot_odometry::parametersConfig> dynServer;
    dynamic_reconfigure::Server<omnidirectional_robot_odometry::parametersConfig>::CallbackType dynCallback;

    int integration_method = Euler;

    /**
     * @brief Get the message coming from the topic /cmd_vel and compute the odometry,
     * publishing it to the topic /odom. It also broadcasts the transform `odom`->`base_link`
     * 
     * @param msg message coming from the topic /cmd_vel
     */
    void compute_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg);
    /**
     * @brief Compute odometry from velocities relative to the `odom` frame
     * 
     * @param vel_x robot linear velocity along odom axis x
     * @param vel_y robot linear velocity along odom axis y
     * @param vel_th robot angular velocity around odom axis z
     * @return the previous pose of the robot with respect to odom
     */
    pose compute_pose(double vel_x, double vel_y, double vel_th);
    /**
     * @brief Broadcast a transformation using the information of the odometry message
     * 
     * @param msg odometry message
     */
    void pub_transform(const nav_msgs::Odometry msg);
    /**
     * @brief Reset the odometry frame to the position given as input
     * 
     * @param req new values
     * @param res old values
     * @return true if everything worked correctly
     */
    bool reset_odometry(omnidirectional_robot_odometry::ResetOdometry::Request  &req,
                        omnidirectional_robot_odometry::ResetOdometry::Response &res);

    // TODO -> this doesn't work with dynamic reconfigure if declared inside the class. I declared it inside the .cpp file.
    // void choose_integration_method(int *integration_method, omnidirectional_robot_odometry::parametersConfig &config, uint32_t level);
};