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
    ros::Subscriber subscriber_cmd_vel; // Subscriber to `cmd_vel`
    ros::Publisher publish_wheel_speed; // Publisher to `wheels_rpm`
    
    /**
     * @brief Compute the wheels speed starting from robot velocities
     * 
     * @param msg message containing robot velocities
     */
    void compute_wheels_speed(const geometry_msgs::TwistStamped::ConstPtr& msg);
    /**
     * @brief Formula to compute the angular wheel velocities
     * 
     * @param vel_x vel x of robot
     * @param vel_y vel y of robot
     * @param vel_th vel theta of robot
     * @return wheels_speed a struct with the angular velocities of each wheel
     */
    wheels_speed compute_speed(double vel_x, double vel_y, double vel_th);
};