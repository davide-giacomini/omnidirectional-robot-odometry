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

/**
 * @brief This class is aimed at compute the velocities of the robot
 *  starting from the ticks of the encoders.
 *  It then publishes the velocities to the topic `cmd_vel`
 * 
 */
class ComputeVelocities {
public:
    ComputeVelocities();
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_wheel_vel;  // Subscriber to `wheel_states` topic
    ros::Publisher pub_velocities;  // Publisher to `cmd_vel` topic

    double ticks_wheels[4][2];  // Matrix with the four wheels, and for each wheel the ticks taken the previous timestamp and the current timestamp, to compute the difference between them
    double stamp_ns[2] = { -1, -1 }; // Timestamp in nanoseconds, initialized to -1 for ignoring the first computation

    /**
     * @brief Compute the velocities of the robot and publishes them to `cmd_vel` topic
     * 
     * @param msg the message from the `wheel_states` topic
     */
    void compute_velocities(const sensor_msgs::JointState::ConstPtr& msg);
};