#pragma once
#include "ros/ros.h"

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <std_msgs/Float64.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/JointState.h>

#include <tj2_romi_odom/speed_pid.h>
#include <tj2_romi_odom/OdomReset.h>


using namespace std;

#define THROW_EXCEPTION(msg)  throw std::runtime_error(msg)

struct OdomState {
    double x;  // x position
    double y;  // y position
    double theta;  // angle

    double v;  // linear velocity
    double w;  // angular velocity

    double vx;  // x component of velocity
    double vy;  // y component of velocity
};


void reset_odom_state(OdomState* state)
{
    state->x = 0.0;
    state->y = 0.0;
    state->theta = 0.0;
    state->v = 0.0;
    state->w = 0.0;
    state->vx = 0.0;
    state->vy = 0.0;
}

double double_equal(double d1, double d2, double epsilon) {
    return abs(d1 - d2) < epsilon;
}


OdomState* init_odom_state()
{
    OdomState* state = new OdomState;
    reset_odom_state(state);
    return state;
}

class TJ2RomiOdom {
private:
    ros::NodeHandle nh;  // ROS node handle

    // launch parameters
    string child_frame;
    string odom_parent_frame;

    bool publish_odom_tf;
    bool use_sensor_msg_time;

    double wheel_distance_m;
    double wheel_dist_half_m;
    double speed_smooth_k_left, speed_smooth_k_right;

    double min_angular_speed, max_angular_speed;
    double min_linear_speed, max_linear_speed;
    double zero_speed_epsilon;
    double zero_epsilon;
    double max_cmd, min_cmd;
    std::vector<double> pose_covariances;
    std::vector<double> twist_covariances;

    double left_Kp, left_Ki, left_Kd;
    double right_Kp, right_Ki, right_Kd;

    // State variables
    ros::Time odom_timestamp;
    ros::Time prev_odom_time;
    double prev_left_dist, prev_right_dist;
    double left_dist, right_dist;
    double left_speed, right_speed;

    // Speed PIDs
    SpeedPID* left_pid;
    SpeedPID* right_pid;
    void compute_motor_commands();

    // Publishers
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher odom_pub;
    ros::Publisher motor_left_pub;
    ros::Publisher motor_right_pub;

    // Subscribers
    ros::Subscriber twist_sub;
    ros::Subscriber encoder_left_sub;
    ros::Subscriber encoder_right_sub;

    // Services
    ros::ServiceServer odom_reset_srv;
    string odom_reset_service_name;
    bool odom_reset_callback(tj2_romi_odom::OdomReset::Request &req, tj2_romi_odom::OdomReset::Response &resp);

    // Sub callbacks
    void twist_callback(geometry_msgs::Twist msg);
    void encoder_left_callback(std_msgs::Float64 msg);
    void encoder_right_callback(std_msgs::Float64 msg);

    // messages
    OdomState* odom_state;
    nav_msgs::Odometry odom_msg;
    std_msgs::Float64 motor_left_msg;
    std_msgs::Float64 motor_right_msg;

    // Compute odometry
    void odom_estimator_update(double left_speed, double right_speed, double dt);
    void compute_motor_speeds(double dt);
    void compute_odometry(double dt);
    void publish_chassis_data();

    // Twist commands
    double bound_speed(double value, double lower, double upper, double epsilon);
    double m_to_cmd(double value_m);

    void loop();
    
public:
    TJ2RomiOdom(ros::NodeHandle* nodehandle);
    int run();
};