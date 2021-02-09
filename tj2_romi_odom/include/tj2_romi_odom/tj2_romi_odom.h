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


using namespace std;

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
    double speed_smooth_k_left, speed_smooth_k_right;

    // State variables
    ros::Time odom_timestamp;
    ros::Time prev_odom_time;
    double prev_left_dist, prev_right_dist;
    double left_dist, right_dist;
    double left_speed, right_speed;

    // Publishers
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher odom_pub;

    // Subscribers
    ros::Subscriber twist_sub;
    ros::Subscriber encoder_left_sub;
    ros::Subscriber encoder_right_sub;

    // Sub callbacks
    void twist_callback(geometry_msgs::Twist msg);
    void encoder_left_callback(std_msgs::Float64 msg);
    void encoder_right_callback(std_msgs::Float64 msg);

    // messages
    OdomState* odom_state;
    nav_msgs::Odometry odom_msg;

    // Compute odometry
    void odom_estimator_update(double left_speed, double right_speed, double dt);
    void compute_odometry();
    void publish_chassis_data();

    void loop();
    
public:
    TJ2RomiOdom(ros::NodeHandle* nodehandle);
    int run();
};