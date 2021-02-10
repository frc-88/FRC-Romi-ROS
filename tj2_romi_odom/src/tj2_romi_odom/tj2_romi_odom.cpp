#include "tj2_romi_odom/tj2_romi_odom.h"


TJ2RomiOdom::TJ2RomiOdom(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    // Topic parameters
    ros::param::param<bool>("~publish_odom_tf", publish_odom_tf, true);
    ros::param::param<bool>("~use_sensor_msg_time", use_sensor_msg_time, true);

    // Twist parameters
    ros::param::param<double>("~min_angular_speed", min_angular_speed, 0.0);
    ros::param::param<double>("~max_angular_speed", max_angular_speed, 0.0);
    ros::param::param<double>("~min_linear_speed", min_linear_speed, 0.0);
    ros::param::param<double>("~max_linear_speed", max_linear_speed, 0.0);
    ros::param::param<double>("~zero_speed_epsilon", zero_speed_epsilon, 0.01);
    ros::param::param<double>("~max_cmd", max_cmd, 1.0);
    ros::param::param<double>("~min_cmd", min_cmd, -1.0);

    // Publishers
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    motor_left_pub = nh.advertise<std_msgs::Float64>("motor_left", 50);
    motor_right_pub = nh.advertise<std_msgs::Float64>("motor_right", 50);

    // Subscribers
    twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &TJ2RomiOdom::twist_callback, this);
    encoder_left_sub = nh.subscribe<std_msgs::Float64>("encoder_left", 50, &TJ2RomiOdom::encoder_left_callback, this);
    encoder_right_sub = nh.subscribe<std_msgs::Float64>("encoder_right", 50, &TJ2RomiOdom::encoder_right_callback, this);

    // TF parameters
    ros::param::param<string>("~child_frame", child_frame, "base_link");
    ros::param::param<string>("~odom_parent_frame", odom_parent_frame, "odom");

    // Odometry state
    ros::param::param<double>("~speed_smooth_k_left", speed_smooth_k_left, 0.9);
    ros::param::param<double>("~speed_smooth_k_right", speed_smooth_k_right, 0.9);
    ros::param::param<double>("~wheel_distance_m", wheel_distance_m, 0.1405);

    odom_timestamp = ros::Time::now();
    prev_odom_time = ros::Time::now();
    prev_left_dist = 0.0;
    prev_right_dist = 0.0;
    left_dist = 0.0;
    right_dist = 0.0;
    left_speed = 0.0;
    right_speed = 0.0;

    odom_state = init_odom_state();

    odom_msg.header.frame_id = odom_parent_frame;
    odom_msg.child_frame_id = child_frame;
    /* [
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
    ] */
    /* [
         0,  1,  2,  3,  4,  5,
         6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35
    ] */
    // odom_msg.pose.covariance.resize(36);
    odom_msg.pose.covariance[0] = 1e-3;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[14] = 1e-3;
    odom_msg.pose.covariance[21] = 1e-3;
    odom_msg.pose.covariance[28] = 1e-3;
    odom_msg.pose.covariance[35] = 1e-3;

    ROS_INFO("tj2_romi_odom init done");
}

void TJ2RomiOdom::loop()
{
    compute_odometry();
    publish_chassis_data();
}

int TJ2RomiOdom::run()
{
    ros::Rate clock_rate(60);  // run loop at 60 Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    return exit_code;
}

void TJ2RomiOdom::encoder_left_callback(std_msgs::Float64 msg)
{
    left_dist = msg.data;
}

void TJ2RomiOdom::encoder_right_callback(std_msgs::Float64 msg)
{
    right_dist = msg.data;
}


double TJ2RomiOdom::bound_speed(double value, double lower, double upper, double epsilon)
{
    double abs_value = abs(value);
    if (abs_value < lower) {
        if (abs_value > epsilon) {
            value = copysign(lower, value);
        }
        else {
            value = 0.0;
        }
    }
    if (upper != 0.0 && abs_value > upper) {
        value = copysign(upper, value);
    }
    return value;
}

double TJ2RomiOdom::m_to_cmd(double value_m)
{
    return value_m / max_linear_speed;
}


void TJ2RomiOdom::twist_callback(geometry_msgs::Twist msg)
{
    double linear_speed_mps = msg.linear.x;  // m/s
    double angular_speed_radps = msg.angular.z;  // rad/s

    linear_speed_mps = bound_speed(linear_speed_mps, min_linear_speed, max_linear_speed, zero_speed_epsilon);
    angular_speed_radps = bound_speed(
        angular_speed_radps,
        (linear_speed_mps == 0.0) ? min_angular_speed : 0.0,
        max_angular_speed,
        (linear_speed_mps == 0.0) ? zero_speed_epsilon : 0.0);

    // arc = angle * radius
    // rotation speed at the wheels
    double rotational_speed_mps = angular_speed_radps * wheel_distance_m / 2.0;

    int64_t left_command = m_to_cmd(linear_speed_mps - rotational_speed_mps);
    int64_t right_command = m_to_cmd(linear_speed_mps + rotational_speed_mps);

    int64_t larger_cmd = max(left_command, right_command);
    if (abs(larger_cmd) > max_cmd)
    {
        int64_t abs_left = abs(left_command);
        int64_t abs_right = abs(right_command);
        if (abs_left > abs_right) {
            left_command = (int64_t)copysign(max_cmd, left_command);
            right_command = (int64_t)copysign(max_cmd * (double)abs_right / (double)abs_left, right_command);
        }
        else {
            left_command = (int64_t)copysign(max_cmd * (double)abs_left / (double)abs_right, left_command);
            right_command = (int64_t)copysign(max_cmd, right_command);
        }
    }

    motor_left_msg.data = left_command;
    motor_right_msg.data = right_command;

    motor_left_pub.publish(motor_left_msg);
    motor_right_pub.publish(motor_right_msg);
}

//
// Compute odometry
//

void TJ2RomiOdom::odom_estimator_update(double left_speed, double right_speed, double dt)
{
    double theta_n_1 = odom_state->theta;

    double v = (left_speed + right_speed) / 2.0;
    double w = (right_speed - left_speed) / wheel_distance_m;

    // reference: https://www.cs.cmu.edu/16311/current/labs/lab03/
    double k00 = v * cos(theta_n_1);
    double k01 = v * sin(theta_n_1);
    double k02 = w;

    double k10 = v * cos(theta_n_1 + dt / 2.0 * k02);
    double k11 = v * sin(theta_n_1 + dt / 2.0 * k02);
    double k12 = w;

    double k20 = v * cos(theta_n_1 + dt / 2.0 * k12);
    double k21 = v * sin(theta_n_1 + dt / 2.0 * k12);
    double k22 = w;

    double k30 = v * cos(theta_n_1 + dt / 2.0 * k22);
    double k31 = v * sin(theta_n_1 + dt / 2.0 * k22);
    double k32 = w;

    double dx     = dt / 6.0 * (k00 + 2 * (k10 + k20) + k30);
    double dy     = dt / 6.0 * (k01 + 2 * (k11 + k21) + k31);
    double dtheta = dt / 6.0 * (k02 + 2 * (k12 + k22) + k32);
    
    odom_state->x += dx;
    odom_state->y += dy;
    odom_state->theta += dtheta;
    odom_state->theta = fmod(odom_state->theta, 2.0 * M_PI);

    odom_state->vx = v * cos(odom_state->theta);
    odom_state->vy = v * sin(odom_state->theta);

    odom_state->v = v;
    odom_state->w = w;
}

void TJ2RomiOdom::compute_odometry()
{
    ros::Time now = ros::Time::now();
    odom_timestamp = now;
    double dt = (now - prev_odom_time).toSec();
    prev_odom_time = now;

    double delta_left = left_dist - prev_left_dist;
    double delta_right = right_dist - prev_right_dist;
    double left_speed_raw = delta_left / dt;
    double right_speed_raw = delta_right / dt;
    left_speed += speed_smooth_k_left * (left_speed_raw - left_speed);
    right_speed += speed_smooth_k_right * (right_speed_raw - right_speed);

    prev_left_dist = left_dist;
    prev_right_dist = right_dist;

    odom_estimator_update(
        left_speed,
        right_speed,
        dt
    );
}

void TJ2RomiOdom::publish_chassis_data()
{
    ros::Time now;
    if (use_sensor_msg_time) {
        now = odom_timestamp;
    }
    else {
        now = ros::Time::now();
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_state->theta);

    geometry_msgs::Quaternion quat_msg;
    tf2::convert(q, quat_msg);

    if (publish_odom_tf)
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = odom_parent_frame;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = odom_state->x;
        transformStamped.transform.translation.y = odom_state->y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster.sendTransform(transformStamped);
    }

    odom_msg.header.stamp = now;
    odom_msg.pose.pose.position.x = odom_state->x;
    odom_msg.pose.pose.position.y = odom_state->y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = quat_msg;

    odom_msg.twist.twist.linear.x = odom_state->vx;
    odom_msg.twist.twist.linear.y = odom_state->vy;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = odom_state->w;

    odom_pub.publish(odom_msg);
}