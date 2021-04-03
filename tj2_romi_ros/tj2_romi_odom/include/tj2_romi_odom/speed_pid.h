#pragma once
#include "ros/ros.h"

using namespace std;

#define CURRENT_TIME ros::Time::now()

class SpeedPID
{
private:
    string _name;
    double K_ff;  // feedforward constant. (ticks per s to motor command conversion)
    double _target;
    double _error_sum, _prev_error;
    double _feedforward;
    ros::Time _prev_setpoint_time;
    ros::Time _current_time, _prev_update_time;
    double _dt;
    double _out;
    bool _is_timed_out;
    
    double limit(double value);

public:
    double max_command;
    double Kp, Ki, Kd;
    double setpoint_timeout;

    SpeedPID(string name);

    bool timed_out();
    void set_target(double target);
    double get_target();
    void reset();
    double compute(double measurement);
};
