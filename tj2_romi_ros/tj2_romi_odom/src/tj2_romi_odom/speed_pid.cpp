#include <tj2_romi_odom/speed_pid.h>

SpeedPID::SpeedPID(string name)
{
    _name = name;
    _target = 0.0;
    _error_sum = 0.0;
    _prev_error = 0.0;
    _feedforward = 0.0;
    _prev_setpoint_time = CURRENT_TIME;
    _current_time = CURRENT_TIME;
    _prev_update_time = CURRENT_TIME;
    _dt = 0.0;
    _out = 0.0;
    _is_timed_out = false;

    max_command = 1.0;
    setpoint_timeout = 1.0;

    K_ff = 1.0;  // assuming PID is in m/s
    Kp = 0.01;
    Ki = 0.0;
    Kd = 0.0;
}


bool SpeedPID::timed_out() {
    return _is_timed_out;
}

void SpeedPID::set_target(double target) {
    _feedforward = K_ff * target;
    _target = target;
    _prev_setpoint_time = CURRENT_TIME;
    _prev_update_time = CURRENT_TIME;
    _is_timed_out = false;
}

double SpeedPID::get_target() {
    return _target;
}

void SpeedPID::reset() {
    _prev_error = 0.0;
    _error_sum = 0.0;
    set_target(0.0);
}

double SpeedPID::limit(double value) {
    if (value > max_command) {
        return max_command;
    }
    if (value < -max_command) {
        return -max_command;
    }
    return value;
}

double SpeedPID::compute(double measurement)
{
    if (!_is_timed_out)
    {
        double setpoint_dt = (_current_time - _prev_setpoint_time).toSec();
        if (setpoint_dt > setpoint_timeout)
        {
            set_target(0.0);
            _is_timed_out = true;
            ROS_INFO("PID '%s' setpoint timed out", _name.c_str());
        }
    }

    _current_time = CURRENT_TIME;
    _dt = (_current_time - _prev_update_time).toSec();
    if (_dt == 0.0) {
        return _out;
    }

    double error = _target - measurement;
    _prev_update_time = _current_time;

    _out = 0.0;
    if (Kp != 0.0) {
        _out += Kp * error;
    }
    if (Kd != 0.0) {
        _out += Kd * (error - _prev_error) / _dt;
        _prev_error = error;
    }
    if (Ki != 0.0) {
        _out += Ki * _error_sum * _dt;
        _error_sum += error;
    }
    _out += _feedforward;

    return limit(_out);
}