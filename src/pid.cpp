#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "sam_mission_planner/pid.h"

using namespace std;

float PID::calculate( float setpoint, float current )
{
    
    // Calculate error
    float error = setpoint - current;

    // Proportional term
    float Pout = Kp_ * error;

    // Integral term
    if (error * integral_ > 0)
        integral_ += error * dt_;
    else
        integral_ += error * dt_ * unwinding_factor_;
    float Iout = Ki_ * integral_;

    // Derivative term
    float derivative = (error - prev_error_) / dt_;
    float Dout = Kd_ * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    // Save error to previous error
    prev_error_ = error;

    if ( verbose_mode_ )
        // cout << pid_name_ << " - error: " << error << " integral: " << integral_ << " derivative: " << derivative << " Output: " << output << endl;
        cout << pid_name_ << " - Pout: " << Pout << " Iout: " << Iout << " Dout: " << Dout << " Output: " << output << endl;

    return output;
}

void PID::set_unwinding_factor( float unwinding_factor )
{
    unwinding_factor_ = unwinding_factor;
}

void PID::reset()
{
    prev_error_ = 0.0;
    integral_ = 0.0;
}

void PID::enable_verbose_mode(bool is_enable, string pid_name)
{
    verbose_mode_ = is_enable;
    pid_name_ = pid_name;
}

#endif