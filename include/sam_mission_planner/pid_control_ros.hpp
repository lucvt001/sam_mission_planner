#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "pid.h"

using namespace std;
using Float32 = std_msgs::msg::Float32;

class PidControl : public rclcpp::Node 
{

public:
    PidControl();

private:
    void subCb(const Float32::SharedPtr msg);
    void initializePidController();

    rclcpp::Subscription<Float32>::SharedPtr subscription_;
    rclcpp::Publisher<Float32>::SharedPtr control_pub_;

    PID pid_ = PID(0,0,0,0,0,0);
    float setpoint_;
    int is_axis_inverted_;
};

#endif // PID_CONTROL_HPP
