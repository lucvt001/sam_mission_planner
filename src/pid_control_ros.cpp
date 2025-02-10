#include "sam_mission_planner/pid_control_ros.h"

PidControl::PidControl() : Node("pid_control") {
    string input_data_topic = this->declare_parameter("input_data_topic", "");
    subscription_ = this->create_subscription<Float32>(input_data_topic, 10, std::bind(&PidControl::subCb, this, std::placeholders::_1));
    
    string output_control_topic = this->declare_parameter("output_control_topic", "");
    control_pub_ = this->create_publisher<Float32>(output_control_topic, 10);
    
    setpoint_ = this->declare_parameter("setpoint", 0.0);
    is_axis_inverted_ = this->declare_parameter("is_axis_inverted", 1);

    initializePidController();
}

void PidControl::subCb(const Float32::SharedPtr msg) {
    float current = msg->data;
    float control_signal = is_axis_inverted_ * pid_.calculate(setpoint_, current);
    Float32 control_msg;
    control_msg.data = control_signal;
    control_pub_->publish(control_msg);
}

void PidControl::initializePidController() {
    float kp = this->declare_parameter("kp", 0.0);
    float ki = this->declare_parameter("ki", 0.0);
    float kd = this->declare_parameter("kd", 0.0);
    float dt = this->declare_parameter("dt", 0.1);
    float max = this->declare_parameter("max", 1.0);
    float min = this->declare_parameter("min", -1.0);
    
    if (abs(dt-0.1) > 1e-6) {
        RCLCPP_WARN(this->get_logger(), "pid/dt must be 0.1. Defaulting to dt=0.1.");
        dt = 0.1;
    }
    
    pid_ = PID(kp, ki, kd, dt, max, min);
    RCLCPP_INFO(this->get_logger(), "PID initialized with kp: %f, ki: %f, kd: %f, dt: %f, max: %f, min: %f", kp, ki, kd, dt, max, min);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
