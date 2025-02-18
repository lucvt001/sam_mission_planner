#ifndef THRUSTER_ANGLES_PUBLISHER_HPP
#define THRUSTER_ANGLES_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sam_msgs/msg/thruster_angles.hpp"

using namespace std;
using Float32 = std_msgs::msg::Float32;
using ThrusterAngles = sam_msgs::msg::ThrusterAngles;

class ThrusterAnglesPublisher : public rclcpp::Node
{
public:
    ThrusterAnglesPublisher();

private:
    void publishThrusterAngles();

    rclcpp::Subscription<Float32>::SharedPtr vertical_angle_sub_;
    rclcpp::Subscription<Float32>::SharedPtr horizontal_angle_sub_;
    rclcpp::Publisher<ThrusterAngles>::SharedPtr thruster_angles_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float vertical_angle_;
    float horizontal_angle_;
};

#endif // THRUSTER_ANGLES_PUBLISHER_HPP