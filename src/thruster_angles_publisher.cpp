#include "sam_mission_planner/thruster_angles_publisher.hpp"

ThrusterAnglesPublisher::ThrusterAnglesPublisher()
    : Node("thruster_angles_publisher"), vertical_angle_(0.0213), horizontal_angle_(0.0)
{
    string vertical_angle_topic = this->declare_parameter<string>("vertical_angle_topic", "");
    string horizontal_angle_topic = this->declare_parameter<string>("horizontal_angle_topic", "");
    string thruster_angles_topic = this->declare_parameter<string>("thruster_angles_topic", "");

    vertical_angle_sub_ = this->create_subscription<Float32>(
        vertical_angle_topic, 10, [this](const Float32::SharedPtr msg) {
            vertical_angle_ = msg->data;
        });

    horizontal_angle_sub_ = this->create_subscription<Float32>(
        horizontal_angle_topic, 10, [this](const Float32::SharedPtr msg) {
            horizontal_angle_ = msg->data;
        });

    thruster_angles_pub_ = this->create_publisher<ThrusterAngles>(thruster_angles_topic, 10);

    timer_ = this->create_wall_timer(
        200ms, std::bind(&ThrusterAnglesPublisher::publishThrusterAngles, this));   // 5 Hz
}

void ThrusterAnglesPublisher::publishThrusterAngles()
{
    auto message = ThrusterAngles();
    message.thruster_vertical_radians = vertical_angle_;
    message.thruster_horizontal_radians = horizontal_angle_;
    message.header.stamp = this->get_clock()->now();
    thruster_angles_pub_->publish(message);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterAnglesPublisher>());
    rclcpp::shutdown();
    return 0;
}