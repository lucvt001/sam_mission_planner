#include "sam_mission_planner/relative_target_position_publisher.hpp"

RelativeTargetPositionPublisher::RelativeTargetPositionPublisher()
    : Node("relative_target_position_publisher"), prev_relative_position_({0.0, 0.0, 0.0}), toggle_(false)
{
    // Declare and get parameters
    offset_x_ = this->declare_parameter<float>("x", 0.0);
    offset_y_ = this->declare_parameter<float>("y", 0.0);
    offset_z_ = this->declare_parameter<float>("z", 0.0);

    cout << "Offset x: " << offset_x_ << " Offset y: " << offset_y_ << " Offset z: " << offset_z_ << endl;

    string follower_position_topic = this->declare_parameter<string>("follower_position_topic", "");
    string x_topic = this->declare_parameter<string>("follower_offset_position_x_topic", "");
    string y_topic = this->declare_parameter<string>("follower_offset_position_y_topic", "");
    string z_topic = this->declare_parameter<string>("follower_offset_position_z_topic", "");
    string follower_offset_position_topic = this->declare_parameter<string>("follower_offset_position_topic", "");
    string og_target_topic = this->declare_parameter<string>("og_target_position_topic", "");

    // Create subscription and publisher
    point_sub_ = this->create_subscription<Point>(
        follower_position_topic, 10, std::bind(&RelativeTargetPositionPublisher::pointCallback, this, std::placeholders::_1));
    x_pub_ = this->create_publisher<Float32>(x_topic, 10);
    y_pub_ = this->create_publisher<Float32>(y_topic, 10);
    z_pub_ = this->create_publisher<Float32>(z_topic, 10);
    follower_offset_pub_ = this->create_publisher<Point>(follower_offset_position_topic, 10);
    og_target_pub_ = this->create_publisher<Point>(og_target_topic, 10);
}

void RelativeTargetPositionPublisher::pointCallback(const Point::SharedPtr msg)
{
    float x = msg->x - offset_x_;
    float y = msg->y - offset_y_;
    float z = msg->z - offset_z_;
    // if (toggle_) {
    //     Eigen::Vector3f current_relative_position(x, y, 0);
    //     Eigen::Vector3f instant_velocity = (current_relative_position - prev_relative_position_) / 0.5;
    //     if (instant_velocity[0] < 0) instant_velocity = -instant_velocity;
    //     cout << "Instant velocity: " << instant_velocity[0] << " " << instant_velocity[1] << " " << instant_velocity[2] << endl;
    //     // cout << "Previous relative position: " << prev_relative_position_[0] << " " << prev_relative_position_[1] << " " << prev_relative_position_[2] << endl;
    //     cout << "Current relative position: " << current_relative_position[0] << " " << current_relative_position[1] << " " << current_relative_position[2] << endl;
    //     float angle = calculateAngle(instant_velocity, -current_relative_position);
    //     cout << "Angle: " << angle << endl;
    //     if (abs(prev_angle_ - angle) < 0.3) {
    //         prev_angle_ = angle;
    //     }
    //     theta_pub_->publish(Float32().set__data(angle));
    //     prev_relative_position_ = current_relative_position;
    // }
    x_pub_->publish(Float32().set__data(x));
    y_pub_->publish(Float32().set__data(y));
    z_pub_->publish(Float32().set__data(z));

    Point offset_target;
    offset_target.x = x;
    offset_target.y = y;
    offset_target.z = z;
    follower_offset_pub_->publish(offset_target);
    
    Point og_target;
    og_target.x = offset_x_;
    og_target.y = offset_y_;
    og_target.z = offset_z_;
    og_target_pub_->publish(og_target);

    // toggle_ = !toggle_;
}

// float RelativeTargetPositionPublisher::calculateAngle(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2)
// {
//     Eigen::Vector3f vec1_norm = vec1.normalized();
//     Eigen::Vector3f vec2_norm = vec2.normalized();
//     Eigen::Vector3f cross_product = vec1_norm.cross(vec2_norm);
//     float cross_product_norm = cross_product.norm();
//     return asin(cross_product_norm) * (cross_product[2] > 0 ? 1 : -1);
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeTargetPositionPublisher>());
    rclcpp::shutdown();
    return 0;
}