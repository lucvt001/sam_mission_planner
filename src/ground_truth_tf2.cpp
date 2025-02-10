#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class GroundTruthTF2 : public rclcpp::Node
{
public:
    GroundTruthTF2() : Node("ground_truth_tf2")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<string>("follower_relative_gt_topic", "");
        string follower_relative_gt_topic = this->get_parameter("follower_relative_gt_topic").as_string();
        point_pub_ = this->create_publisher<Point>(follower_relative_gt_topic, 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&GroundTruthTF2::computeRelativeTransform, this));

        // Convert to NED coordinate system
        tf2::Matrix3x3 rotation_matrix(
            1, 0, 0,  // X -> X
            0, -1, 0,  // Y -> -Y
            0, 0, -1  // Z -> -Z
        );
        ned_transform_.setBasis(rotation_matrix);
    }

private:
    void computeRelativeTransform()
    {
        try
        {
            // Get the transform from leader base_link to leader modem
            TransformStamped leader_base_to_modem =
                tf_buffer_->lookupTransform("leader1/base_link_gt", "leader1/aft_transceiver_link_gt", tf2::TimePointZero);

            // Get the transform from follower base_link to follower modem
            TransformStamped follower_base_to_modem =
                tf_buffer_->lookupTransform("follower/base_link_gt", "follower/aft_transceiver_link_gt", tf2::TimePointZero);

            // Get the transform from leader base_link to follower base_link
            TransformStamped leader_base_to_follower_base =
                tf_buffer_->lookupTransform("leader1/base_link_gt", "follower/base_link_gt", tf2::TimePointZero);

            // Compute the transform from leader modem to follower modem
            tf2::Transform leader_base_to_modem_tf;
            tf2::fromMsg(leader_base_to_modem.transform, leader_base_to_modem_tf);

            tf2::Transform follower_base_to_modem_tf;
            tf2::fromMsg(follower_base_to_modem.transform, follower_base_to_modem_tf);

            tf2::Transform leader_base_to_follower_base_tf;
            tf2::fromMsg(leader_base_to_follower_base.transform, leader_base_to_follower_base_tf);

            tf2::Transform leader_modem_to_follower_modem_tf =
                leader_base_to_modem_tf.inverse() * leader_base_to_follower_base_tf * follower_base_to_modem_tf;

            leader_modem_to_follower_modem_tf = ned_transform_ * leader_modem_to_follower_modem_tf;

            Point leader_modem_to_follower_modem_point;
            leader_modem_to_follower_modem_point.x = leader_modem_to_follower_modem_tf.getOrigin().x();
            leader_modem_to_follower_modem_point.y = leader_modem_to_follower_modem_tf.getOrigin().y();
            leader_modem_to_follower_modem_point.z = leader_modem_to_follower_modem_tf.getOrigin().z();

            point_pub_->publish(leader_modem_to_follower_modem_point);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<Point>::SharedPtr point_pub_;

    tf2::Transform ned_transform_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruthTF2>());
    rclcpp::shutdown();
    return 0;
}