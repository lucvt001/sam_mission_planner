#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace std::chrono_literals;
using Odometry = nav_msgs::msg::Odometry;

class GroundTruth : public rclcpp::Node
{
public:
    GroundTruth()
        : Node("ground_truth")
    {
        this->declare_parameter<string>("leader1_odom_topic", "/leader1/odom");
        this->declare_parameter<string>("leader2_odom_topic", "/leader2/odom");
        this->declare_parameter<string>("follower_odom_topic", "/follower/odom");
        this->declare_parameter<string>("leader2_relative_topic", "/leader2/relative");
        this->declare_parameter<string>("follower_relative_topic", "/follower/relative");

        string leader1_odom_topic = this->get_parameter("leader1_odom_topic").as_string();
        string leader2_odom_topic = this->get_parameter("leader2_odom_topic").as_string();
        string follower_odom_topic = this->get_parameter("follower_odom_topic").as_string();
        string leader2_relative_topic = this->get_parameter("leader2_relative_topic").as_string();
        string follower_relative_topic = this->get_parameter("follower_relative_topic").as_string();

        leader1_sub_ = this->create_subscription<Odometry>(
            leader1_odom_topic, 10, std::bind(&GroundTruth::leader1Callback, this, std::placeholders::_1));
        leader2_sub_ = this->create_subscription<Odometry>(
            leader2_odom_topic, 10, std::bind(&GroundTruth::leader2Callback, this, std::placeholders::_1));
        follower_sub_ = this->create_subscription<Odometry>(
            follower_odom_topic, 10, std::bind(&GroundTruth::followerCallback, this, std::placeholders::_1));

        leader2_pub_ = this->create_publisher<Odometry>(leader2_relative_topic, 10);
        follower_pub_ = this->create_publisher<Odometry>(follower_relative_topic, 10);

        // Convert to NED coordinate system
        tf2::Matrix3x3 rotation_matrix(
            1, 0, 0,  // X -> X
            0, -1, 0,  // Y -> -Y
            0, 0, -1  // Z -> -Z
        );
        ned_transform_.setBasis(rotation_matrix);
    }

private:
    void leader1Callback(const Odometry::SharedPtr msg)
    {
        leader1_pose_ = msg->pose.pose;
    }

    void leader2Callback(const Odometry::SharedPtr msg)
    {
        if (!leader1_pose_)
        {
            return;
        }

        Odometry relative_odom;
        relative_odom.header = msg->header;
        relative_odom.pose.pose = getRelativePose(msg->pose.pose, *leader1_pose_);
        leader2_pub_->publish(relative_odom);
    }

    void followerCallback(const Odometry::SharedPtr msg)
    {
        if (!leader1_pose_)
        {
            return;
        }

        Odometry relative_odom;
        relative_odom.header = msg->header;
        relative_odom.pose.pose = getRelativePose(msg->pose.pose, *leader1_pose_);
        follower_pub_->publish(relative_odom);
    }

    geometry_msgs::msg::Pose getRelativePose(const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Pose &origin)
    {
        tf2::Transform tf_pose, tf_origin, tf_relative;
        tf2::fromMsg(pose, tf_pose);
        tf2::fromMsg(origin, tf_origin);

        tf_relative = tf_origin.inverse() * tf_pose;
        tf_relative = ned_transform_ * tf_relative;

        geometry_msgs::msg::Pose relative_pose;
        tf2::toMsg(tf_relative, relative_pose);
        return relative_pose;
    }

    rclcpp::Subscription<Odometry>::SharedPtr leader1_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr leader2_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr follower_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr leader2_pub_;
    rclcpp::Publisher<Odometry>::SharedPtr follower_pub_;
    std::optional<geometry_msgs::msg::Pose> leader1_pose_;

    tf2::Transform ned_transform_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundTruth>());
    rclcpp::shutdown();
    return 0;
}