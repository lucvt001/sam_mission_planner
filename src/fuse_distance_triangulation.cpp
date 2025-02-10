#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using namespace std;
using Float32 = std_msgs::msg::Float32;
using FluidPressure = sensor_msgs::msg::FluidPressure;
using Point = geometry_msgs::msg::Point;

class FuseDistanceTriangulation : public rclcpp::Node
{
public:
    FuseDistanceTriangulation() : Node("fuse_distance_triangulation")
    {
        string leader1_distance_topic = this->declare_parameter<string>("leader1_distance_topic", "");
        string leader2_distance_topic = this->declare_parameter<string>("leader2_distance_topic", "");
        string follower_position_topic = this->declare_parameter<string>("follower_relative_position_triangulation_topic", "");
        string follower_depth_topic = this->declare_parameter<string>("follower_depth_topic", "");
        leaders_distance_ = this->declare_parameter<double>("leaders_distance", 0.0);

        // string leader1_distance_topic = this->get_parameter("leader1_distance_topic").as_string();
        // string leader2_distance_topic = this->get_parameter("leader2_distance_topic").as_string();
        // string follower_position_topic = this->get_parameter("follower_relative_position_triangulation_topic").as_string();
        // string follower_depth_topic = this->get_parameter("follower_depth_topic").as_string();
        // leaders_distance_ = this->get_parameter("leaders_distance").as_double();

        leader1_sub_ = this->create_subscription<Float32>(
            leader1_distance_topic, 10, std::bind(&FuseDistanceTriangulation::leader1Callback, this, std::placeholders::_1));
        leader2_sub_ = this->create_subscription<Float32>(
            leader2_distance_topic, 10, std::bind(&FuseDistanceTriangulation::leader2Callback, this, std::placeholders::_1));
        follower_depth_sub_ = this->create_subscription<FluidPressure>(
            follower_depth_topic, 10, std::bind(&FuseDistanceTriangulation::followerDepthCallback, this, std::placeholders::_1));

        follower_position_pub_ = this->create_publisher<Point>(follower_position_topic, 10);
        follower_position_pub_x_ = this->create_publisher<Float32>(follower_position_topic + "_x", 10);
        follower_position_pub_y_ = this->create_publisher<Float32>(follower_position_topic + "_y", 10);
        follower_position_pub_z_ = this->create_publisher<Float32>(follower_position_topic + "_z", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void leader1Callback(const Float32::SharedPtr msg)
    {
        leader1_distance_ = msg->data;
        // triangulatePosition();
    }

    void leader2Callback(const Float32::SharedPtr msg)
    {
        leader2_distance_ = msg->data;
        triangulatePosition();
    }

    void followerDepthCallback(const FluidPressure::SharedPtr msg)
    {
        double pressure_depth = msg->fluid_pressure / 9806.65; // Convert pressure to depth in meters

        try
        {
            // Get the transform from the pressure sensor frame to the follower modem frame
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("follower/pressure_link_gt", "follower/aft_transceiver_link_gt", tf2::TimePointZero);

            tf2::Transform transform;
            tf2::fromMsg(transform_stamped.transform, transform);

            // Calculate the depth of the modem
            follower_depth_ = pressure_depth - transform.getOrigin().z();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void triangulatePosition()
    {
        if (leader1_distance_ < 0 || leader2_distance_ < 0)
        {
            return;
        }

        // Assuming leader2 is at (d, 0) relative to leader1 at (0, 0)
        double d = leaders_distance_; // Distance between leader1 and leader2 (example value)
        double d1 = std::sqrt(std::pow(leader1_distance_, 2) - std::pow(follower_depth_, 2));
        double d2 = std::sqrt(std::pow(leader2_distance_, 2) - std::pow(follower_depth_, 2));

        // Imagine a triangle with three sides: d1, d2, and d
        // Using the Law of Cosines to find the angle between d and d1
        double cosine = (std::pow(d1, 2) + std::pow(d, 2) - std::pow(d2, 2)) / (2 * d1 * d);
        double angle = std::acos(cosine);
        double sine = std::sin(angle);

        Point follower_position;
        follower_position.x = - d1 * sine;
        follower_position.y = d1 * cosine;
        follower_position.z = follower_depth_;
        std::cout << "follower position: (" << follower_position.x << ", " << follower_position.y << ", " << follower_position.z << ")" << std::endl;

        follower_position_pub_->publish(follower_position);
        follower_position_pub_x_->publish(Float32().set__data(follower_position.x));
        follower_position_pub_y_->publish(Float32().set__data(follower_position.y));
        follower_position_pub_z_->publish(Float32().set__data(follower_position.z));
    }

    rclcpp::Subscription<Float32>::SharedPtr leader1_sub_;
    rclcpp::Subscription<Float32>::SharedPtr leader2_sub_;
    rclcpp::Subscription<FluidPressure>::SharedPtr follower_depth_sub_;
    rclcpp::Publisher<Point>::SharedPtr follower_position_pub_;
    rclcpp::Publisher<Float32>::SharedPtr follower_position_pub_x_;
    rclcpp::Publisher<Float32>::SharedPtr follower_position_pub_y_;
    rclcpp::Publisher<Float32>::SharedPtr follower_position_pub_z_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double leader1_distance_ = -1.0;
    double leader2_distance_ = -1.0;
    double follower_depth_;
    double leaders_distance_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuseDistanceTriangulation>());
    rclcpp::shutdown();
    return 0;
}