#include <rclcpp/rclcpp.hpp>
#include <smarc_msgs/msg/string_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/float32.hpp>

#define SPEED_OF_SOUND 1500.0

using Float32 = std_msgs::msg::Float32;
using StringStamped = smarc_msgs::msg::StringStamped;
using namespace std;

class SoundDistanceCalculator : public rclcpp::Node
{
public:
    SoundDistanceCalculator() : Node("sound_distance_calculator")
    {
        this->declare_parameter<string>("follower_acoustic_topic", "");
        string follower_acoustic_topic = this->get_parameter("follower_acoustic_topic").as_string();

        this->declare_parameter<string>("leader1_distance_topic", "");
        string leader1_distance_topic = this->get_parameter("leader1_distance_topic").as_string();

        this->declare_parameter<string>("leader2_distance_topic", "");
        string leader2_distance_topic = this->get_parameter("leader2_distance_topic").as_string();

        sub_ = this->create_subscription<StringStamped>(
            follower_acoustic_topic, 10, std::bind(&SoundDistanceCalculator::topicCallback, this, std::placeholders::_1));

        leader1_distance_pub_ = this->create_publisher<Float32>(leader1_distance_topic, 10);
        leader2_distance_pub_ = this->create_publisher<Float32>(leader2_distance_topic, 10);
    }

private:
    void topicCallback(const StringStamped::SharedPtr msg)
    {
        string acoustic_data_source = msg->data;
        rclcpp::Time time_sent = msg->time_sent;
        rclcpp::Time time_received = msg->time_received;

        auto time_diff = time_received - time_sent;
        double time_diff_sec = time_diff.seconds();

        double distance = SPEED_OF_SOUND * time_diff_sec;
        auto message = Float32();
        if (acoustic_data_source == "1")
        {
            message.data = distance;
            leader1_distance_pub_->publish(message);
        }
        else if (acoustic_data_source == "2")
        {
            message.data = distance;
            leader2_distance_pub_->publish(message);
        }

        // RCLCPP_INFO(this->get_logger(), "Distance sound traveled: %.2f meters", distance);
    }

    rclcpp::Subscription<StringStamped>::SharedPtr sub_;
    rclcpp::Publisher<Float32>::SharedPtr leader1_distance_pub_;
    rclcpp::Publisher<Float32>::SharedPtr leader2_distance_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoundDistanceCalculator>());
    rclcpp::shutdown();
    return 0;
}