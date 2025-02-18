#include <rclcpp/rclcpp.hpp>
#include <smarc_msgs/msg/string_stamped.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;
using StringStamped = smarc_msgs::msg::StringStamped;

class PingSynchronizer : public rclcpp::Node
{
public:
    PingSynchronizer() : Node("ping_synchronizer"), toggle_(true)
    {
        this->declare_parameter<int>("interval_ms", 500);
        int interval_ms = this->get_parameter("interval_ms").as_int();

        this->declare_parameter<string>("leader1_acoustic_topic", "");
        string leader1_acoustic_topic = this->get_parameter("leader1_acoustic_topic").as_string();

        this->declare_parameter<string>("leader2_acoustic_topic", "");
        string leader2_acoustic_topic = this->get_parameter("leader2_acoustic_topic").as_string();

        publisher1_ = this->create_publisher<StringStamped>(leader1_acoustic_topic, 10);
        publisher2_ = this->create_publisher<StringStamped>(leader2_acoustic_topic, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms), std::bind(&PingSynchronizer::publishMessage, this));
    }

private:
    void publishMessage()
    {
        auto message = StringStamped();
        if (toggle_)
        {
            message.data = "1";
            publisher1_->publish(message);
        }
        else
        {
            message.data = "2";
            publisher2_->publish(message);
        }
        toggle_ = !toggle_;
    }

    rclcpp::Publisher<StringStamped>::SharedPtr publisher1_;
    rclcpp::Publisher<StringStamped>::SharedPtr publisher2_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool toggle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingSynchronizer>());
    rclcpp::shutdown();
    return 0;
}