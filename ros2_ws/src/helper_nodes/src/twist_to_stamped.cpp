#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToStamped : public rclcpp::Node
{
public:
    TwistToStamped()
    : Node("twist_to_stamped")
    {
        // Parameters (VERY IMPORTANT for flexibility)
        input_topic_ = this->declare_parameter("input_topic", "/cmd_vel_smoothed");
        output_topic_ = this->declare_parameter("output_topic", "/cmd_vel_stamped");

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_topic_, 10,
            std::bind(&TwistToStamped::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            output_topic_, 10
        );

        RCLCPP_INFO(this->get_logger(), "TwistToStamped node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped stamped_msg;

        stamped_msg.header.stamp = this->get_clock()->now();

        // Optional: leave empty unless required
        stamped_msg.header.frame_id = "";

        stamped_msg.twist = *msg;

        pub_->publish(stamped_msg);
    }

    std::string input_topic_;
    std::string output_topic_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToStamped>());
    rclcpp::shutdown();
    return 0;
}