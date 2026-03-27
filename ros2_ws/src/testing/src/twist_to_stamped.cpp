#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToStamped : public rclcpp::Node
{
public:
    TwistToStamped()
    : Node("twist_to_stamped")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TwistToStamped::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_drive_controller/cmd_vel", 10    // ← revert back
        );
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped stamped_msg;

        stamped_msg.header.stamp = this->get_clock()->now();
        stamped_msg.header.frame_id = "base_link";  // important

        stamped_msg.twist = *msg;

        pub_->publish(stamped_msg);
    }

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
