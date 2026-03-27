#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFrameFixer : public rclcpp::Node
{
public:
    ScanFrameFixer()
    : Node("scan_frame_fixer")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ScanFrameFixer::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan_fixed", 10
        );
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto fixed_msg = *msg;
        fixed_msg.header.frame_id = "lidar_link";

        pub_->publish(fixed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFrameFixer>());
    rclcpp::shutdown();
    return 0;
}