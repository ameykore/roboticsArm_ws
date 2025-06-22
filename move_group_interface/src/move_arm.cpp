
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MoveArm : public rclcpp::Node
{
public:
    MoveArm() : Node("MoveArm_node") 
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("arm_publish", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MoveArm::timerCallback, this));
    }
private:
    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " ;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), message.data.c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveArm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}