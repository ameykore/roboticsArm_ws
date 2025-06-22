#include "rclcpp/rclcpp.hpp"

class MoveArm : public rclcpp::Node
{
public:
    MoveArm() : Node("MoveArm_node") 
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MoveArm::timerCallback, this));
    }
private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
    }
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveArm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}