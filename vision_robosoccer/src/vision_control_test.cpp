#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;//可以使用数字加单位的表示法，如：1ms，需要 C++14 标准

class Vision_Control : public rclcpp::Node
{
public:
  Vision_Control(std::string name) : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "%s已创建!", name.c_str());
    
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("camera_control", 1);
    timer_ = this->create_wall_timer(10ms, std::bind(&Vision_Control::publishMessage, this));
  }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void publishMessage()
    {
        auto message = std_msgs::msg::Int32();

        int nStartKey = getchar();
        switch (nStartKey)
        {
        //打开相机
        case 'O':
        case 'o':
            message.data = 0; // 设置消息的值

            RCLCPP_INFO(this->get_logger(), "打开相机......");
            publisher_->publish(message);
            break;
        //关闭相机
        case 'C':
        case 'c':
            message.data = 1; // 设置消息的值

            RCLCPP_INFO(this->get_logger(), "关闭相机......");
            publisher_->publish(message);
            break;
        //开始采集
        case 'S':
        case 's':
            message.data = 2; // 设置消息的值

            RCLCPP_INFO(this->get_logger(), "开始采集......");
            publisher_->publish(message);
            break;
        //停止采集
        case 'Q':
        case 'q':
            message.data = 3; // 设置消息的值

            RCLCPP_INFO(this->get_logger(), "停止采集......");
            publisher_->publish(message);
            break;

        default:
            break;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Vision_Control>("vision_control");
    //rclcpp::spin(node);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
