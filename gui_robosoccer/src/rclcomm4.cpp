#include "rclcomm4.h"

rclcomm4::rclcomm4()
{
    node=rclcpp::Node::make_shared("gui_robosoccer_publisher");
    RCLCPP_INFO(node->get_logger(), "gui_robosoccer_publisher已创建!");

    isToStop = 0;

    _publisher_camCtrl = node->create_publisher<std_msgs::msg::Int32>("camera_control", 1);
    _publisher_boundSet = node->create_publisher<message_interface_robosoccer::msg::PointCalibration>("camera_calibration_point", 1);
    _publisher_colorSet = node->create_publisher<message_interface_robosoccer::msg::ArrayThreshold>("robot_colorthreshold_array", 1);
    _publisher_decisionVar = node->create_publisher<message_interface_robosoccer::msg::DecisionVar>("decision_var", 1);

    this->start();
}

void rclcomm4::run()
{
    qDebug() << "gui_robosoccer_publisher thread:" << QThread::currentThreadId();
    while (rclcpp::ok() && this->isToStop == 0)
    {
        rclcpp::spin_some(node);
    }
    quit();
}

void rclcomm4::open_camera()
{
    auto message = std_msgs::msg::Int32();
    message.data = 0;
    _publisher_camCtrl->publish(message);
}

void rclcomm4::close_camera()
{
    auto message = std_msgs::msg::Int32();
    message.data = 1;
    _publisher_camCtrl->publish(message);
}

void rclcomm4::start_acq()
{
    auto message = std_msgs::msg::Int32();
    message.data = 2;
    _publisher_camCtrl->publish(message);
}

void rclcomm4::quit_acq()
{
    auto message = std_msgs::msg::Int32();
    message.data = 3;
    _publisher_camCtrl->publish(message);
}

void rclcomm4::BoundsetPoint(Point* calipoint_)
{
    auto message = message_interface_robosoccer::msg::PointCalibration();
    for (int i=0; i<centerNum; i++)
    {
        message.point[i].x = calipoint_[i].x;
        message.point[i].y = calipoint_[i].y;
    }
    _publisher_boundSet->publish(message);
    RCLCPP_INFO(this->node->get_logger(), "已发布标定点!");
}

void rclcomm4::ColorsetThreshold(int* hsithreshold_)
{
    auto message = message_interface_robosoccer::msg::ArrayThreshold();
    for (int i=0; i<8; i++)
    {
        for (int j=0; j<6; j++)
        {
            message.hsithreshold[i*6+j] = hsithreshold_[i*6+j];
        }
    }
    _publisher_colorSet->publish(message);
    RCLCPP_INFO(this->node->get_logger(), "已发布颜色阈值!");
}

void rclcomm4::DecisionsetVar()
{
    auto message = message_interface_robosoccer::msg::DecisionVar();
    message.decisionvar[0] = isAttack;
    message.decisionvar[1] = isRightArea;
    message.decisionvar[2] = StartMode;
    message.decisionvar[3] = isBackInitPose;
    message.decisionvar[4] = isReInit;
    message.decisionvar[5] = isStartGame;
    _publisher_decisionVar->publish(message);
    RCLCPP_INFO(this->node->get_logger(), "已发布策略变量!");
}
