#ifndef RCLCOMM3_H
#define RCLCOMM3_H
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPainter>
#include <QDebug>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "message_interface_robosoccer/msg/pose_robots.hpp"

#include "globalVar.h"

class rclcomm3 : public QThread
{
    Q_OBJECT
public:
    rclcomm3();

    void recv_callback_p(const message_interface_robosoccer::msg::PoseRobots::SharedPtr msg);
    
    void drawRobotsAndBall(message_interface_robosoccer::msg::PoseRobots Poses);

    bool isToStop;

protected:
    void run();

private:
    rclcpp::Subscription<message_interface_robosoccer::msg::PoseRobots>::SharedPtr _subscription_pose;

    std::shared_ptr<rclcpp::Node> node;  //节点，作为 rclcomm类 的 成员 来操作，将原本节点内需要定义的成员放在rclcomm类下，节点只需要负责初始化发布者/订阅者和其运行即可
    
signals:
    void sendSimuGround(QPixmap);

public slots:
    
};
#endif // RCLCOMM3_H
