#ifndef RCLCOMM2_H
#define RCLCOMM2_H
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPainter>
#include <QDebug>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "globalVar.h"

class rclcomm2 : public QThread
{
    Q_OBJECT
public:
    rclcomm2();

    void recv_callback_i2(const sensor_msgs::msg::Image::SharedPtr msg);

    bool isToStop;

protected:
    void run();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_image_after;

    std::shared_ptr<rclcpp::Node> node;  //节点，作为 rclcomm类 的 成员 来操作，将原本节点内需要定义的成员放在rclcomm类下，节点只需要负责初始化发布者/订阅者和其运行即可
    
    int encoding2mat_type(std::string encoding);
    
signals:
    void TopicData_image_after(QImage);

public slots:
    
};
#endif // RCLCOMM2_H
