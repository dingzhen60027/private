#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <QObject>
#include <QThread>
#include <QImage>
#include <QPainter>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_interface_robosoccer/msg/pose_robots.hpp"
#include "message_interface_robosoccer/msg/point_calibration.hpp"
#include "message_interface_robosoccer/msg/array_threshold.hpp"
#include "message_interface_robosoccer/msg/decision_var.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "globalVar.h"

class rclcomm : public QThread
{
    Q_OBJECT
public:
    rclcomm();
    //void publish_topic(int count);
    //void timer_callback();
    //void recv_callback(const std_msgs::msg::Int32::SharedPtr msg);
    
    void recv_callback_i1(const sensor_msgs::msg::Image::SharedPtr msg);
    void recv_callback_i2(const sensor_msgs::msg::Image::SharedPtr msg);
    void recv_callback_p(const message_interface_robosoccer::msg::PoseRobots::SharedPtr msg);
    
    void drawRobotsAndBall(message_interface_robosoccer::msg::PoseRobots Poses);

    bool isToStop;

    unsigned long Img_count;

protected:
    void run();

private:
    //rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
    //std_msgs::msg::Int32 pub_msg;
    //rclcpp::TimerBase::SharedPtr _timer;
    //rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_image_before;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_image_after;
    rclcpp::Subscription<message_interface_robosoccer::msg::PoseRobots>::SharedPtr _subscription_pose;
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher_camCtrl;
    rclcpp::Publisher<message_interface_robosoccer::msg::PointCalibration>::SharedPtr _publisher_boundSet;
    rclcpp::Publisher<message_interface_robosoccer::msg::ArrayThreshold>::SharedPtr _publisher_colorSet;
    rclcpp::Publisher<message_interface_robosoccer::msg::DecisionVar>::SharedPtr _publisher_decisionVar;

    std::shared_ptr<rclcpp::Node> node;  //节点，作为 rclcomm类 的 成员 来操作，将原本节点内需要定义的成员放在rclcomm类下，节点只需要负责初始化发布者/订阅者和其运行即可
    
    int encoding2mat_type(std::string encoding);
    
signals:
    //void TopicData(QString);
    void TopicData_image_before(QImage);
    void TopicData_image_after(QImage);
    void sendSimuGround(QPixmap);

public slots:
    void open_camera();
    void close_camera();
    void start_acq();
    void quit_acq();
    void BoundsetPoint(Point* calipoint_);
    void ColorsetThreshold(int* hsithreshold_);

    void DecisionsetVar();
};
#endif // RCLCOMM_H
