#ifndef RCLCOMM4_H
#define RCLCOMM4_H
#include <QObject>
#include <QThread>
#include <QDebug>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "message_interface_robosoccer/msg/point_calibration.hpp"
#include "message_interface_robosoccer/msg/array_threshold.hpp"
#include "message_interface_robosoccer/msg/decision_var.hpp"

#include "globalVar.h"

class rclcomm4 : public QThread
{
    Q_OBJECT
public:
    rclcomm4();
    
    bool isToStop;

protected:
    void run();

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher_camCtrl;
    rclcpp::Publisher<message_interface_robosoccer::msg::PointCalibration>::SharedPtr _publisher_boundSet;
    rclcpp::Publisher<message_interface_robosoccer::msg::ArrayThreshold>::SharedPtr _publisher_colorSet;
    rclcpp::Publisher<message_interface_robosoccer::msg::DecisionVar>::SharedPtr _publisher_decisionVar;

    std::shared_ptr<rclcpp::Node> node;  //节点，作为 rclcomm类 的 成员 来操作，将原本节点内需要定义的成员放在rclcomm类下，节点只需要负责初始化发布者/订阅者和其运行即可
    
signals:
    
public slots:
    void open_camera();
    void close_camera();
    void start_acq();
    void quit_acq();
    void BoundsetPoint(Point* calipoint_);
    void ColorsetThreshold(int* hsithreshold_);

    void DecisionsetVar();
};
#endif // RCLCOMM4_H
