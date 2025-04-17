#include "rclcomm2.h"

rclcomm2::rclcomm2()
{
    node=rclcpp::Node::make_shared("gui_robosoccer_subImgAfter");
    RCLCPP_INFO(node->get_logger(), "gui_robosoccer_subImgAfter已创建!");

    isToStop = 0;

    //窗口抖动可尝试接收队列长度设为1或者降低帧率
    _subscription_image_after = node->create_subscription<sensor_msgs::msg::Image>("robot_image_processed", 1, std::bind(&rclcomm2::recv_callback_i2, this, std::placeholders::_1));

    this->start();
}

void rclcomm2::run()
{
    qDebug() << "gui_robosoccer_subImgAfter thread:" << QThread::currentThreadId();
    while (rclcpp::ok() && this->isToStop == 0)
    {
        rclcpp::spin_some(node);
    }
    quit();
}

int rclcomm2::encoding2mat_type(std::string encoding)
{
    if (encoding == "mono8") 
    {
        return CV_8UC1;
    } 
    else if (encoding == "bgr8")  //"rgb8"
    {
        return CV_8UC3;
    }
    else if (encoding == "mono16") 
    {
        return CV_16SC1;
    } 
    else if (encoding == "rgba8") 
    {
        return CV_8UC4;
    }
    else 
    {
        return CV_8UC3;
    }
}
void rclcomm2::recv_callback_i2(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (isFinishDisp_2)
    {
        isFinishDisp_2 = false;
        QImage robot_image_after;
        QImage Image_Disp_after;
        cv::Mat msg_cvImage(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()));
        if (msg_cvImage.empty() || msg_cvImage.cols != IMAGE_WIDTH || msg_cvImage.rows != IMAGE_HEIGHT)
        {
            //RCLCPP_INFO(this->node->get_logger(), "处理图像接收错误!!丢失一帧图像!!");
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);//sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_INFO(this->node->get_logger(), "图像转换错误:%s!!丢失一帧图像!!", e.what());
                return;
            }
            robot_image_after = QImage(cv_ptr->image.data,msg->width,msg->height,msg->step,QImage::Format_BGR888);
            Image_Disp_after = robot_image_after.scaled(SIMULATION_WIDTH, SIMULATION_HEIGHT, Qt::IgnoreAspectRatio, Qt::FastTransformation);
            emit TopicData_image_after(Image_Disp_after);
        }
        else
        {
            robot_image_after = QImage((const uchar*)(msg_cvImage.data), msg_cvImage.cols, msg_cvImage.rows, msg_cvImage.step, QImage::Format_BGR888);
            Image_Disp_after = robot_image_after.scaled(SIMULATION_WIDTH, SIMULATION_HEIGHT, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            emit TopicData_image_after(Image_Disp_after);
        }
    }
    return;
}

