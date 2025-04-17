#include "rclcomm1.h"

rclcomm1::rclcomm1()
{
    node=rclcpp::Node::make_shared("gui_robosoccer_subImgBefore");
    RCLCPP_INFO(node->get_logger(), "gui_robosoccer_subImgBefore已创建!");

    isToStop = 0;

    Img_count = 0;

    //窗口抖动可尝试接收队列长度设为1或者降低帧率
    _subscription_image_before = node->create_subscription<sensor_msgs::msg::Image>("robot_image", 1, std::bind(&rclcomm1::recv_callback_i1, this, std::placeholders::_1));

    this->start();
}

void rclcomm1::run()
{
    qDebug() << "gui_robosoccer_subImgBefore thread:" << QThread::currentThreadId();
    while (rclcpp::ok() && this->isToStop == 0)
    {
        rclcpp::spin_some(node);
    }
    quit();
}

int rclcomm1::encoding2mat_type(std::string encoding)
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
void rclcomm1::recv_callback_i1(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (isFinishDisp_1)
    {
        isFinishDisp_1 = false;
        QImage robot_image_before;
        QImage Image_Disp_before;
        cv::Mat msg_cvImage(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()));
        //图片尺寸修改后图像(1920x1200)*0.5
        if (msg_cvImage.empty() || msg_cvImage.cols != (IMAGE_WIDTH*0.5) || msg_cvImage.rows != (IMAGE_HEIGHT*0.5))
        {
            //RCLCPP_INFO(this->node->get_logger(), "原始图像接收错误!!丢失一帧图像!!");
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
            robot_image_before = QImage(cv_ptr->image.data, msg->width, msg->height, msg->step, QImage::Format_BGR888);
            Image_Disp_before = robot_image_before.scaled(DISP_WIDTH, DISP_HEIGHT, Qt::IgnoreAspectRatio, Qt::FastTransformation);
            emit TopicData_image_before(Image_Disp_before);
        }
        else
        {
            robot_image_before = QImage((const uchar*)(msg_cvImage.data), msg_cvImage.cols, msg_cvImage.rows, msg_cvImage.step, QImage::Format_BGR888);
            Image_Disp_before = robot_image_before.copy();//.scaled(DISP_WIDTH, DISP_HEIGHT, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);//.copy();
            emit TopicData_image_before(Image_Disp_before);
        }
    }
    Img_count++;
    if (Img_count > 4000000000)
        Img_count = 1;
    RCLCPP_INFO(this->node->get_logger(), "第%ld次收到图像,高度=%d,宽度=%d,图片ID=%s",Img_count, msg->height, msg->width, msg->header.frame_id.c_str());
    return;
}

