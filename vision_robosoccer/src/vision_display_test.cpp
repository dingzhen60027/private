#define _USE_MATH_DEFINES
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "message_interface_robosoccer/msg/image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_interface_robosoccer/msg/pose.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class Vision_Display : public rclcpp::Node
{
public:
    Vision_Display(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s已创建!", name.c_str());
        
        count=0;
        
        robot_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("robot_image",1, std::bind(&Vision_Display::image_sub_callback, this, std::placeholders::_1));  //"robot_image_processed"
        robot_pose_sub_ = this->create_subscription<message_interface_robosoccer::msg::Pose>("robot_pose",1, std::bind(&Vision_Display::pose_sub_callback, this, std::placeholders::_1));
    }
    
private:
    unsigned long count;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robot_image_sub_;
    rclcpp::Subscription<message_interface_robosoccer::msg::Pose>::SharedPtr robot_pose_sub_;
    
    int encoding2mat_type(std::string encoding)
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
    
    void image_sub_callback(sensor_msgs::msg::Image::SharedPtr image_sub_)
    {
        /*
        // 将ROS图像消息转换为OpenCV图像格式
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(image_sub_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }
        // 在这里进行图像处理操作
        cv::Mat image5processed = cv_ptr->image;
        //...
        */
        
        //RCLCPP_INFO(this->get_logger(), "我接收到了一帧图像!");
        cv::Mat image5processed(
          image_sub_->height, image_sub_->width,
          encoding2mat_type(image_sub_->encoding),
          const_cast<unsigned char *>(image_sub_->data.data()));
        if (image5processed.empty())
        {
            RCLCPP_INFO(this->get_logger(), "图像接收错误!!");
        }
        
        //cv::imshow("Image_to_Process", image4process);
        //cv::waitKey(0);
        
        //*The place to display images
        count++;
        if (count > 4000000000)
            count = 1;
        
        RCLCPP_INFO(this->get_logger(),"第%ld次收到图像,高度=%d,宽度=%d",count, image5processed.rows,image5processed.cols);
        cv::imshow("Image_Processed", image5processed);
        cv::waitKey(10);
    }
    void pose_sub_callback(message_interface_robosoccer::msg::Pose::SharedPtr pose_sub_)
    {
        //RCLCPP_INFO(this->get_logger(), "我接收到了一组位置!");
        RCLCPP_INFO(this->get_logger(), "Pose: (%f, %f, %f)", pose_sub_->x, pose_sub_->y, pose_sub_->theta);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Vision_Display>("vision_display");
    //rclcpp::spin(node);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
