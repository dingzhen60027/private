#include <cstdio>
#include <chrono>
#include <memory>
#include "GxIAPI.h"
#include "DxImageProc.h"
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace cv;
using namespace std::chrono_literals;//可以使用数字加单位的表示法，如：1ms，需要 C++14 标准


#define ACQ_BUFFER_NUM          10               ///< Acquisition Buffer Qty.  //50
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success

//Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        return false;                   \
    }
    


class Vision : public rclcpp::Node
{
public:
    Vision(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s已创建!", name.c_str());
        
        // 创建一个发布器
        publisher_=this->create_publisher<sensor_msgs::msg::Image>("robot_image",1);
        //创建一个定时器，并指定回调函数
        timer_=this->create_wall_timer(10ms,std::bind(&Vision::publishImage,this));  //10ms-100帧/s,20ms-50帧/s,实际帧率略小，因为ROS2传输较大的图片消息会产生延迟，可考虑图片压缩传输(暂未实现)

        // 创建一个订阅器，并指定回调函数
        subscription_ = this->create_subscription<std_msgs::msg::Int32>("camera_control", 1, std::bind(&Vision::controlcallback, this, std::placeholders::_1));

        //image_=cv::imread("/home/mayin/picture/sunflower.jpg");
        //resize(image_,image_,Size(),0.25,0.25);

        img_counter=0;
        bPub_=false;
        bInit_=false;
        bOpened_=false;
    }

    //初始化大恒环境
    bool InitDahengEnv()
    {
        try
        {
            GX_STATUS emStatus = GX_STATUS_SUCCESS;

            //uint32_t ui32DeviceNum = 0;

            // Initialize libary
            emStatus = GXInitLib();
            if (emStatus != GX_STATUS_SUCCESS)
            {
                //GetErrorString(emStatus);
                //return emStatus;
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"大恒相机库环境初始化完成");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }

    //打开相机
    bool OpenCamera()
    {
        try
        {
            GX_STATUS emStatus = GX_STATUS_SUCCESS;

            uint32_t ui32DeviceNum = 0;

            bInit_=InitDahengEnv();

            if (bInit_)
            {
                // Get device enumerated number
                emStatus = GXUpdateDeviceList(&ui32DeviceNum, 1000);
                if (emStatus != GX_STATUS_SUCCESS)
                {
                    GetErrorString(emStatus);
                    GXCloseLib();
                    return false;
                }

                // If no device found, app exit
                if (ui32DeviceNum <= 0)
                {
                    //printf("<No device found>\n");
                    GXCloseLib();
                    return false;
                }

                // Open first device enumerated
                emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
                if (emStatus != GX_STATUS_SUCCESS)
                {
                    GetErrorString(emStatus);
                    GXCloseLib();
                    return false;
                }

                // Get the type of Bayer conversion. whether is a color camera.
                emStatus = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
                if (emStatus != GX_STATUS_SUCCESS)
                {
                    GetErrorString(emStatus);
                    GXCloseDevice(g_hDevice);
                    g_hDevice = NULL;
                    GXCloseLib();
                    return false;
                }
                if (g_bColorFilter)
                {
                    emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
                    if (emStatus != GX_STATUS_SUCCESS)
                    {
                        GetErrorString(emStatus);
                        GXCloseDevice(g_hDevice);
                        g_hDevice = NULL;
                        GXCloseLib();
                        return false;
                    }
                }

                emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
                if (emStatus != GX_STATUS_SUCCESS)
                {
                    GetErrorString(emStatus);
                    GXCloseDevice(g_hDevice);
                    g_hDevice = NULL;
                    GXCloseLib();
                    return false;
                }

                // Set acquisition mode
                emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
                GX_VERIFY_EXIT(emStatus);

                // Set trigger mode
                emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
                GX_VERIFY_EXIT(emStatus);
                
                // Set buffer quantity of acquisition queue
                uint64_t nBufferNum = ACQ_BUFFER_NUM;
                emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
                GX_VERIFY_EXIT(emStatus);

                bool bStreamTransferSize = false;
                emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
                GX_VERIFY_EXIT(emStatus);

                if (bStreamTransferSize)
                {
                    // Set size of data transfer block
                    emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
                    GX_VERIFY_EXIT(emStatus);
                }

                bool bStreamTransferNumberUrb = false;
                emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
                GX_VERIFY_EXIT(emStatus);

                if (bStreamTransferNumberUrb)
                {
                    // Set qty. of data transfer block
                    emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
                    GX_VERIFY_EXIT(emStatus);
                }

                // Set Balance White Mode : Continuous
                emStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
                GX_VERIFY_EXIT(emStatus);

                
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        return true;
        
    }

    //关闭相机
    bool CloseCamera()
    {
        try
        {
            GX_STATUS emStatus = GX_STATUS_SUCCESS;            

            // Close device
            emStatus = GXCloseDevice(g_hDevice);
            if (emStatus != GX_STATUS_SUCCESS)
            {
                GetErrorString(emStatus);
                g_hDevice = NULL;
                GXCloseLib();
                return emStatus;
            }

            // Release libary
            emStatus = GXCloseLib();
            if (emStatus != GX_STATUS_SUCCESS)
            {
                GetErrorString(emStatus);
                return emStatus;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }

    //开始采集
    bool StartAcq()
    {
        GX_STATUS emStatus = GX_STATUS_SUCCESS;
        try
        {
            if(bOpened_)
            {
                // Allocate the memory for pixel format transform
                PreForAcquisition();

                // Device start acquisition
                emStatus = GXStreamOn(g_hDevice);
                if (emStatus != GX_STATUS_SUCCESS)
                {
                    // Release the memory allocated
                    UnPreForAcquisition();
                    GX_VERIFY_EXIT(emStatus);
                }
            }
            else
            {
                return false;
            }            
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }

    //停止采集
    bool StopAcq()
    {
        GX_STATUS emStatus = GX_STATUS_SUCCESS;
        try
        {

            // Device stop acquisition
            emStatus = GXStreamOff(g_hDevice);
            if (emStatus != GX_STATUS_SUCCESS)
            {
                // Release the memory allocated
                UnPreForAcquisition();
                GX_VERIFY_EXIT(emStatus);
            }

            // Release the resources and stop acquisition thread
            UnPreForAcquisition();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
        
    }

    //采集单帧图像
    bool AcqSingleImg(cv::Mat& img)
    {
        GX_STATUS emStatus = GX_STATUS_SUCCESS;

        PGX_FRAME_BUFFER pFrameBuffer = NULL;
        try
        {
            // Get a frame from Queue
            emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, 1000);
            if (emStatus != GX_STATUS_SUCCESS)
                return false;

            if (pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
            {
                return false;
            }
            else
            {
                int nRet = PixelFormatConvert(pFrameBuffer);
                if (nRet == PIXFMT_CVT_SUCCESS)
                {
                    cv::Mat imgtmp = cv::Mat(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3, g_pRGBImageBuf);
                    //图片尺寸修改，原始图像1920x1200
                    cv::resize(imgtmp, imgtmp, cv::Size(imgtmp.cols * 0.5, imgtmp.rows *0.5), 0, 0, cv::INTER_LINEAR);
                    //色彩空间转化
                    cv::cvtColor(imgtmp, imgtmp, cv::COLOR_RGB2BGR); //参数1-要处理的，参数2-处理后的
                    
                    imgtmp.copyTo(img);
                }
                else
                {
                    return false;
                }
                waitKey(1);
            }

            emStatus = GXQBuf(g_hDevice, pFrameBuffer);

            if(emStatus != GX_STATUS_SUCCESS)
            {
                GetErrorString(emStatus);
                return false;
            } 
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }
    
    int PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer)
    {
        VxInt32 emDXStatus = DX_OK;

        // Convert RAW16 or RAW8 image to Raw8 or RGB24 image
        switch (pFrameBuffer->nPixelFormat)
        {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24((unsigned char *)pFrameBuffer->pImgBuf, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                                       RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char *)pFrameBuffer->pImgBuf, g_pRaw8Image, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24(g_pRaw8Image, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                                       RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return PIXFMT_CVT_FAIL;
        }
        }
        return PIXFMT_CVT_SUCCESS;
    }

    void PreForAcquisition()
    {
        g_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3];
        g_pRaw8Image = new unsigned char[g_nPayloadSize];

        return;
    }

    void UnPreForAcquisition()
    {
        // Release resources
        if (g_pRaw8Image != NULL)
        {
            delete[] g_pRaw8Image;
            g_pRaw8Image = NULL;
        }
        if (g_pRGBImageBuf != NULL)
        {
            delete[] g_pRGBImageBuf;
            g_pRGBImageBuf = NULL;
        }

        return;
    }
 
    void GetErrorString(GX_STATUS emErrorStatus)
    {
        char *error_info = NULL;
        size_t size = 0;
        GX_STATUS emStatus = GX_STATUS_SUCCESS;

        // Get length of error description
        emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            printf("<Error when calling GXGetLastError>\n");
            return;
        }

        // Alloc error resources
        error_info = new char[size];
        if (error_info == NULL)
        {
            printf("<Failed to allocate memory>\n");
            return;
        }

        // Get error description
        emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
        if (emStatus != GX_STATUS_SUCCESS)
        {
            printf("<Error when calling GXGetLastError>\n");
        }
        else
        {
            printf("%s\n", error_info);
        }

        // Realease error resources
        if (error_info != NULL)
        {
            delete[] error_info;
            error_info = NULL;
        }
    }

public:
    GX_DEV_HANDLE g_hDevice = NULL;                  ///< Device handle
    bool g_bColorFilter = false;                     ///< Color filter support flag
    int64_t g_i64ColorFilter = GX_COLOR_FILTER_NONE; ///< Color filter of device
    bool g_bAcquisitionFlag = false;                 ///< Thread running flag
    bool g_bSavePPMImage = false;                    ///< Save raw image flag
    bool g_bWaitContinue = true;
    pthread_t g_nAcquisitonThreadID = 0; ///< Thread ID of Acquisition thread

    unsigned char *g_pRGBImageBuf = NULL; ///< Memory for RAW8toRGB24
    unsigned char *g_pRaw8Image = NULL;   ///< Memory for RAW16toRAW8

    int64_t g_nPayloadSize = 0; ///< Payload size

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    cv::Mat image_;
    unsigned long img_counter;
    bool bPub_;    // 是否发布图像
    bool bInit_;   // 是否初始化大恒相机环境
    bool bOpened_; // 是否打开相机成功
    //bool bAcq_;    // 是否开始采集

    void publishImage()
    {

        if (bPub_)
        {
            if (AcqSingleImg(image_))
            {
                img_counter++;
                if (img_counter > 4000000000)
                    img_counter = 1;

                // 创建一个ROS消息对象
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
                msg->header.frame_id = std::to_string(img_counter);
                publisher_->publish(*msg);

                RCLCPP_INFO(this->get_logger(), "第%ld次发布采集图像完成,高度=%d,宽度=%d", img_counter, image_.rows, image_.cols);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "未采集到图像......");
            }
        }
    }

    void controlcallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 在回调函数中处理接收到的消息
        // if (msg->data)
        // {
        //   RCLCPP_INFO(this->get_logger(), "开始发布图像......");
        //   bPub_=true;
        // }
        // else
        // {
        //   RCLCPP_INFO(this->get_logger(), "停止发布图像......");
        //   bPub_=false;
        // }
        switch (msg->data)
        {
        case 0: // 打开相机
            if (!bOpened_)
            {
                bOpened_ = OpenCamera();
                if (bOpened_)
                    RCLCPP_INFO(this->get_logger(), "相机打开成功......");
                else
                    RCLCPP_INFO(this->get_logger(), "相机打开失败......");
            }
            else
                RCLCPP_INFO(this->get_logger(), "相机处于打开状态或大恒环境未初始化......");
            break;
        case 1: // 关闭相机
            if (bOpened_)
            {
                if (CloseCamera())
                {
                    bOpened_ = false;
                    RCLCPP_INFO(this->get_logger(), "相机关闭成功......");
                }
                else
                    RCLCPP_INFO(this->get_logger(), "相机关闭失败......");
            }
            break;
        case 2: // 开始采集
            if (bOpened_ && !bPub_)
            {
                bPub_ = StartAcq();
                RCLCPP_INFO(this->get_logger(), "开始采集图像......");
            }
            else
                RCLCPP_INFO(this->get_logger(), "相机未打开或正在采集中......");
            break;
        case 3: // 停止采集
            if (bOpened_ && bPub_)
            {
                bPub_ = false;
                StopAcq();
                RCLCPP_INFO(this->get_logger(), "停止采集图像......");
            }
            else
                RCLCPP_INFO(this->get_logger(), "相机未打开或处于停止采集状态......");
            break;
        default:
            break;
        }
    }
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Vision>("vision");
    rclcpp::spin(node);
//    while (rclcpp::ok())
//    {
//        rclcpp::spin_some(node);
//    }
    rclcpp::shutdown();
    
    //Mat scr = imread("/home/mayin/picture/sunflower.jpg");
    //Mat scr = imread("sunflower.jpg");
    //imshow("scr", scr);
    //waitKey(0);   			// 这句确保窗口一直打开
    //printf("hello world opencv_use\n");
    
    return 0;
}


