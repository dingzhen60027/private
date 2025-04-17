#include "rclcomm.h"

rclcomm::rclcomm()
{
    node=rclcpp::Node::make_shared("gui_robosoccer");
    RCLCPP_INFO(node->get_logger(), "gui_robosoccer已创建!");

    isToStop = 0;

    Img_count = 0;

    //pub_msg.data = 0;
    //_publisher = node->create_publisher<std_msgs::msg::Int32>("number", 1);
    //_timer = node->create_wall_timer(std::chrono::milliseconds(20), std::bind(&rclcomm::timer_callback,this));
    //_subscription = node->create_subscription<std_msgs::msg::Int32>("number", 1, std::bind(&rclcomm::recv_callback, this, std::placeholders::_1));

    //窗口抖动可尝试接收队列长度设为1或者降低帧率
    _subscription_image_before = node->create_subscription<sensor_msgs::msg::Image>("robot_image", 1, std::bind(&rclcomm::recv_callback_i1, this, std::placeholders::_1));
    _subscription_image_after = node->create_subscription<sensor_msgs::msg::Image>("robot_image_processed", 1, std::bind(&rclcomm::recv_callback_i2, this, std::placeholders::_1));
    _subscription_pose = node->create_subscription<message_interface_robosoccer::msg::PoseRobots>("robots_pose", 1, std::bind(&rclcomm::recv_callback_p, this, std::placeholders::_1));

    _publisher_camCtrl = node->create_publisher<std_msgs::msg::Int32>("camera_control", 1);
    _publisher_boundSet = node->create_publisher<message_interface_robosoccer::msg::PointCalibration>("camera_calibration_point", 1);
    _publisher_colorSet = node->create_publisher<message_interface_robosoccer::msg::ArrayThreshold>("robot_colorthreshold_array", 1);
    _publisher_decisionVar = node->create_publisher<message_interface_robosoccer::msg::DecisionVar>("decision_var", 1);

    this->start();
}

void rclcomm::run()
{
    while (rclcpp::ok() && this->isToStop == 0)
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
}

/*
void rclcomm::timer_callback()
{
    pub_msg.data++;
    _publisher->publish(pub_msg);
}

void rclcomm::recv_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    emit TopicData("I head from number:"+QString::fromStdString(std::to_string(msg->data)));
}
*/

int rclcomm::encoding2mat_type(std::string encoding)
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
void rclcomm::recv_callback_i1(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (isFinishDisp_1)
    {
        isFinishDisp_1 = false;
        QImage robot_image_before;
        QImage Image_Disp_before;
        cv::Mat msg_cvImage(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()));
        if (msg_cvImage.empty() || msg_cvImage.cols != IMAGE_WIDTH || msg_cvImage.rows != IMAGE_HEIGHT)
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
            Image_Disp_before = robot_image_before.scaled(DISP_WIDTH, DISP_HEIGHT, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            emit TopicData_image_before(Image_Disp_before);
        }
    }
    Img_count++;
    if (Img_count > 4000000000)
        Img_count = 1;
    RCLCPP_INFO(this->node->get_logger(), "第%ld次收到图像,高度=%d,宽度=%d,图片ID=%s",Img_count, msg->height, msg->width, msg->header.frame_id.c_str());
    return;
}
void rclcomm::recv_callback_i2(const sensor_msgs::msg::Image::SharedPtr msg)
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

void rclcomm::recv_callback_p(const message_interface_robosoccer::msg::PoseRobots::SharedPtr msg)
{
    if (isFinishDrawRobo)
    {
        isFinishDrawRobo = false;
        message_interface_robosoccer::msg::PoseRobots robots_pose;
        for (int i=0; i<11; i++)
        {
            robots_pose.pose[i] = msg->pose[i];
        }
        drawRobotsAndBall(robots_pose);
    }
    return;
}

void rclcomm::drawRobotsAndBall(message_interface_robosoccer::msg::PoseRobots Poses)
{
    int x, y, theta;
    QPixmap bitmapGround(":/res/images/ground.bmp");
    QPainter painter(&bitmapGround);

    //QPen pen;
    //pen.setWidth(2);
    //pen.setColor(Qt::yellow);
    //painter.setPen(pen);
    //设置字体
    QFont font;
    font.setPointSize(12);
    font.setWeight(75);
    painter.setFont(font);
    //转换成屏幕坐标
    for(int i=0;i<5;i++)//己方机器人
    {
        if ((Poses.pose[i].x != 0) && (Poses.pose[i].y != 0))
        {
            x = (int)(Poses.pose[i].x * GROUND_BITMAP_WIDTH/GROUND_WIDTH)+GROUND_BITMAP_X_OFFSET;
            y = (int)(Poses.pose[i].y * GROUND_BITMAP_HEIGHT/GROUND_HEIGHT)+GROUND_BITMAP_Y_OFFSET;
            theta = (int)Poses.pose[i].theta;
            if((theta<0)||(theta>359))
            {
                continue;
            }
            QPen pen1;
            pen1.setWidth(2);
            pen1.setColor(Qt::yellow);
            painter.setPen(pen1);
            //连接关键点画出机器人
            QPoint p1 = QPoint(x+robot_xy[theta][0][0],y+robot_xy[theta][0][1]);
            QPoint p2 = QPoint(x+robot_xy[theta][1][0],y+robot_xy[theta][1][1]);
            QPoint p3 = QPoint(x+robot_xy[theta][3][0],y+robot_xy[theta][3][1]);
            QPoint p4 = QPoint(x+robot_xy[theta][2][0],y+robot_xy[theta][2][1]);
            painter.drawLine(p1,p2);
            painter.drawLine(p2,p3);
            painter.drawLine(p3,p4);
            painter.drawLine(p4,p1);

            QPoint p5 = QPoint(x+robot_xy[theta][4][0],y+robot_xy[theta][4][1]);
            QPoint p6 = QPoint(x+robot_xy[theta][5][0],y+robot_xy[theta][5][1]);
            QPoint p7 = QPoint(x+robot_xy[theta][10][0],y+robot_xy[theta][10][1]);
            QPoint p8 = QPoint(x+robot_xy[theta][11][0],y+robot_xy[theta][11][1]);
            painter.drawLine(p5,p6);
            painter.drawLine(p5,p7);
            painter.drawLine(p5,p8);

            QPen pen2;
            pen2.setWidth(2);
            pen2.setColor(Qt::red);
            painter.setPen(pen2);
            QString str;
            str = QString("%1").arg(i+1);
            painter.drawText(x,y,str);
            //qDebug()<<i<<"-num"<<robotInfor[i].x<<"x"<<robotInfor[i].y;
        }
    }
    for(int i=0;i<5;i++)//敌方机器人
    {
        if ((Poses.pose[i+5].x != 0) && (Poses.pose[i+5].y != 0))
        {
            x = (int)(Poses.pose[i+5].x * GROUND_BITMAP_WIDTH/GROUND_WIDTH)+GROUND_BITMAP_X_OFFSET;
            y = (int)(Poses.pose[i+5].y * GROUND_BITMAP_HEIGHT/GROUND_HEIGHT)+GROUND_BITMAP_Y_OFFSET;
            theta = (int)Poses.pose[i+5].theta;
            if((theta<0)||(theta>359))
            {
                continue;
            }
            QPen pen1;
            pen1.setWidth(2);
            pen1.setColor(Qt::blue);
            painter.setPen(pen1);
            //画出机器人
            painter.drawEllipse(x,y,10,10);

            QPen pen2;
            pen2.setWidth(2);
            pen2.setColor(Qt::black);
            painter.setPen(pen2);
            QString str;
            str = QString("O");
            painter.drawText(x,y,str);
            //qDebug()<<i<<"-num"<<robotInfor[i+5].x<<"x"<<robotInfor[i+5].y;
        }
    }

    QPen pen;
    pen.setColor(qRgb(255,128,0));
    pen.setWidth(3);
    QBrush brush;
    brush.setColor(qRgb(255,128,0));
    brush.setStyle(Qt::SolidPattern);
    painter.setPen(pen);
    painter.setBrush(brush);
    if((Poses.pose[10].x!=0)&&Poses.pose[10].y!=0)
    {
        x = (int)(Poses.pose[10].x * GROUND_BITMAP_WIDTH/GROUND_WIDTH)+GROUND_BITMAP_X_OFFSET;
        y = (int)(Poses.pose[10].y * GROUND_BITMAP_HEIGHT/GROUND_HEIGHT)+GROUND_BITMAP_Y_OFFSET;
    }
    painter.drawEllipse(x,y,8,8);

    emit sendSimuGround(bitmapGround);
    return;
}

void rclcomm::open_camera()
{
    auto message = std_msgs::msg::Int32();
    message.data = 0;
    _publisher_camCtrl->publish(message);
}

void rclcomm::close_camera()
{
    auto message = std_msgs::msg::Int32();
    message.data = 1;
    _publisher_camCtrl->publish(message);
}

void rclcomm::start_acq()
{
    auto message = std_msgs::msg::Int32();
    message.data = 2;
    _publisher_camCtrl->publish(message);
}

void rclcomm::quit_acq()
{
    auto message = std_msgs::msg::Int32();
    message.data = 3;
    _publisher_camCtrl->publish(message);
}

void rclcomm::BoundsetPoint(Point* calipoint_)
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

void rclcomm::ColorsetThreshold(int* hsithreshold_)
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

void rclcomm::DecisionsetVar()
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
