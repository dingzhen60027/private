#include "rclcomm3.h"

rclcomm3::rclcomm3()
{
    node=rclcpp::Node::make_shared("gui_robosoccer_subPose");
    RCLCPP_INFO(node->get_logger(), "gui_robosoccer_subPose已创建!");

    isToStop = 0;

    _subscription_pose = node->create_subscription<message_interface_robosoccer::msg::PoseRobots>("robots_pose", 1, std::bind(&rclcomm3::recv_callback_p, this, std::placeholders::_1));

    this->start();
}

void rclcomm3::run()
{
    qDebug() << "gui_robosoccer_subPose thread:" << QThread::currentThreadId();
    while (rclcpp::ok() && this->isToStop == 0)
    {
        rclcpp::spin_some(node);
    }
    quit();
}

void rclcomm3::recv_callback_p(const message_interface_robosoccer::msg::PoseRobots::SharedPtr msg)
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

void rclcomm3::drawRobotsAndBall(message_interface_robosoccer::msg::PoseRobots Poses)
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

