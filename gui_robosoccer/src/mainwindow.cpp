#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <thread>
#include <QThread>
#include <QDebug>

//#include <QScreen>
//#include <QGuiApplication>

#include <QString>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>

#include <exception>

#include "globalVar.h"
#include "globalFun.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    qDebug() << "gui_robosoccer已创建!";
    qDebug() << "gui_robosoccer thread:" << QThread::currentThreadId();
    
    centerfcalib = 0;

    //----------------------------------GUI设置相关---------------------------------
    //设置图像窗口初始图片
    QImage img, img1;
    img.load(":/res/images/waitForVision_camera.png");
    ui->Lab_Image_C->setFixedSize(DISP_WIDTH, DISP_HEIGHT);
    ui->Lab_Image_C->setScaledContents(true);
    ui->Lab_Image_C->setPixmap(QPixmap::fromImage(img));
    ui->Lab_Image_C->show();
    m_QImageForDisp = img.scaled(DISP_WIDTH, DISP_HEIGHT, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    img1.load(":/res/images/waitForVision_process.png");
    ui->Lab_Image_P->setFixedSize(SIMULATION_WIDTH, SIMULATION_HEIGHT);
    ui->Lab_Image_P->setScaledContents(true);
    ui->Lab_Image_P->setPixmap(QPixmap::fromImage(img1));
    ui->Lab_Image_P->show();
    //设置HSI色盘图片
    ui->Lab_ColorData->setScaledContents(true);
    ui->Lab_ColorData->setPixmap(QPixmap(":/res/images/HSICir.bmp"));
    ui->Lab_ColorData->show();

    //设置采色当前Object对象按钮组
    buttonGroup = new QButtonGroup(this);
    buttonGroup->addButton(ui->Sel_Team,0);
    buttonGroup->addButton(ui->Sel_Mem1,1);
    buttonGroup->addButton(ui->Sel_Mem2,2);
    buttonGroup->addButton(ui->Sel_Mem3,3);
    buttonGroup->addButton(ui->Sel_Mem4,4);
    buttonGroup->addButton(ui->Sel_Mem5,5);
    buttonGroup->addButton(ui->Sel_Ball,6);
    buttonGroup->addButton(ui->Sel_Opp,7);

    //设置策略进攻方按钮组
    buttonGroup_Attack = new QButtonGroup(this);
    buttonGroup_Attack->addButton(ui->Sel_SelfAttack,0);
    buttonGroup_Attack->addButton(ui->Sel_OppAttack,1);

    //设置策略半场选择按钮组
    buttonGroup_HalfGround = new QButtonGroup(this);
    buttonGroup_HalfGround->addButton(ui->Sel_LeftGround,0);
    buttonGroup_HalfGround->addButton(ui->Sel_RightGround,1);

    //设置策略比赛模式按钮组
    buttonGroup_GameMode = new QButtonGroup(this);
    buttonGroup_GameMode->addButton(ui->Sel_GameMode0,0);
    buttonGroup_GameMode->addButton(ui->Sel_GameMode1,1);
    buttonGroup_GameMode->addButton(ui->Sel_GameMode2,2);
    buttonGroup_GameMode->addButton(ui->Sel_GameMode3,3);
    buttonGroup_GameMode->addButton(ui->Sel_GameMode4,4);
    buttonGroup_GameMode->addButton(ui->Sel_GameMode5,5);

    //设置策略开始模式按钮组
    buttonGroup_StartMode = new QButtonGroup(this);
    buttonGroup_StartMode->addButton(ui->Sel_BackInit,0);
    buttonGroup_StartMode->addButton(ui->Sel_StartGame,1);

    //设置策略变量重复模式按钮组
    buttonGroup_VarSet = new QButtonGroup(this);
    buttonGroup_VarSet->addButton(ui->Sel_InitVar,0);
    buttonGroup_VarSet->addButton(ui->Sel_ContiVar,1);


    //设置Slider的最大值
    ui->Sld_H_Low->setMaximumByNumIn(3600);
    ui->Sld_H_High->setMaximumByNumIn(3600);
    ui->Sld_S_Low->setMaximumByNumIn(100);
    ui->Sld_S_High->setMaximumByNumIn(100);
    ui->Sld_I_Low->setMaximumByNumIn(255);
    ui->Sld_I_High->setMaximumByNumIn(255);

    //初始化颜色阈值
    for(int i=0;i<8;i++)
    {
        HSIThreshold[i][0]=0;
        HSIThreshold[i][1]=0;//3600
        HSIThreshold[i][2]=0;
        HSIThreshold[i][3]=0;//100
        HSIThreshold[i][4]=0;
        HSIThreshold[i][5]=0;//255
    }
    //初始化HSI颜色转换表
    HLUT = (int ***)malloc(256 * sizeof(int **));
    for(int i=0;i<256;i++)
    {
        if(HLUT)
        {
            HLUT[i] = (int **)malloc(256 * sizeof(int *));
            if(HLUT[i])
            {
                for(int j=0;j<256;j++)
                {
                    HLUT[i][j] = (int *)malloc(256* sizeof(int));
                }
            }
        }
    }
    if(HLUT == NULL)
    {
        qDebug()<<"HLUT无法动态申请内存";
    }
    else   //RGB转换H
    {
        for(int r=0;r<256;++r)
        {
            for(int g=0;g<256;++g)
            {
                for(int b=0;b<256;++b)
                {
                    HLUT[r][g][b]=(int)changeColorFromRGBtoHSI(r,g,b);
                }
            }
        }
    }
    //初始化机器人仿真关键点查找表
    double ttheta, side = 7.5 * 0.5 * GROUND_BITMAP_WIDTH/GROUND_WIDTH;
    for (int i = 0;i < 360;i++)
    {
        ttheta = 3.1415926 * (2 - (double)i / 180);//2pi-i*pi/180
        robot_xy[i][0][0] = (int)(side * cos(ttheta) - side * sin(ttheta));
        robot_xy[i][0][1] = (int)(side * sin(ttheta) + side * cos(ttheta));
        robot_xy[i][1][0] = (int)(side * cos(ttheta) + side * sin(ttheta));
        robot_xy[i][1][1] = (int)(side * sin(ttheta) - side * cos(ttheta));
        robot_xy[i][2][0] = (int)(-side * cos(ttheta) - side * sin(ttheta));
        robot_xy[i][2][1] = (int)(-side * sin(ttheta) + side * cos(ttheta));
        robot_xy[i][3][0] = (int)(-side * cos(ttheta) + side * sin(ttheta));
        robot_xy[i][3][1] = (int)(-side * sin(ttheta) - side * cos(ttheta));

        robot_xy[i][4][0] = (robot_xy[i][0][0] + robot_xy[i][1][0]) / 2;
        robot_xy[i][4][1] = (robot_xy[i][0][1] + robot_xy[i][1][1]) / 2;
        robot_xy[i][5][0] = (robot_xy[i][2][0] + robot_xy[i][3][0]) / 2;
        robot_xy[i][5][1] = (robot_xy[i][2][1] + robot_xy[i][3][1]) / 2;
        robot_xy[i][6][0] = (robot_xy[i][0][0] + robot_xy[i][2][0]) / 2;
        robot_xy[i][6][1] = (robot_xy[i][0][1] + robot_xy[i][2][1]) / 2;
        robot_xy[i][7][0] = (robot_xy[i][1][0] + robot_xy[i][3][0]) / 2;
        robot_xy[i][7][1] = (robot_xy[i][1][1] + robot_xy[i][3][1]) / 2;

        robot_xy[i][8][0] = robot_xy[i][6][0] / 2;
        robot_xy[i][8][1] = robot_xy[i][6][1] / 2;
        robot_xy[i][9][0] = robot_xy[i][7][0] / 2;
        robot_xy[i][9][1] = robot_xy[i][7][1] / 2;

        robot_xy[i][10][0] = (robot_xy[i][4][0] + robot_xy[i][6][0]) / 2;
        robot_xy[i][10][1] = (robot_xy[i][4][1] + robot_xy[i][6][1]) / 2;
        robot_xy[i][11][0] = (robot_xy[i][4][0] + robot_xy[i][7][0]) / 2;
        robot_xy[i][11][1] = (robot_xy[i][4][1] + robot_xy[i][7][1]) / 2;
    }

    //信号与槽的连接
    //Tab1标定页
    /*************按钮控制鼠标点的移动**********************/
    connect(ui->Btn_PointUp,&QPushButton::pressed,[=](){
        if(ui->Lab_Image_C->y_pos>0)
            ui->Lab_Image_C->y_pos -= 1;
        ui->Lab_Image_C->m_isMousePress=true;
        ui->Lab_Image_C->update();
    });
    connect(ui->Btn_PointUp,&QPushButton::released,[=](){
        ui->Lab_Image_C->m_isMousePress=false;
    });//上移
    connect(ui->Btn_PointDown,&QPushButton::pressed,[=](){
        if(ui->Lab_Image_C->y_pos<IMAGE_HEIGHT)
            ui->Lab_Image_C->y_pos += 1;
        ui->Lab_Image_C->m_isMousePress=true;
        ui->Lab_Image_C->update();
    });
    connect(ui->Btn_PointDown,&QPushButton::released,[=](){
        ui->Lab_Image_C->m_isMousePress=false;
    });//下移
    connect(ui->Btn_PointLeft,&QPushButton::pressed,[=](){
        if(ui->Lab_Image_C->x_pos>0)
            ui->Lab_Image_C->x_pos -= 1;
        ui->Lab_Image_C->m_isMousePress=true;
        ui->Lab_Image_C->update();
    });
    connect(ui->Btn_PointLeft,&QPushButton::released,[=](){
        ui->Lab_Image_C->m_isMousePress=false;
    });//左移
    connect(ui->Btn_PointRight,&QPushButton::pressed,[=](){
        if(ui->Lab_Image_C->x_pos<IMAGE_WIDTH)
            ui->Lab_Image_C->x_pos += 1;
        ui->Lab_Image_C->m_isMousePress=true;
        ui->Lab_Image_C->update();
    });
    connect(ui->Btn_PointRight,&QPushButton::released,[=](){
        ui->Lab_Image_C->m_isMousePress=false;
    });//右移
    /****************************************************/
    /******************标定点区域放大显示******************/
    connect(ui->Lab_Image_C,&ImageLabelControl::pointDrew,[=](){
        //QScreen *screen=QGuiApplication::primaryScreen();
        int x=ui->Lab_Image_C->x_pos-50;
        int y=ui->Lab_Image_C->y_pos-50;
        //QPixmap pix=screen->grabWindow(ui->Lab_Image_C->winId(),x,y,100,100);
        //QImage img = m_QImageForDisp.copy(x*IMAGE_WIDTH/DISP_WIDTH, y*IMAGE_HEIGHT/DISP_HEIGHT, 100*IMAGE_WIDTH/DISP_WIDTH, 100*IMAGE_HEIGHT/DISP_HEIGHT);
        QImage img = m_QImageForDisp.copy(x, y, 101, 101);
        img.setPixel(QPoint(50, 50), qRgb(255, 0, 0));
        ui->Lab_PointSel->setScaledContents(true);
        //ui->Lab_PointSel->setPixmap(pix);
        ui->Lab_PointSel->setPixmap(QPixmap::fromImage(img));
        ui->Lab_PointSel->show();
    });
    /****************************************************/
    //Tab2采色页
    //获取当前选中的Object对象按钮
    //void(QButtonGroup:: *buttonSignal)(int)=&QButtonGroup::buttonClicked;
    //connect(buttonGroup,buttonSignal,ui->Lab_ColorData,&ImageLabelColor::getRadioButtonID);
    //connect(buttonGroup,buttonSignal,ui->Lab_ShowIdata,&ImageLabelDrawI::getRadioButtonID);
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Lab_ColorData,SLOT(getRadioButtonID(int)));
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Lab_ShowIdata,SLOT(getRadioButtonID(int)));
    /*****切换Object对象按钮的时候更新enlarge界面*****/
    //connect(buttonGroup,buttonSignal,ui->Lab_ColorEnlarge,&ImageLabelEnlarge::updateLabel);
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Lab_ColorEnlarge,SLOT(updateLabel(int)));
    /*****切换Object对象按钮对象时重置spinBox和Slider的位置*****/
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Sld_H_Low,SLOT(HToMini(int)));
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Sld_H_High,SLOT(HToMax(int)));
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Sld_S_Low,SLOT(SToMini(int)));
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Sld_S_High,SLOT(SToMax(int)));
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Sld_I_Low,SLOT(IToMini(int)));
    connect(buttonGroup,SIGNAL(buttonClicked(int)),ui->Sld_I_High,SLOT(IToMax(int)));
    /****************************************************/
    /******************采色区域放大显示******************/
    connect(ui->Lab_Image_C, SIGNAL(sendRect(QRect)), this, SLOT(get_Color_Enlarge(QRect)));
    //通过enlarge取点采色并分析
    //connect(ui->Lab_ColorEnlarge,&ImageLabelEnlarge::updateColorImage,ui->Lab_ColorData,&ImageLabelColor::getColorImage);
    connect(ui->Lab_ColorEnlarge,SIGNAL(updateColorImage(QPoint*, int)),ui->Lab_ColorData,SLOT(getColorImage(QPoint*, int)));
    connect(ui->Lab_ColorEnlarge,&ImageLabelEnlarge::updateIdataImage,ui->Lab_ShowIdata,&ImageLabelDrawI::getIimage);
    /****************************************************/
    /*****spinBox QSlider值的改变来确定HSI并在Lab_ColorData中绘制*****/
    connect(ui->Sld_H_Low,&HSIslider::valueIsChanged,ui->Lab_ColorData,&ImageLabelColor::getHLOWNum);
    connect(ui->Sld_H_High,&HSIslider::valueIsChanged,ui->Lab_ColorData,&ImageLabelColor::getHHIGHNUm);
    connect(ui->Sld_S_Low,&HSIslider::valueIsChanged,ui->Lab_ColorData,&ImageLabelColor::getSLOWNum);
    connect(ui->Sld_S_High,&HSIslider::valueIsChanged,ui->Lab_ColorData,&ImageLabelColor::getSHIGHNum);
    connect(ui->Sld_I_Low,&HSIslider::valueIsChanged,ui->Lab_ColorData,&ImageLabelColor::getILOWNum);
    connect(ui->Sld_I_High,&HSIslider::valueIsChanged,ui->Lab_ColorData,&ImageLabelColor::getIHIGHNum);
    /*****spinBox QSlider值的改变来确定HSI并在Lab_ShowIdata中绘制*****/
    connect(ui->Sld_I_Low,&HSIslider::valueIsChanged,ui->Lab_ShowIdata,&ImageLabelDrawI::getILOWNum);
    connect(ui->Sld_I_High,&HSIslider::valueIsChanged,ui->Lab_ShowIdata,&ImageLabelDrawI::getIHIGHNum);
    /****************************************************/
    //Tab3比赛页
    connect(buttonGroup_Attack,SIGNAL(buttonClicked(int)),this,SLOT(getRadioButtonID_A(int)));
    connect(buttonGroup_HalfGround,SIGNAL(buttonClicked(int)),this,SLOT(getRadioButtonID_H(int)));
    connect(buttonGroup_GameMode,SIGNAL(buttonClicked(int)),this,SLOT(getRadioButtonID_G(int)));
    connect(buttonGroup_StartMode,SIGNAL(buttonClicked(int)),this,SLOT(getRadioButtonID_S(int)));
    connect(buttonGroup_VarSet,SIGNAL(buttonClicked(int)),this,SLOT(getRadioButtonID_V(int)));
    /****************************************************/
    //--------------------------------------------------------------------------
    
    //----------------------------------ROS2相关----------------------------------
    //初始化 ROS2
    int argc=0;
    char **argv=NULL;
    rclcpp::init(argc,argv);

    /*
    //实例化节点（可以在这里实例化多个节点，用于不同的功能）
    commNode = new rclcomm();
    
    //ROS2信号与Mainwindow槽的连接
    //connect(commNode, SIGNAL(TopicData(QString)), this, SLOT(updateTopicInfo(QString)));
    connect(commNode, SIGNAL(TopicData_image_before(QImage)), this, SLOT(updateTopicInfo_image_before(QImage)));
    connect(commNode, SIGNAL(TopicData_image_after(QImage)), this, SLOT(updateTopicInfo_image_after(QImage)));
    connect(commNode, SIGNAL(sendSimuGround(QPixmap)), this, SLOT(updateSimuImage(QPixmap)));
    //Mainwindow信号与ROS2槽的连接
    connect(this, SIGNAL(Open_Camera_INFO()), commNode, SLOT(open_camera()));
    connect(this, SIGNAL(Close_Camera_INFO()),commNode, SLOT(close_camera()));
    connect(this, SIGNAL(Start_Acq_INFO()),commNode, SLOT(start_acq()));
    connect(this, SIGNAL(Quit_Acq_INFO()),commNode, SLOT(quit_acq()));
    connect(ui->Lab_Image_C, SIGNAL(sendBoundsetPoint(Point*)), commNode, SLOT(BoundsetPoint(Point*)));
    connect(this, SIGNAL(sendHSIcolor(int*)), commNode, SLOT(ColorsetThreshold(int*)));

    connect(this, SIGNAL(sendDecisionvar()), commNode, SLOT(DecisionsetVar()));
    */
    
    ////分工模式加速
    //实例化节点（可以在这里实例化多个节点，用于不同的功能）
    commNode1 = new rclcomm1;
    //commNode2 = new rclcomm2;
    commNode3 = new rclcomm3;
    commNode4 = new rclcomm4;
    
    ////分工模式加速
    //ROS2信号与Mainwindow槽的连接
    connect(commNode1, SIGNAL(TopicData_image_before(QImage)), this, SLOT(updateTopicInfo_image_before(QImage)));
    //connect(commNode2, SIGNAL(TopicData_image_after(QImage)), this, SLOT(updateTopicInfo_image_after(QImage)));
    connect(commNode3, SIGNAL(sendSimuGround(QPixmap)), this, SLOT(updateSimuImage(QPixmap)));
    //Mainwindow信号与ROS2槽的连接
    connect(this, SIGNAL(Open_Camera_INFO()), commNode4, SLOT(open_camera()));
    connect(this, SIGNAL(Close_Camera_INFO()),commNode4, SLOT(close_camera()));
    connect(this, SIGNAL(Start_Acq_INFO()),commNode4, SLOT(start_acq()));
    connect(this, SIGNAL(Quit_Acq_INFO()),commNode4, SLOT(quit_acq()));
    connect(ui->Lab_Image_C, SIGNAL(sendBoundsetPoint(Point*)), commNode4, SLOT(BoundsetPoint(Point*)));
    connect(this, SIGNAL(sendHSIcolor(int*)), commNode4, SLOT(ColorsetThreshold(int*)));

    connect(this, SIGNAL(sendDecisionvar()), commNode4, SLOT(DecisionsetVar()));
    
    //--------------------------------------------------------------------------

    initDecisionButtonGroup();

    //QMessageBox msg;
    //msg.setText("界面初始化完成!");
    //msg.exec();
}

MainWindow::~MainWindow()
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭线程
    /*
    if(commNode->isToStop == 0)
    {
        commNode->isToStop = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭节点线程
        delete commNode;
        qDebug()<<"已经关闭GUI节点!";
    }
    */
    if(commNode1->isToStop == 0)
    {
        commNode1->isToStop = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭节点线程
        if (commNode1 != NULL)
        {
            delete commNode1;
        }
        qDebug()<<"已经关闭GUI_subImgBefore节点!";
    }
//    if(commNode2->isToStop == 0)
//    {
//        commNode2->isToStop = 1;
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭节点线程
//        if (commNode2 != NULL)
//        {
//          delete commNode2;
//        }
//        qDebug()<<"已经关闭GUI_subImgAfter节点!";
//    }
    if(commNode3->isToStop == 0)
    {
        commNode3->isToStop = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭节点线程
        if (commNode3 != NULL)
        {
            delete commNode3;
        }
        qDebug()<<"已经关闭GUI_subPose节点!";
    }
    if(commNode4->isToStop == 0)
    {
        commNode4->isToStop = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭节点线程
        if (commNode4 != NULL)
        {
            delete commNode4;
        }
        qDebug()<<"已经关闭GUI_publisher节点!";
    }
    
    rclcpp::shutdown();
    
    //释放数组
    if (HLUT != NULL)
    {
        for (int i = 0; i < 256; i++)
        {
            if(HLUT[i] != NULL)
            {
                for (int j = 0; j < 256; j++)
                {
                    if(HLUT[i][j] != NULL)
                    {
                        delete[] HLUT[i][j];
                    }
                }
                delete[] HLUT[i];
            }
        }
        delete[] HLUT;
        qDebug()<<"已经清除HLUT所占内存!";
    }
    
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));//等待10ms关闭窗口线程
    delete ui;
}

/*
void MainWindow::updateTopicInfo(QString data)
{
    ui->label_num->clear();
    ui->label_num->setText(data);
}
*/

void MainWindow::updateTopicInfo_image_before(QImage data)
{
//    //ui->Lab_Image_C->clear();
//    //ui->Lab_Image_C->setScaledContents(true);
//    m_QImageForDisp = data;
//    QPixmap Disp_Image = QPixmap::fromImage(data);
//    ui->Lab_Image_C->setPixmap(Disp_Image);//.scaled(QSize(DISP_WIDTH,DISP_HEIGHT), Qt::KeepAspectRatio, Qt::SmoothTransformation));
//    //ui->Lab_Image_C->adjustSize();
//    ui->Lab_Image_C->update();

//    isFinishDisp_1 = true;


    m_QImageForDisp = data;
    QPixmap Disp_Image = QPixmap::fromImage(data);

    if (!isSimuing)//大窗口(采集图像窗口)显示
    {
        ui->Lab_Image_C->setPixmap(Disp_Image);
        ui->Lab_Image_C->update();
    }
    else//小窗口(处理图像窗口)显示
    {
        ui->Lab_Image_P->setPixmap(Disp_Image);
        ui->Lab_Image_P->update();
    }
    
    isFinishDisp_1 = true;
}

void MainWindow::updateTopicInfo_image_after(QImage data)
{
    //小窗口(处理图像窗口)显示
    //ui->Lab_Image_P->clear();
    //ui->Lab_Image_P->setScaledContents(true);
    QPixmap Disp_Image = QPixmap::fromImage(data);
    ui->Lab_Image_P->setPixmap(Disp_Image);//.scaled(QSize(SIMULATION_WIDTH,SIMULATION_HEIGHT), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    //ui->Lab_Image_P->adjustSize();
    ui->Lab_Image_P->update();

    //大窗口(采集图像窗口)显示
    //ui->Lab_Image_C->setPixmap(Disp_Image);
    //ui->Lab_Image_C->update();
    ui->Lab_Image_C->m_boundsetPress=false;

    //只显示一次标定结果，实时显示则注释if
    if(ui->Lab_Image_C->m_boundsetPress==true)
    {
        isFinishDisp_2 = true;
    }

    return;
}

void MainWindow::updateSimuImage(QPixmap data)
{
//    //小窗口(处理图像窗口)显示
//    //ui->Lab_Image_P->clear();
//    //ui->Lab_Image_P->setScaledContents(true);
//    ui->Lab_Image_P->setPixmap(data);//.scaled(QSize(SIMULATION_WIDTH,SIMULATION_HEIGHT), Qt::KeepAspectRatio, Qt::SmoothTransformation));
//    //ui->Lab_Image_P->adjustSize();
//    ui->Lab_Image_P->update();

//    //大窗口(采集图像窗口)显示
//    //ui->Lab_Image_C->setPixmap(data);
//    //ui->Lab_Image_C->update();

//    isFinishDrawRobo = true;

    if (isSimuing)//大窗口(采集图像窗口)显示
    {
        ui->Lab_Image_C->setPixmap(data);
        ui->Lab_Image_C->update();
    }
    else//小窗口(处理图像窗口)显示
    {
        ui->Lab_Image_P->setPixmap(data);
        ui->Lab_Image_P->update();
    }
    
    isFinishDrawRobo = true;
}

/*
void MainWindow::on_myStopButton_clicked()
{
    //rclcpp::shutdown();
    commNode->isToStop = 1;
}
*/

void MainWindow::on_Btn_Open_clicked()
{
    emit Open_Camera_INFO();
    isOpenCamera = true;
    qDebug() << "开启相机...";
}

void MainWindow::on_Btn_Close_clicked()
{
    emit Close_Camera_INFO();
    isOpenCamera = false;
    qDebug() << "关闭相机...";
}

void MainWindow::on_Btn_Start_clicked()
{
    isSimuing = false;
    
    ui->Lab_Image_C->m_acquisition = true;
    emit Start_Acq_INFO();
    isStartAcq = true;
    qDebug() << "开始采集...";
}

void MainWindow::on_Btn_Quit_clicked()
{
    ui->Lab_Image_C->m_acquisition = false;
    emit Quit_Acq_INFO();
    isStartAcq = false;
    qDebug() << "停止采集...";
}

void MainWindow::on_Btn_Load_clicked()
{
    QString path;
    try
    {
        if (isStartAcq)
        {
            emit Quit_Acq_INFO();
            isStartAcq = false;
            qDebug() << "停止采集...";
        }
        //path = QFileDialog::getExistingDirectory(this, "选择加载路径", "./");//无法显示考虑用参数QFileDialog::DontUseNativeDialog,若卡住了：右键程序图标-退出所有窗口
        path = "/home/mayin/document/robodata";
    }
    catch (std::exception &e)
    {
        qDebug() << "启动加载错误:%s!!" << e.what();
        return;
    }
    if(!path.isEmpty())
    {
        QFile calibFile(path + "/CaliPoint.dat");
        if(!calibFile.open(QIODevice::ReadOnly))
        {
            qDebug() << "文件打开失败!!";
            return;
        }
        QTextStream In(&calibFile);
        for(int i=0;i<centerNum;i++)
        {
            In >> calipoint[i].x;
            In >> calipoint[i].y;
        }
        calibFile.close();
        centerfcalib = 16;

        QFile HSIthresholdFile(path + "/HSIthreshold.dat");
        HSIthresholdFile.open(QIODevice::ReadOnly);
        QTextStream hsiIn(&HSIthresholdFile);
        for(int object=0;object<8;object++)
        {
            for(int k=0;k<6;k++)
            {
                hsiIn >> HSIThreshold[object][k];
            }
        }
        HSIthresholdFile.close();

        QMessageBox msg;
        msg.setText("加载成功!");
        msg.exec();
    }
    else
    {
        qDebug() << "文件默认加载路径：" << path;
    }
    return;
}

void MainWindow::on_Btn_Save_clicked()
{
    QString path;
    try
    {
        if (isStartAcq)
        {
            emit Quit_Acq_INFO();
            isStartAcq = false;
            qDebug() << "停止采集...";
        }
        //path = QFileDialog::getExistingDirectory(this, "选择保存路径", "./");
        path = "/home/mayin/document/robodata";
    }
    catch (std::exception &e)
    {
        qDebug() << "启动保存错误:%s!!" << e.what();
        return;
    }
    if(!path.isEmpty())
    {
        QFile calibFile(path + "/CaliPoint.dat");
        if(!calibFile.open(QIODevice::WriteOnly | QIODevice::Truncate))
        {
            qDebug() << "文件打开失败!!";
            return;
        }
        QTextStream out(&calibFile);
        for(int i=0;i<centerNum;i++)
        {
            //文本文件流一次流读取是以一个空格为结束的
            out << calipoint[i].x;
            out << " ";
            out << calipoint[i].y;
            out << " ";
        }
        calibFile.close();

        QFile HSIthresholdFile(path + "/HSIthreshold.dat");
        HSIthresholdFile.open(QIODevice::WriteOnly | QIODevice::Truncate);
        QTextStream hsiOut(&HSIthresholdFile);
        for(int object=0;object<8;object++)
        {
            for(int k=0;k<6;k++)
            {
                hsiOut << HSIThreshold[object][k];
                hsiOut << " ";
            }
        }
        HSIthresholdFile.close();

        QMessageBox msg;
        msg.setText("保存成功!");
        msg.exec();
    }
    else
    {
        qDebug() << "文件默认保存路径：" << path;
    }
    return;
}


void MainWindow::on_Btn_PointRecord_clicked()
{
    if(!ui->Lab_Image_C->m_acquisition && centerfcalib<16)
    {
        calipoint[centerfcalib].x=(ui->Lab_Image_C->x_pos)*IMAGE_WIDTH/DISP_WIDTH;
        calipoint[centerfcalib].y=(ui->Lab_Image_C->y_pos)*IMAGE_HEIGHT/DISP_HEIGHT;
        centerfcalib++;
        qDebug()<<"第"<<centerfcalib<<"个记录标记点   "<<"x:"<<calipoint[centerfcalib-1].x<<"  y:"<<calipoint[centerfcalib-1].y;  //图像坐标
        //qDebug()<<ui->Lab_Image_C->x_pos<<ui->Lab_Image_C->y_pos;  窗口坐标
    }

    if(centerfcalib<16)
    {
        QMessageBox msg1;
        QString str=QString("第%1个标定点记录好了，请记录下一个标定点!").arg(centerfcalib);
        msg1.setText(str);
        msg1.exec();
    }

    if(centerfcalib==16)
    {
        QMessageBox msg2;
        msg2.setText("所有16个标定点记录完成，请点击开始标定!");
        msg2.exec();
    }

    ui->Lab_Image_C->update();
    return;
}

void MainWindow::on_Btn_PointCalib_clicked()
{
    if(centerfcalib==16)
    {
        ui->Lab_Image_C->boundset(calipoint);
        //ui->Lab_Image_C->boundset(&calipoint[0]);

        qDebug() << "发布标定点...";
        centerfcalib = 0;   //标定点计数清零
    }
    else
    {
        QMessageBox msg;
        QString str=QString("标定点数错误!!需要16个标定点，当前记录%1个标定点!").arg(centerfcalib);
        msg.setText(str);
        msg.exec();
    }

    ui->Lab_Image_C->update();
    return;
}

void MainWindow::on_Btn_PointDelone_clicked()
{
    if(centerfcalib>0)
    {
        centerfcalib--;
        calipoint[centerfcalib].x=0;
        calipoint[centerfcalib].y=0;
        QMessageBox msg1;
        QString str=QString("已撤销一个标定点，当前记录%1个标定点!").arg(centerfcalib);
        msg1.setText(str);
        msg1.exec();
    }
    else
    {
        QMessageBox msg2;
        msg2.setText("没有标定点了!!");
        msg2.exec();
    }

    ui->Lab_Image_C->update();
    return;
}

void MainWindow::on_Btn_PointDelall_clicked()
{
    for (int i=0; i<centerfcalib; i++)
    {
        calipoint[i].x=0;
        calipoint[i].y=0;
    }
    centerfcalib = 0;   //标定点计数清零
    QMessageBox msg;
    msg.setText("已撤销全部标定点!");
    msg.exec();
    ui->Lab_Image_C->update();
}


void MainWindow::get_Color_Enlarge(QRect rect)
{
    ui->Lab_ColorEnlarge->setScaledContents(true);
    QImage imgToEnlarge;
    imgToEnlarge = m_QImageForDisp.copy(rect.topLeft().x(),rect.topLeft().y(),rect.width(),rect.height());
    ui->Lab_ColorEnlarge->setPixmap(QPixmap::fromImage(imgToEnlarge));
    ui->Lab_ColorEnlarge->show();
}

void MainWindow::on_Btn_Sampling_clicked()
{
    ui->Lab_ColorEnlarge->colorAnalyse();

    return;
}

void MainWindow::on_Btn_ColorTest_clicked()
{
    QImage temp;
    QImage m_QImageForTest = m_QImageForDisp.copy(0, 0, m_QImageForDisp.width(), m_QImageForDisp.height());
    temp = ui->Lab_ColorData->colorTest(m_QImageForTest);
    qDebug() << "颜色测试的图像大小： " << m_QImageForTest.width() << "x" << m_QImageForTest.height();
    //小窗口(处理图像窗口)显示
    ui->Lab_Image_P->setPixmap(QPixmap::fromImage(temp));
    ui->Lab_Image_P->update();
    return;
}

void MainWindow::on_Btn_RePoints_clicked()
{
    /*
    for (int i=0; i<ui->Lab_ColorEnlarge->pointCount; i++)
    {
        ui->Lab_ColorEnlarge->point[i].x()=0;
        ui->Lab_ColorEnlarge->point[i].y()=0;
    }
    */
    ui->Lab_ColorEnlarge->pointCount=0;
    QMessageBox msg;
    msg.setText("已撤销全部采色点!");
    msg.exec();
    ui->Lab_ColorEnlarge->update();
}

void MainWindow::on_Btn_HSIcomplete_clicked()
{
    int HSIcolorSender[8*6];
    for(int i=0; i<8; i++)
    {
        for(int j=0; j<6; j++)
        {
            HSIcolorSender[i*6+j]=HSIThreshold[i][j];
        }
    }
    emit sendHSIcolor(HSIcolorSender);
    qDebug() << "发布颜色阈值...";
}

void MainWindow::initDecisionButtonGroup()
{
    ui->Sel_SelfAttack->setChecked(true);
    ui->Sel_LeftGround->setChecked(true);
    ui->Sel_GameMode0->setChecked(true);
    ui->Sel_BackInit->setChecked(true);
    ui->Sel_InitVar->setChecked(true);
}
void MainWindow::getRadioButtonID_A(int id)
{
    if (id == 0)
    {
        isAttack = true;
    }
    else
    {
        isAttack = false;
    }

    return;
}
void MainWindow::getRadioButtonID_H(int id)
{
    if (id == 0)
    {
        isRightArea = false;
    }
    else
    {
        isRightArea = true;
    }

    return;
}
void MainWindow::getRadioButtonID_G(int id)
{
    StartMode = id;

    return;
}
void MainWindow::getRadioButtonID_S(int id)
{
    if (id == 0)
    {
        isBackInitPose = true;
    }
    else
    {
        isBackInitPose = false;
    }

    return;
}
void MainWindow::getRadioButtonID_V(int id)
{
    if (id == 0)
    {
        isReInit = true;
    }
    else
    {
        isReInit = false;
    }

    return;
}

void MainWindow::on_Btn_ToStart_clicked()
{
    isSimuing = true;
    
    //开始比赛时停止订阅原始图像消息从而提高速度
//    if(commNode1->isToStop == 0)
//    {
//        commNode1->isToStop = 1;
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));//等待20ms关闭节点线程
//        if (commNode1 != NULL)
//        {
//            delete commNode1;
//        }
//        qDebug()<<"已经关闭GUI_subImgBefore节点!";
//    }
    
    isStartGame = true;
    emit sendDecisionvar();
}


void MainWindow::on_Btn_ToStop_clicked()
{
    isStartGame = false;
    emit sendDecisionvar();
}

