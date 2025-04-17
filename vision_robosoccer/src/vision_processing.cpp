#define _USE_MATH_DEFINES
#include <cmath>

#include "vision_processing.h"

#include "rclcpp/rclcpp.hpp"

#include "message_interface_robosoccer/msg/point_calibration.hpp"
#include "message_interface_robosoccer/msg/array_threshold.hpp"
#include "message_interface_robosoccer/msg/image.hpp"
#include "sensor_msgs/msg/image.hpp"
//#include "message_interface_robosoccer/msg/pose.hpp"
#include "message_interface_robosoccer/msg/pose_robots.hpp"
//#include "message_interface_robosoccer/msg/pose.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

class Vision_Processing : public rclcpp::Node
{
public:
    Vision_Processing(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s已创建!", name.c_str());

        InitInfoLUT();
        
        camera_calibration_point_sub_ = this->create_subscription<message_interface_robosoccer::msg::PointCalibration>("camera_calibration_point",1, std::bind(&Vision_Processing::camera_calibration_point_sub_callback, this, std::placeholders::_1));
        robot_colorthreshold_array_sub_ = this->create_subscription<message_interface_robosoccer::msg::ArrayThreshold>("robot_colorthreshold_array",1, std::bind(&Vision_Processing::robot_colorthreshold_array_sub_callback, this, std::placeholders::_1));
        //需要设置一个标志，完成上面操作才能执行小车位置发布/处理图像发布
        isGetCaliPoints = 0;
        isGetColrthhold = 0;
        
        //srcX.create(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32F);
        //srcY.create(IMAGE_HEIGHT,IMAGE_WIDTH,CV_32F);
        
        pointV_num = 0;
        
        processed_counter = 0;
        
//        isGetImage1 = 0;
        isGetImage2 = 0;
//        robot_image_sub1_ = this->create_subscription<sensor_msgs::msg::Image>("robot_image",2, 
//                                                                               std::bind(&Vision_Processing::image_sub_callback1, this, std::placeholders::_1));
        robot_image_sub2_ = this->create_subscription<sensor_msgs::msg::Image>("robot_image",1,
                                                                               std::bind(&Vision_Processing::image_sub_callback2, this, std::placeholders::_1));
        
        //sleep(2);
        //sub_frame_id_cp = std::to_string(0);
        is_Send_Opp_Pose = true;
        for(int i=0; i<11; i++)
        {
            pose_processed.pose[i].x = 0;
            pose_processed.pose[i].y = 0;
            pose_processed.pose[i].theta = 0;
        }
//        robot_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("robot_image_processed",1);
        robot_pose_pub_ = this->create_publisher<message_interface_robosoccer::msg::PoseRobots>("robots_pose",1);
    }
    ~Vision_Processing()
    {
        //释放查找表数组
        if (ground.groundInfo != NULL) 
        {
            for (int i = 0; i <IMAGE_WIDTH; i++) 
            {
                if (ground.groundInfo[i] != NULL)
                {
                    delete[] ground.groundInfo[i];
                }
            }
            delete[] ground.groundInfo;
        }
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
        }
//        if (nNewi != NULL)
//        {
//            for (int i=0; i<IMAGE_WIDTH; i++)
//            {
//                if (nNewi[i] != NULL)
//                {
//                    delete[] nNewi[i];
//                }
//            }
//            delete[] nNewi;
//        }
//        if (nNewj != NULL)
//        {
//            for (int i=0; i<IMAGE_WIDTH; i++)
//            {
//                if (nNewj[i] != NULL)
//                {
//                    delete[] nNewj[i];
//                }
//            }
//            delete[] nNewj;
//        }
    }
private:
    rclcpp::Subscription<message_interface_robosoccer::msg::PointCalibration>::SharedPtr camera_calibration_point_sub_;//订阅GUI上确定的标定点坐标，然后进行相机标定
    rclcpp::Subscription<message_interface_robosoccer::msg::ArrayThreshold>::SharedPtr robot_colorthreshold_array_sub_;//订阅GUI上确定的小车颜色阈值数组
    bool isGetCaliPoints;
    bool isGetColrthhold;
    
    // 标定映射参数
    //cv::Mat srcX; 
    //cv::Mat srcY;
    
//    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robot_image_sub1_;  //图像标定
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robot_image_sub2_;  //小车识别
    
//    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr robot_image_pub_;
    rclcpp::Publisher<message_interface_robosoccer::msg::PoseRobots>::SharedPtr robot_pose_pub_;
    
    //std::string sub_frame_id_cp;
    cv::Mat image_processed;
    message_interface_robosoccer::msg::PoseRobots pose_processed;
    
    unsigned long processed_counter;
    
//    bool isGetImage1;
    bool isGetImage2;
    
    //-------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------

    std::array<int32_t, 8*6> HSIThreshold;//int HSIThreshold[8][6]; //[i][j]->[i*6+j] //可自动释放 //HSI阈值表
    int*** HLUT;                  //RGB转HSI查找表
    Ground ground;                //场地信息表

    //图像标定用
//    int** nNewi;
//    int** nNewj;                 //图像映射表
    //相机标定用
    Point point[centerNum];       //标定点
    Camera_calib_param dcc;       //相机标定参数
    double xd[centerNum], yd[centerNum], rd2[centerNum];
    //标定点世界坐标
    double xw[centerNum]={0,110,220,220,220,220,110,0,0,0,55,110,165,165,110,55};
    double yw[centerNum]={4,0,4,50,130,176,180,176,130,50,30,65,30,150,115,150};

    //小车识别用
    double m_theta;  //主色标方向角，使用坐标转换后的世界坐标求出，准确度较高
    double NormalTheta[20] = {0};
    
    RoboPoint_img m_TargetN;  //主色标位置，使用图像色块坐标求出重心后再转换为世界坐标，偏差不大，且对图像中像素处理有帮助，后续也可以考虑用色标一样的方法求出<现在作为像素处理辅助点>
    Point_img TeamTarget[20];
    Point m_TargetN_true;  //主色标位置，直接将色块所有像素点转换为世界坐标后直接求出重心，排除了相机畸变的干扰，位置比 m_TargetN 转换后的更准确  <现在作为真实位置点>
    Point TeamTarget_true[20];
    //可以通过查找“旧方法”“新方法”进行方式切换，若坐标有问题可来此看一看
    
    std::vector<Point_img> pointV; //可自动释放
    int pointV_num;
    
    int m_xLeft, m_xRight, m_yTop, m_yBottom;
    int blackSum[4] = {0};
    message_interface_robosoccer::msg::PoseRobots pose_processed_last;
    message_interface_robosoccer::msg::PoseRobots pose_unIdentified;
    message_interface_robosoccer::msg::PoseRobots pose_notCorrectlyFound;

    bool is_Send_Opp_Pose; //是否发送对手坐标信息
    
    //-------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------
        
    //for image process
    void InitInfoLUT()
    {
        //初始化场地信息表-标定用，ground.groundInfo[col][row]!!!
        ground.groundInfo = (Point **)malloc(IMAGE_WIDTH*sizeof(Point *));
        for(int i=0;i<IMAGE_WIDTH;i++)
        {
            if(ground.groundInfo)
            {
                ground.groundInfo[i]= (Point *)malloc(IMAGE_HEIGHT*sizeof(Point));
            }
        }
        if(ground.groundInfo==NULL)
        {
            RCLCPP_INFO(this->get_logger(), "groundInfo无法动态申请内存!!");
        }
        
        //初始化颜色阈值表-小车识别用
        for(int i=0;i<8;i++)
        {
            HSIThreshold[i*6+0]=0;
            HSIThreshold[i*6+1]=0;//3600
            HSIThreshold[i*6+2]=0;
            HSIThreshold[i*6+3]=0;//100
            HSIThreshold[i*6+4]=0;
            HSIThreshold[i*6+5]=0;//255
        }
        
        //初始化HSI颜色转换表-颜色空间转换用
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
            RCLCPP_INFO(this->get_logger(), "HLUT无法动态申请内存!!");
        }
        else
        {
            //RGB转换H
            for(int r=0;r<256;++r)
                for(int g=0;g<256;++g)
                    for(int b=0;b<256;++b)
                        HLUT[r][g][b]=(int)changeColorFromRGBtoHSI(r,g,b);
        }
        
        //初始化图像映射表-标定图像用
//        nNewi = (int** )malloc(IMAGE_WIDTH * sizeof(int* ));
//        nNewj = (int** )malloc(IMAGE_WIDTH * sizeof(int* ));
//        for(int i=0;i<IMAGE_WIDTH;i++)
//        {
//            if(nNewi)
//            {
//                nNewi[i] = (int* )malloc(IMAGE_HEIGHT * sizeof(int));
//            }
//            if(nNewj)
//            {
//                nNewj[i] = (int* )malloc(IMAGE_HEIGHT * sizeof(int));
//            }
//        }
//        if(nNewi == NULL || nNewj == NULL)
//        {
//            RCLCPP_INFO(this->get_logger(), "图像映射表无法动态申请内存!!");
//        }
//        else
//        {
//            for(int i=0;i<IMAGE_WIDTH;++i)
//            {
//                for(int j=0;j<IMAGE_HEIGHT;++j)
//                {
//                    nNewi[i][j]=0;
//                    nNewj[i][j]=0;
//                }
//            }
//        }
//        
    }
    double changeColorFromRGBtoHSI(int r, int g, int b)
    {
        double H = 0;
        double R, G, B;
        R = double(r);
        G = double(g);
        B = double(b);
        if (R == G && G == B)
        {
            H = 0;
        }
        else
        {
            if (G >= B)
            {
                H = acos(((R - G) + (R - B)) / 2 / sqrt(((R - G) * (R - G)) + (R - B) * (R - B))) * 180 / Pi;
            }
            else
                H = 360 - acos(((R - G) + (R - B)) / 2 / sqrt((R - G) * (R -G) + (R - B) * (R - B))) * 180 / Pi;
        }
        return H;
    }
    
    void camera_calibration_point_sub_callback(message_interface_robosoccer::msg::PointCalibration::SharedPtr Points_)
    {
        //isGetCaliPoints = false;
        for (int i=0; i<centerNum; i++)
        {
            point[i].x = Points_->point[i].x;
            point[i].y = Points_->point[i].y;
        }
        RCLCPP_INFO(this->get_logger(), "已接收到标定点!");
        camera_calibration();
        if(isGetCaliPoints == false)
            isGetCaliPoints = true;
        RCLCPP_INFO(this->get_logger(), "标定成功!");
    }
    void robot_colorthreshold_array_sub_callback(message_interface_robosoccer::msg::ArrayThreshold::SharedPtr Array_)
    {
        //isGetColrthhold = false;
        //HSIThreshold = Array_->hsithreshold;
        for (int i=0; i<8; i++)
        {
            for (int j=0; j<6; j++)
            {
                HSIThreshold[i*6+j] = Array_->hsithreshold[i*6+j];
            }
        }
        RCLCPP_INFO(this->get_logger(), "已接收到颜色阈值!");
        if(isGetColrthhold == false)
            isGetColrthhold = true;
        //for (int i=0; i<8; i++)
        //    for (int j=0; j<6; j++)
        //        HSIThreshold[i][j] = Array_->hsithreshold[i*6+j];
    }
    void camera_calibration()
    {   
        //相机内外参数计算
        dcc.cx=IMAGE_WIDTH/2;
        dcc.cy=IMAGE_HEIGHT/2;
        dcc.f=-11.2512*Work_Distance/2500;//使用的大恒相机像素尺寸5.86umx5.86um，分辨率1920x1200,CCD靶面宽度v为5.86x1920x1e-3=11.2512mm，f = v*D/W, 场地最大宽度W为2500mm，工作距离D约为2500mm，调整相机时要尽量让场地的ground_width(x轴)刚好填满图像的image_width(x轴)
        compute_xd_yd_rd2();
        compute_Rz_Tx_Ty();
        compute_sx();
        Compute_Rx_Ry_Tz_k1();
        
        //相机图像像素(i,j)在标定后的位置(ground.groundInfo[i][j].x, ground.groundInfo[i][j].y)<真实位置，非整数> 位置坐标(x--列, y--行)
        //可以考虑做成函数 void ground_calibration(Camera_calib_param ccp)
        double zw=0;
        for(int i=0;i<IMAGE_WIDTH;i++)
        {
            for(int j=0;j<IMAGE_HEIGHT;j++)
            {
                float xdd,ydd,rd22,xdu,ydu,com_deno,temp1,temp2,temp3,temp4;

                xdd=(i-dcc.cx)*5.86*1e-3/dcc.sx;
                ydd = (j - dcc.cy) * 5.86 * 1e-3;
                rd22 = SQR(xdd) + SQR(ydd);
                xdu = xdd / (1 + dcc.k1 * rd22);//考虑径向畸变
                ydu = ydd / (1 + dcc.k1 * rd22);
                zw = -4.8;


                com_deno = xdu * dcc.Ry - dcc.f - ydu * dcc.Rx;
                temp1 = zw * ((dcc.Rx * (dcc.Ry * sin(dcc.Rz) - dcc.Rx * cos(dcc.Rz)) - cos(dcc.Rz)) * xdu
                        + (dcc.Ry * (dcc.Ry * sin(dcc.Rz) - dcc.Rx * cos(dcc.Rz)) + sin(dcc.Rz)) * ydu
                        - dcc.f * (dcc.Rx * sin(dcc.Rz) + dcc.Ry * cos(dcc.Rz)));
                temp2 = xdu * (dcc.Ty * (dcc.Ry * sin(dcc.Rz) - dcc.Rx * cos(dcc.Rz)) - dcc.Tz * cos(dcc.Rz))
                        + ydu * (dcc.Tx * (dcc.Rx * cos(dcc.Rz) - dcc.Ry * sin(dcc.Rz)) + dcc.Tz * sin(dcc.Rz))
                        + dcc.f * (dcc.Tx * cos(dcc.Rz) - dcc.Ty * sin(dcc.Rz));
                temp3 = (xdu * (dcc.Rx * (dcc.Ry * cos(dcc.Rz) + dcc.Rx * sin(dcc.Rz)) + sin(dcc.Rz))
                        + ydu * (dcc.Ry * (dcc.Ry * cos(dcc.Rz) + dcc.Rx * sin(dcc.Rz)) + cos(dcc.Rz))
                        - dcc.f * cos(dcc.Rz) * dcc.Rx + dcc.f * dcc.Ry * sin(dcc.Rz)) * zw;
                temp4 = xdu * (dcc.Ty * (dcc.Ry * cos(dcc.Rz) + dcc.Rx * sin(dcc.Rz)) + dcc.Tz * sin(dcc.Rz)) +
                        ydu * (dcc.Tz * cos(dcc.Rz) - dcc.Tx * dcc.Ry * cos(dcc.Rz) - dcc.Tx * dcc.Rx * sin(dcc.Rz)) +
                        dcc.f * (-cos(dcc.Rz) * dcc.Ty - dcc.Tx * sin(dcc.Rz));

                ground.groundInfo[i][j].x=(temp1+temp2)/com_deno; //Xw
                ground.groundInfo[i][j].y=-(temp3+temp4)/com_deno;//Yw
                
                //前向映射
//                nNewi[i][j] = int(ground.groundInfo[i][j].x-GROUND_WIDTH/2)+IMAGE_WIDTH/2;
//                nNewj[i][j] = int(ground.groundInfo[i][j].y-GROUND_HEIGHT/2)+IMAGE_HEIGHT/2;
                //建立一个画板将image(i,j)处的颜色绘制到image_after(nNewi,nNewj) 位置坐标(x--列, y--行) i列 j行
                
                /*
                //前向映射，remap函数暂不支持
                srcX.at<float>(j, i) = (temp1+temp2)/com_deno;
                srcY.at<float>(j, i) = -(temp3+temp4)/com_deno;
                */
                /*
                //后向映射，优于前向映射，暂未实现
                //将image(i,j)处的颜色绘制到image_after(i,j) 位置坐标(x--列, y--行)
                srcX.at<float>(j, i) = ???;
                srcY.at<float>(j, i) = ???;
                //
                */
            }
        } 
    
    }
    void compute_xd_yd_rd2()
    {
        for(int i=0;i<centerNum;i++)
        {
            xd[i]=5.86*1e-3*(point[i].x-dcc.cx);
            yd[i]=5.86*1e-3*(point[i].y-dcc.cy);
            rd2[i]=SQR(xd[i]) + SQR(yd[i]);
        }

        return;
    }
    void compute_Rz_Tx_Ty()
    {
        int i=0,j=0;
        int m = centerNum, n=3 ,ka =centerNum+1;
        double x[3], v[9], eps=1e-7;
        double* a, * aa,* u;

        a = new double[m*n];
        aa = new double[m*n];
        u = new double[m*m];
        for(i=0;i<m;i++)
        {
            for(j=0;j<n;j++)
            {
                switch (j) 
                {
                    case 0:
                        a[i*n+j]=xd[i]*yw[i]-xw[i]*yd[i];
                        break;
                    case 1:
                        a[i*n+j]=-(xd[i]*xw[i]+yw[i]*yd[i]);
                        break;
                    case 2:
                        a[i*n+j]=xd[i];
                    default:
                        break;
                }
            }
        }
        gmiv(a,m,n,yd,x,aa,eps,u,v,ka);//超定方程求解器
        //求解Rz Tx Ty
        dcc.Rz = atan2(x[1],x[0]);
        dcc.Tx = cos(dcc.Rz)/x[0];
        dcc.Ty = x[2] * dcc.Tx;
        if (a != NULL)
            delete[] a;
        if (aa != NULL)
            delete[] aa;
        if (u != NULL)
            delete[] u;

        return;
    }
    void compute_sx()
    {
        int m = centerNum, n = 5, ka = centerNum + 1;
        double a[80], x[5], aa[80], u[256], v[25], eps = 1e-7;

        for (int i = 0;i < m;i++)
        {
            for (int j = 0;j < n;j++)
            {
                switch (j)
                {
                    case 0: a[i * n + j] = yd[i] * xw[i];
                        break;
                    case 1: a[i * n + j] = yd[i] * yw[i];
                        break;
                    case 2: a[i * n + j] = yd[i];
                        break;
                    case 3: a[i * n + j] = -xd[i] * xw[i];
                        break;
                    case 4: a[i * n + j] = -xd[i] * yw[i];
                        break;
                }
            }
        }
        gmiv(a, m, n, xd, x, aa, eps, u, v, ka);
        dcc.sx = sqrt(SQR(x[0]) + SQR(x[1])) * fabs(dcc.Ty);

        return;
    }
    void Compute_Rx_Ry_Tz_k1()
    {
        int i = 0, j = 0;
        double yc[centerNum];
        for (i = 0;i < centerNum;i++)
        {
            yc[i] = -sin(dcc.Rz) * xw[i] + yw[i] * cos(dcc.Rz) + dcc.Ty;
        }

        int n = 4, m = centerNum, ka = centerNum + 1;
        double x[4], v[16], eps = 1e-7;
        double* a, * aa, * u, * b;

        a = new double[m * n];
        aa = new double[m * n];
        u = new double[m * m];
        b = new double[m];
        for (i = 0;i < m; i++)
        {
            for (j = 0; j < n; j++)
            {
                switch (j)
                {
                    case 0:
                        a[i * n + j] = -dcc.f * rd2[i] * yc[i];
                        break;
                    case 1:
                        a[i * n + j] = -cos(dcc.Rz) * yw[i] * yd[i] + sin(dcc.Rz) * xw[i] * yd[i];
                        break;
                    case 2:
                        a[i * n + j] = yd[i];
                        break;
                    case 3:
                        a[i * n + j] = (cos(dcc.Rz) * xw[i] + sin(dcc.Rz) * yw[i]) * yd[i];
                        break;
                    default:
                        break;
                }
            }
            b[i] = yc[i] * dcc.f;
        }
        gmiv(a, m, n, b, x, aa, eps, u, v, ka);
        dcc.k1 = x[0];
        dcc.Rx = x[1];
        dcc.Tz = x[2];
        dcc.Ry = x[3];
        if (a!=NULL)
            delete[] a;
        if (aa!=NULL)
            delete[] aa;
        if (u!=NULL)
            delete[] u;
        if (b!=NULL)
            delete[] b;

        return;
    }
    //超定方程求解器-最小二乘的广义逆法求解超定线性方程组
    int gmiv(double a[],int m,int n,double b[],double x[],double aa[],double eps,double u[],double v[],int ka)
    {
        int i,j;
        i=ginv(a,m,n,aa,eps,u,v,ka);
        if (i<0) 
            return(-1);
        for (i=0; i<=n-1; i++)
        {
            x[i]=0.0;
            for (j=0; j<=m-1; j++)
                x[i]=x[i]+aa[i*m+j]*b[j];
        }
        return(1);
    }
    int ginv(double a[],int m,int n,double aa[],double eps,double u[],double v[],int ka)
    {
        int i,j,k,l,t,p,q,f;
        i=muav(a,m,n,u,v,eps,ka);
        if (i<0) 
            return(-1);
        j=n;
        if (m<n) 
            j=m;
        j=j-1;
        k=0;
        while ((k<=j)&&(a[k*n+k]!=0.0)) 
            k=k+1;
        k=k-1;
        for (i=0; i<=n-1; i++)
            for (j=0; j<=m-1; j++)
            { 
                t=i*m+j; aa[t]=0.0;
                for (l=0; l<=k; l++)
                { 
                    f=l*n+i; p=j*m+l; q=l*n+l;
                    aa[t]=aa[t]+v[f]*u[p]/a[q];
                }
            }
        return(1);
    }
    int muav(double a[],int m,int n,double u[],double v[],double eps,int ka)
    { 
        int i,j,k,l,it,ll,kk,ix,iy,mm,nn,iz,m1,ks;
        double d,dd,t,sm,sm1,em1,sk,ek,b,c,shh,fg[2],cs[2];
        double *s,*e,*w;

        s=(double *)malloc(ka*sizeof(double));
        e=(double *)malloc(ka*sizeof(double));
        w=(double *)malloc(ka*sizeof(double));
        it=60; 
	    k=n;
        if (m-1<n) 
		    k=m-1;
        l=m;
        if (n-2<m) 
		    l=n-2;
        if (l<0) 
		    l=0;
        ll=k;
        if (l>k) 
		    ll=l;
        if (ll>=1)
        { 
		    for (kk=1; kk<=ll; kk++)
            { 
			    if (kk<=k)
                { 
				    d=0.0;
                    for (i=kk; i<=m; i++)
                    { 
					    ix=(i-1)*n+kk-1; d=d+a[ix]*a[ix];
				    }
                    s[kk-1]=sqrt(d);
                    if (s[kk-1]!=0.0)
                    { 
					    ix=(kk-1)*n+kk-1;
                        if (a[ix]!=0.0)
                        { 
						    s[kk-1]=fabs(s[kk-1]);
                            if (a[ix]<0.0) 
							    s[kk-1]=-s[kk-1];
                        }
                        for (i=kk; i<=m; i++)
                        { 
						    iy=(i-1)*n+kk-1;
                            a[iy]=a[iy]/s[kk-1];
                        }
                        a[ix]=1.0+a[ix];
                    }
                    s[kk-1]=-s[kk-1];
                }
                if (n>=kk+1)
                { 
				    for (j=kk+1; j<=n; j++)
                    { 
					    if ((kk<=k)&&(s[kk-1]!=0.0))
                        { 
						    d=0.0;
                            for (i=kk; i<=m; i++)
                            { 
							    ix=(i-1)*n+kk-1;
                                iy=(i-1)*n+j-1;
                                d=d+a[ix]*a[iy];
                            }
                            d=-d/a[(kk-1)*n+kk-1];
                            for (i=kk; i<=m; i++)
                            { 
							    ix=(i-1)*n+j-1;
                                iy=(i-1)*n+kk-1;
                                a[ix]=a[ix]+d*a[iy];
                            }
                        }
                        e[j-1]=a[(kk-1)*n+j-1];
                    }
                }
                if (kk<=k)
                { 
				    for (i=kk; i<=m; i++)
                    { 
					    ix=(i-1)*m+kk-1; iy=(i-1)*n+kk-1;
                        u[ix]=a[iy];
                    }
                }
                if (kk<=l)
                { 
				    d=0.0;
                    for (i=kk+1; i<=n; i++)
                      d=d+e[i-1]*e[i-1];
                    e[kk-1]=sqrt(d);
                    if (e[kk-1]!=0.0)
                    { 
					    if (e[kk]!=0.0)
                        { 
						    e[kk-1]=fabs(e[kk-1]);
                            if (e[kk]<0.0) 
							    e[kk-1]=-e[kk-1];
                        }
                        for (i=kk+1; i<=n; i++)
                          e[i-1]=e[i-1]/e[kk-1];
                        e[kk]=1.0+e[kk];
                    }
                    e[kk-1]=-e[kk-1];
                    if ((kk+1<=m)&&(e[kk-1]!=0.0))
                    { 
					    for (i=kk+1; i<=m; i++) 
						    w[i-1]=0.0;
                        for (j=kk+1; j<=n; j++)
                          for (i=kk+1; i<=m; i++)
                            w[i-1]=w[i-1]+e[j-1]*a[(i-1)*n+j-1];
                        for (j=kk+1; j<=n; j++)
                          for (i=kk+1; i<=m; i++)
                          { 
						      ix=(i-1)*n+j-1;
                              a[ix]=a[ix]-w[i-1]*e[j-1]/e[kk];
                          }
                    }
                    for (i=kk+1; i<=n; i++)
                      v[(i-1)*n+kk-1]=e[i-1];
                }
            }
        }
        mm=n;
        if (m+1<n) 
		    mm=m+1;
        if (k<n) 
		    s[k]=a[k*n+k];
        if (m<mm) 
		    s[mm-1]=0.0;
        if (l+1<mm) 
		    e[l]=a[l*n+mm-1];
        e[mm-1]=0.0;
        nn=m;
        if (m>n) 
		    nn=n;
        if (nn>=k+1)
        { 
		    for (j=k+1; j<=nn; j++)
            { 
			    for (i=1; i<=m; i++)
                  u[(i-1)*m+j-1]=0.0;
                u[(j-1)*m+j-1]=1.0;
            }
        }
        if (k>=1)
        { 
		    for (ll=1; ll<=k; ll++)
            { 
			    kk=k-ll+1; iz=(kk-1)*m+kk-1;
                if (s[kk-1]!=0.0)
                { 
				    if (nn>=kk+1)
                      for (j=kk+1; j<=nn; j++)
                      { 
					      d=0.0;
                          for (i=kk; i<=m; i++)
                          { 
						      ix=(i-1)*m+kk-1;
                              iy=(i-1)*m+j-1;
                              d=d+u[ix]*u[iy]/u[iz];
                          }
                          d=-d;
                          for (i=kk; i<=m; i++)
                          { 
						      ix=(i-1)*m+j-1;
                              iy=(i-1)*m+kk-1;
                              u[ix]=u[ix]+d*u[iy];
                          }
                      }
                    for (i=kk; i<=m; i++)
                    { 
				      ix=(i-1)*m+kk-1; 
				      u[ix]=-u[ix];
				    }
                    u[iz]=1.0+u[iz];
                    if (kk-1>=1)
                      for (i=1; i<=kk-1; i++)
                        u[(i-1)*m+kk-1]=0.0;
                }
                else
                { 
				    for (i=1; i<=m; i++)
                      u[(i-1)*m+kk-1]=0.0;
                    u[(kk-1)*m+kk-1]=1.0;
                }
            }
        }
        for (ll=1; ll<=n; ll++)
        { 
		    kk=n-ll+1; iz=kk*n+kk-1;
            if ((kk<=l)&&(e[kk-1]!=0.0))
            { 
			    for (j=kk+1; j<=n; j++)
                { 
				    d=0.0;
                    for (i=kk+1; i<=n; i++)
                    { 
					    ix=(i-1)*n+kk-1; iy=(i-1)*n+j-1;
                        d=d+v[ix]*v[iy]/v[iz];
                    }
                    d=-d;
                    for (i=kk+1; i<=n; i++)
                    { 
					    ix=(i-1)*n+j-1; iy=(i-1)*n+kk-1;
                        v[ix]=v[ix]+d*v[iy];
                    }
                }
            }
            for (i=1; i<=n; i++)
              v[(i-1)*n+kk-1]=0.0;
            v[iz-n]=1.0;
        }
        for (i=1; i<=m; i++)
          for (j=1; j<=n; j++)
            a[(i-1)*n+j-1]=0.0;
        m1=mm; it=60;
        while (1==1)
        { 
		    if (mm==0)
            { 
			    ppp(a,e,s,v,m,n);
                free(s); free(e); free(w); return(1);
            }
            if (it==0)
            { 
			    ppp(a,e,s,v,m,n);
                free(s); free(e); free(w); return(-1);
            }
            kk=mm-1;
	        while ((kk!=0)&&(fabs(e[kk-1])!=0.0))
            { 
			    d=fabs(s[kk-1])+fabs(s[kk]);
                dd=fabs(e[kk-1]);
                if (dd>eps*d) 
				    kk=kk-1;
                else 
				    e[kk-1]=0.0;
            }
            if (kk==mm-1)
            { 
			    kk=kk+1;
                if (s[kk-1]<0.0)
                { 
				    s[kk-1]=-s[kk-1];
                    for (i=1; i<=n; i++)
                    { 
					    ix=(i-1)*n+kk-1; v[ix]=-v[ix];
				    }
                }
                while ((kk!=m1)&&(s[kk-1]<s[kk]))
                { 
				    d=s[kk-1]; s[kk-1]=s[kk]; s[kk]=d;
                    if (kk<n)
                      for (i=1; i<=n; i++)
                      { 
					      ix=(i-1)*n+kk-1; iy=(i-1)*n+kk;
                          d=v[ix]; v[ix]=v[iy]; v[iy]=d;
                      }
                    if (kk<m)
                      for (i=1; i<=m; i++)
                      { 
					      ix=(i-1)*m+kk-1; iy=(i-1)*m+kk;
                          d=u[ix]; u[ix]=u[iy]; u[iy]=d;
                      }
                    kk=kk+1;
                }
                it=60;
                mm=mm-1;
            }
            else
            { 
			    ks=mm;
                while ((ks>kk)&&(fabs(s[ks-1])!=0.0))
                { 
				    d=0.0;
                    if (ks!=mm) 
					    d=d+fabs(e[ks-1]);
                    if (ks!=kk+1) 
					    d=d+fabs(e[ks-2]);
                    dd=fabs(s[ks-1]);
                    if (dd>eps*d) 
					    ks=ks-1;
                    else 
					    s[ks-1]=0.0;
                }
                if (ks==kk)
                { 
				    kk=kk+1;
                    d=fabs(s[mm-1]);
                    t=fabs(s[mm-2]);
                    if (t>d) 
					    d=t;
                    t=fabs(e[mm-2]);
                    if (t>d) 
					    d=t;
                    t=fabs(s[kk-1]);
                    if (t>d) 
					    d=t;
                    t=fabs(e[kk-1]);
                    if (t>d) 
					    d=t;
                    sm=s[mm-1]/d; sm1=s[mm-2]/d;
                    em1=e[mm-2]/d;
                    sk=s[kk-1]/d; ek=e[kk-1]/d;
                    b=((sm1+sm)*(sm1-sm)+em1*em1)/2.0;
                    c=sm*em1; c=c*c; shh=0.0;
                    if ((b!=0.0)||(c!=0.0))
                    { 
					    shh=sqrt(b*b+c);
                        if (b<0.0) 
						    shh=-shh;
                        shh=c/(b+shh);
                    }
                    fg[0]=(sk+sm)*(sk-sm)-shh;
                    fg[1]=sk*ek;
                    for (i=kk; i<=mm-1; i++)
                    { 
					    sss(fg,cs);
                        if (i!=kk) 
						    e[i-2]=fg[0];
                        fg[0]=cs[0]*s[i-1]+cs[1]*e[i-1];
                        e[i-1]=cs[0]*e[i-1]-cs[1]*s[i-1];
                        fg[1]=cs[1]*s[i];
                        s[i]=cs[0]*s[i];
                        if ((cs[0]!=1.0)||(cs[1]!=0.0))
                          for (j=1; j<=n; j++)
                          { 
						      ix=(j-1)*n+i-1;
                              iy=(j-1)*n+i;
                              d=cs[0]*v[ix]+cs[1]*v[iy];
                              v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
                              v[ix]=d;
                          }
                        sss(fg,cs);
                        s[i-1]=fg[0];
                        fg[0]=cs[0]*e[i-1]+cs[1]*s[i];
                        s[i]=-cs[1]*e[i-1]+cs[0]*s[i];
                        fg[1]=cs[1]*e[i];
                        e[i]=cs[0]*e[i];
                        if (i<m)
                          if ((cs[0]!=1.0)||(cs[1]!=0.0))
                            for (j=1; j<=m; j++)
                            { 
							    ix=(j-1)*m+i-1;
                                iy=(j-1)*m+i;
                                d=cs[0]*u[ix]+cs[1]*u[iy];
                                u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
                                u[ix]=d;
                            }
                    }
                    e[mm-2]=fg[0];
                    it=it-1;
                }
                else
                { 
				    if (ks==mm)
                    { 
					    kk=kk+1;
                        fg[1]=e[mm-2]; e[mm-2]=0.0;
                        for (ll=kk; ll<=mm-1; ll++)
                        { 
						    i=mm+kk-ll-1;
                            fg[0]=s[i-1];
                            sss(fg,cs);
                            s[i-1]=fg[0];
                            if (i!=kk)
                            { 
							    fg[1]=-cs[1]*e[i-2];
                                e[i-2]=cs[0]*e[i-2];
                            }
                            if ((cs[0]!=1.0)||(cs[1]!=0.0))
                              for (j=1; j<=n; j++)
                              { 
							      ix=(j-1)*n+i-1;
                                  iy=(j-1)*n+mm-1;
                                  d=cs[0]*v[ix]+cs[1]*v[iy];
                                  v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
                                  v[ix]=d;
                              }
                        }
                    }
                    else
                    { 
					    kk=ks+1;
                        fg[1]=e[kk-2];
                        e[kk-2]=0.0;
                        for (i=kk; i<=mm; i++)
                        { 
						    fg[0]=s[i-1];
                            sss(fg,cs);
                            s[i-1]=fg[0];
                            fg[1]=-cs[1]*e[i-1];
                            e[i-1]=cs[0]*e[i-1];
                            if ((cs[0]!=1.0)||(cs[1]!=0.0))
                              for (j=1; j<=m; j++)
                              { 
							      ix=(j-1)*m+i-1;
                                  iy=(j-1)*m+kk-2;
                                  d=cs[0]*u[ix]+cs[1]*u[iy];
                                  u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
                                  u[ix]=d;
                              }
                        }
                    }
                }
            }
        }
        if (s!=NULL)
            free(s); 
        if (e!=NULL)
            free(e); 
        if (w!=NULL)
            free(w);
        
        return(1);
    }
    static void ppp(double a[],double e[],double s[],double v[],int m,int n)
    { 
	    int i,j,p,q;
        double d;
        if (m>=n) 
            i=n;
        else i=m;
        for (j=1; j<=i-1; j++)
        { 
		    a[(j-1)*n+j-1]=s[j-1];
            a[(j-1)*n+j]=e[j-1];
        }
        a[(i-1)*n+i-1]=s[i-1];
        if (m<n) 
		    a[(i-1)*n+i]=e[i-1];
        for (i=1; i<=n-1; i++)
            for (j=i+1; j<=n; j++)
            { 
		        p=(i-1)*n+j-1; q=(j-1)*n+i-1;
                d=v[p]; 
			    v[p]=v[q]; 
			    v[q]=d;
            }
        return;
    }
    static void sss(double fg[],double cs[])
    { 
	    double r,d;
        if ((fabs(fg[0])+fabs(fg[1]))==0.0)
        { 
		    cs[0]=1.0; cs[1]=0.0; d=0.0;
	    }
        else 
        { 
		    d=sqrt(fg[0]*fg[0]+fg[1]*fg[1]);
            if (fabs(fg[0])>fabs(fg[1]))
            { 
			    d=fabs(d);
                if (fg[0]<0.0) d=-d;
            }
            if (fabs(fg[1])>=fabs(fg[0]))
            { 
			    d=fabs(d);
                if (fg[1]<0.0) d=-d;
            }
            cs[0]=fg[0]/d; cs[1]=fg[1]/d;
        }
        r=1.0;
        if (fabs(fg[0])>fabs(fg[1])) 
		    r=cs[1];
        else
          if (cs[0]!=0.0) 
		      r=1.0/cs[0];
        fg[0]=d; fg[1]=r;
        return;
    }
    
    
    //ROS2订阅者存在队列，不用担心新消息处理冲突，ROS2会进行调度分配
    //一旦一个消息开始处理，ROS2一定会将其处理完毕(除非节点中断)
    
    void image_sub_callback2(sensor_msgs::msg::Image::SharedPtr image_sub_)
    {
        if (isGetImage2==0)
        {
            RCLCPP_INFO(this->get_logger(), "我接收到了一帧图像!图片ID=%s", image_sub_->header.frame_id.c_str());
            //sub_frame_id_cp = image_sub_->header.frame_id;
            cv::Mat image4process(
              image_sub_->height, image_sub_->width,
              encoding2mat_type(image_sub_->encoding),
              const_cast<unsigned char *>(image_sub_->data.data()));
              //图片尺寸修改后图像(1920x1200)*0.5
            if (image4process.empty() || image4process.cols != (IMAGE_WIDTH*0.5) || image4process.rows != (IMAGE_HEIGHT*0.5))
            {
                RCLCPP_INFO(this->get_logger(), "图像接收错误!!");
                isGetImage2 = 0;
                return;
            }
            else
            {
                //cv::imshow("Image_to_Process", image4process);
                //cv::waitKey(0);
                
                
                //*The place to do image_processing*-------------------------------------------------------------
                
                if (isGetCaliPoints==true && isGetColrthhold==true)
                {
                    //图片尺寸修改，原始图像1920x1200
                    cv::resize(image4process, image4process, cv::Size(image4process.cols * 2, image4process.rows * 2), 0, 0, cv::INTER_LINEAR);
                    
                    cv::Mat image2process = image4process.clone();
                    //小车识别
                    pose_processed = robot_identify(image2process);
                    //后期考虑要不要图像标定只运行一次<不确定是否耗时,目前暂时是用映射表减少了一定的时间>，已完成
                    //以及图像标定要不要与小车识别异步发送<设置不同定时器>#已完成，并且两者独立，标定、识别分开，以加快后续任务速度
                    
                    isGetImage2 = 1;
                }
                else
                {
                    //RCLCPP_INFO(this->get_logger(), "还没有收到标定点和颜色阈值!");
                    isGetImage2 = 0;
                    return;
                }

            }
        }
        
        /****************************发布识别结果******************************************/
        if (isGetImage2==1)
        {   
            message_interface_robosoccer::msg::PoseRobots pose_to_pub;
            
            pose_to_pub = pose_processed;

            pose_processed_last = pose_processed;

            robot_pose_pub_->publish(pose_to_pub);
            
            //测试图像发布速度
            processed_counter++;
            if (processed_counter > 4000000000)
                processed_counter = 1;
//            RCLCPP_INFO(this->get_logger(), "第%ld次处理图像完成! | 1号车坐标：x:%f y:%f theta:%f | 2号车坐标：x:%f y:%f theta:%f | 3号车坐标：x:%f y:%f theta:%f |", 
//                                            processed_counter, pose_to_pub.pose[0].x, pose_to_pub.pose[0].y, pose_to_pub.pose[0].theta,
//                                                               pose_to_pub.pose[1].x, pose_to_pub.pose[1].y, pose_to_pub.pose[1].theta,
//                                                               pose_to_pub.pose[2].x, pose_to_pub.pose[2].y, pose_to_pub.pose[2].theta);
            RCLCPP_INFO(this->get_logger(), "第%ld次处理图像完成! | 1号车坐标：x:%f y:%f theta:%f |", 
                                            processed_counter, pose_to_pub.pose[0].x, pose_to_pub.pose[0].y, pose_to_pub.pose[0].theta);
            
            
            isGetImage2 = 0;
        }
        else
        {
            //RCLCPP_INFO(this->get_logger(), "等待新图像!");
        }
        /*********************************************************************************/
        
        return;
    }
    std::string mat_type2encoding(int mat_type)
    {
        switch (mat_type)
        {
            case CV_8UC1:
                return "mono8";
                break;
            case CV_8UC3:
                return "bgr8";          // ‘rgb8’
                break;
            case CV_16SC1:
                return "mono16";
                break;
            case CV_8UC4:
                return "rgba8";
                break;
            default:
                return "bgr8";
        }
    }
    int encoding2mat_type(std::string encoding)
    {
        if (encoding == "mono8") 
        {
            return CV_8UC1;
        } 
        else if (encoding == "bgr8")  //"rgb8"，此处颜色默认 b-g-r 的排列顺序各一字节8位
        {
            return CV_8UC3;  //8位3通道
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

    message_interface_robosoccer::msg::PoseRobots robot_identify(cv::Mat& image)
    {
        message_interface_robosoccer::msg::PoseRobots robots_pose;
        message_interface_robosoccer::msg::PoseRobots opp_robots_pose;
        
        /*robots_pose.pose[0].x = 10.0;
        robots_pose.pose[0].y = 15.0;
        robots_pose.pose[0].theta = Pi/2;
        return robots_pose;*/
        
        //-----------------------------------------------------
        
        int NumOpp, NumBall, NumTeam;
        NumOpp = 0;
        NumBall = 0;
        NumTeam = 0;
        
        bool isIdentifyOpp = false;
        
        RoboPoint_img TemTeam[20], TemOpp[20], TemBall[5], Tem;
        Point TemOpp_true[20], TemBall_true[5], Tem_true;
        
        
        for(int i = 0; i<IMAGE_WIDTH;i+= GRIDSIZE)
        {
            for(int j = 0;j<IMAGE_HEIGHT;j+= GRIDSIZE)
            {
                if(findPixel(TEAM,i,j,image))
                {
                    if(NumTeam<20 && searchTeam(TEAM,i,j,TEAMSIZEMIN,TEAMSIZEMAX,image))
                    {
                        if ((m_TargetN.x < 0) || (m_TargetN.x > (IMAGE_WIDTH - 1)) || (m_TargetN.y < 0) || (m_TargetN.y > (IMAGE_HEIGHT - 1)))
                            continue;
                        else
                        {
                            TemTeam[NumTeam] = m_TargetN;
                            
                            TeamTarget_true[NumTeam] = m_TargetN_true;
                            
                            NormalTheta[NumTeam] = m_theta;
                            NumTeam++;
                        }
                    }
                }

                else if(isIdentifyOpp && findPixel(OPP,i,j,image))
                {
                    if(NumOpp <10 && searchOppAndBall(OPP,i,j,OPPSIZEMIN,OPPSIZEMAX,image))
                    {
                        if((m_TargetN.x<0) || (m_TargetN.x > (IMAGE_WIDTH-1)) || (m_TargetN.y < 0) || (m_TargetN.y > (IMAGE_HEIGHT - 1)))
                            continue;
                        else
                        {
                            TemOpp[NumOpp] = m_TargetN;
                            
                            TemOpp_true[NumOpp] = m_TargetN_true;
                            
                            NumOpp++;
                        }
                    }
                }

                else if(NumBall<5 && findPixel(BALL,i,j,image))
                {
                    if(searchOppAndBall(BALL,i,j,BALLSIZEMIN,BALLSIZEMAX,image))
                    {
                        if ((m_TargetN.x < 0) || (m_TargetN.x > (IMAGE_WIDTH - 1)) || (m_TargetN.y < 0) || (m_TargetN.y > (IMAGE_HEIGHT - 1)))
                            continue;
                        else
                        {
                            TemBall[NumBall] = m_TargetN;
                            
                            TemBall_true[NumBall] = m_TargetN_true;
                            
                            NumBall++;
                        }
                    }
                }
            }
        }

        for(int i=0;i<NumTeam;i++)
        {
            TeamTarget[i] = Point_img(TemTeam[i].x, TemTeam[i].y);
        }

        //己方信息
        robots_pose = identiRobo(NumTeam, image);

        //对手信息
        if (NumOpp <= 5)
        {
            for (int k = 0;k < NumOpp;k++)
            {
                //旧方法：色块重心----坐标转换---->物体世界坐标重心
                //opp_robots_pose.pose[k].x = ground.groundInfo[TemOpp[k].x][TemOpp[k].y].x;
                //opp_robots_pose.pose[k].y = ground.groundInfo[TemOpp[k].x][TemOpp[k].y].y;
                
                //新方法：色块所有像素世界坐标----求重心---->物体世界坐标重心，推荐使用，更准确
                opp_robots_pose.pose[k].x = TemOpp_true[k].x;
                opp_robots_pose.pose[k].y = TemOpp_true[k].y;
            }
        }
        else
        {
            for (int k = 0;k < NumOpp - 1;k++)
            {
                for (int n = k;n < NumOpp;n++)
                {
                    if (TemOpp[k].num < TemOpp[n].num)
                    {
                        Tem = TemOpp[k];
                        TemOpp[k] = TemOpp[n];
                        TemOpp[n] = Tem;
                        
                        Tem_true = TemOpp_true[k];
                        TemOpp_true[k] = TemOpp_true[n];
                        TemOpp_true[n] = Tem_true;
                    }
                }
            }
            for (int k = 0;k < 5;k++)
            {
                //旧方法：色块重心----坐标转换---->物体世界坐标重心
                //opp_robots_pose.pose[k].x = ground.groundInfo[TemOpp[k].x][TemOpp[k].y].x;
                //opp_robots_pose.pose[k].y = ground.groundInfo[TemOpp[k].x][TemOpp[k].y].y;
                
                //新方法：色块所有像素世界坐标----求重心---->物体世界坐标重心，推荐使用，更准确
                opp_robots_pose.pose[k].x = TemOpp_true[k].x;
                opp_robots_pose.pose[k].y = TemOpp_true[k].y;
            }
        }

        //球信息
        if (NumBall >= 1)
        {
            for (int i = 0;i < NumBall;i++)
            {
                if (TemBall[0].num < TemBall[i].num)
                {
                    TemBall[0] = TemBall[i];
                    
                    TemBall_true[0] = TemBall_true[i];
                }
                    
            }
            //旧方法：色块重心----坐标转换---->物体世界坐标重心
            //robots_pose.pose[10].x = ground.groundInfo[TemBall[0].x][TemBall[0].y].x;
            //robots_pose.pose[10].y = ground.groundInfo[TemBall[0].x][TemBall[0].y].y;
            
            //新方法：色块所有像素世界坐标----求重心---->物体世界坐标重心，推荐使用，更准确
            robots_pose.pose[10].x = TemBall_true[0].x;
            robots_pose.pose[10].y = TemBall_true[0].y;
        }
        else
        {
            BallPosFilter(&robots_pose, &opp_robots_pose);
        }
        
        //-----------------------------------------------------
        if (is_Send_Opp_Pose)
        {
            for (int i=0; i<5; i++)
            {
                robots_pose.pose[i+5].x = opp_robots_pose.pose[i].x;
                robots_pose.pose[i+5].y = opp_robots_pose.pose[i].y;
            }
        }
        //-----------------------------------------------------
        
        return robots_pose;
    }
    //2024.3.30记录：记得修改cv::Mat的拷贝，当前均为浅拷贝，部分地方需要修改为深拷贝，注意！！！
    //2024.4.1记录：好像也可以不用修改，浅拷贝不会造成影响，暂时不修改，后续有问题再来看看
    bool findPixel(int object, int m, int n, cv::Mat& P)
    {   
        int R, G, B, H, S, I;
        R = P.at<cv::Vec3b>(n, m)[2];   // n 行 m 列
        G = P.at<cv::Vec3b>(n, m)[1];
        B = P.at<cv::Vec3b>(n, m)[0];
        
        if (R < 0 || R>255 || G < 0 || G>255 || B < 0 || B>255)
            return false;
        H = 10 * HLUT[R][G][B];
        if(H<=0)
            return false;
        if(HSIThreshold[object*6+1]>=HSIThreshold[object*6+0])
        {
            if (H >= HSIThreshold[object*6+0] && H <= HSIThreshold[object*6+1])
            {
                S = int(100 * (1 - 3.0 * countMin(R, G, B, 3) / (double(R) + double(G) + double(B))));
                if (S >= HSIThreshold[object*6+2] && S <= HSIThreshold[object*6+3])
                {
                    I = (int)((R + G + B) / 3);
                    if (I >= HSIThreshold[object*6+4] && I <= HSIThreshold[object*6+5])
                    {
                        return true;
                    }
                    else
                        return false;
                }
                else
                    return false;
            }
            else
                return false;
        }
        else if (HSIThreshold[object*6+1] < HSIThreshold[object*6+0])
        {
            if (H >= HSIThreshold[object*6+0] || H <= HSIThreshold[object*6+1])
            {
                S = int(100 * (1 - 3.0 * countMin(R, G, B, 3) / (double(R) + double(G) + double(B))));
                if (S >= HSIThreshold[object*6+2] && S <= HSIThreshold[object*6+3])
                {
                    I = (int)((R + G + B) / 3);
                    if (I >= HSIThreshold[object*6+4] && I <= HSIThreshold[object*6+5])
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false;
    }
    bool searchTeam(int tab, int startX, int startY, int sizeMin, int sizeMax, cv::Mat& P)
    {
        int ii, jj;
        double k;
        int sum, sumx, sumy;
        double sumupx, sumdownx, sumrighty, sumlefty;
        double sumleftupx, sumleftupy, sumrightupx, sumrightupy, sumleftdownx, sumleftdowny, sumrightdownx, sumrightdowny;
        int sumleftup, sumleftdown, sumrightup, sumrightdown, sumup, sumdown, sumright, sumleft;
        int x, y, y1;
        double a, b, c, averx, avery;
        bool spanLeft, spanRight;
        double xerror, yerror;
        double sumx1, sumy1, sumxx, sumxy, sumyy;
        double sumx1_W, sumy1_W;
        
        k = 0;
        
        sum = sumx = sumy = 0;
        sumx1 = sumy1 = sumxx = sumxy = sumyy = 0;
        sumx1_W = sumy1_W = 0;

        x = startX;
        y = startY;
        
        m_xLeft = m_xRight = x;
        m_yTop = m_yBottom =y;
        
        pointV.clear();
        push(Point_img(x,y));
            
        while(pop(x,y))
        {
            y1= y;
            while(findPixel(tab,x,y1,P) && y1>0)
                y1--;
            y1++;
            spanLeft = spanRight = 0;
            while(findPixel(tab,x,y1,P) && y1<IMAGE_HEIGHT-1)
            {
                P.at<cv::Vec3b>(y1, x)[2] = 100;   // y1 行 x 列
                P.at<cv::Vec3b>(y1, x)[1] = 100;
                P.at<cv::Vec3b>(y1, x)[0] = 100;

                if(x <= m_xLeft)
                    m_xLeft = x;
                else if(x >= m_xRight)
                    m_xRight = x;
                if(y1 <= m_yTop)
                    m_yTop = y1;
                else if(y1 >= m_yBottom)
                    m_yBottom = y1;

                sum++;
                sumx = sumx + x;
                sumy = sumy + y1;
                //图像系
                
                sumx1 = sumx1 + ground.groundInfo[x][y1].x;
                sumy1 = sumy1 + (double(GROUND_HEIGHT) - double(ground.groundInfo[x][y1].y));
                sumxx = sumxx + double(ground.groundInfo[x][y1].x)*double(ground.groundInfo[x][y1].x);
                sumxy = sumxy + double(ground.groundInfo[x][y1].x)*(double(GROUND_HEIGHT) - double(ground.groundInfo[x][y1].y));
                sumyy = sumyy + (double(GROUND_HEIGHT) - double(ground.groundInfo[x][y1].y))*(double(GROUND_HEIGHT) - double(ground.groundInfo[x][y1].y));
                //策略系
                
                sumx1_W = sumx1_W + ground.groundInfo[x][y1].x;
                sumy1_W = sumy1_W + ground.groundInfo[x][y1].y;
                //世界系

                if(!spanLeft && x>0 && findPixel(tab,x-1,y1,P))
                {
                    if(!push(Point_img(x-1,y1)))
                        return false;
                    spanLeft = 1;

                }
                else if(spanLeft && x>0 && !findPixel(tab,x-1,y1,P))
                {
                    spanLeft = 0;
                }
                if(!spanRight && x<IMAGE_WIDTH && findPixel(tab,x+1,y1,P))
                {
                    if(!push(Point_img(x+1,y1)))
                        return false;
                    spanRight = 1;
                }
                else if(spanRight && x<IMAGE_WIDTH && !findPixel(tab,x+1,y1,P))
                {
                    spanRight = 0;
                }
                y1++;
            }
        }

        //测试己方机器人色块大小
        //std::cout << tab << "色块大小:" << sum << std::endl;
        if((sum > sizeMin - 1) && (sum<sizeMax + 1) )
        {
            if(fabs(m_xRight-m_xLeft)<TEAMSIZEXYMIN || fabs(m_xRight-m_xLeft)>TEAMSIZEXYMAX ||fabs(m_yTop-m_yBottom)<TEAMSIZEXYMIN || fabs(m_yTop-m_yBottom)>TEAMSIZEXYMAX)
                return false;

            m_TargetN.x = sumx /sum;
            m_TargetN.y = sumy /sum;
            m_TargetN.num = sum;
            
            averx = double(sumx1) /sum;
            avery = double(sumy1) /sum;
            
            m_TargetN_true.x = double(sumx1_W) / double(sum);
            m_TargetN_true.y = double(sumy1_W) / double(sum);
            
            sumupx = sumdownx = sumrighty = sumlefty = 0;
            sumup = sumdown = sumright = sumleft = 0;
            sumleftupx = sumrightupx = sumleftdownx = sumrightdownx = 0;
            sumleftupy = sumrightupy = sumleftdowny = sumrightdowny = 0;
            sumleftup = sumrightup = sumleftdown = sumrightdown = 0;

            a = sumxx - sumx1 * averx;
            b = 2 * (sumxy - averx * sumy1);
            c = sumyy - sumy1 * avery;

            if(a!=c)
                k = b/(a-c);  //基于垂线段总和最小的最小二乘法
            else
                k = b/(2*a);  //常规最小二乘法

            for(jj = m_TargetN.y-10;jj<=m_TargetN.y;jj++)//左上块
                for(ii = m_TargetN.x-10;ii<m_TargetN.x;ii++)
                {
                    if(ii<0 || ii>(IMAGE_WIDTH-1) || jj<0 || jj>(IMAGE_HEIGHT-1))
                        continue;
                    if(screenBuffer(ii, jj, P) == 12684)//rgb(100,100,100)
                    {
                        sumleftup++;
                        sumleftupx = sumleftupx + ground.groundInfo[ii][jj].x;
                        sumleftupy = sumleftupy + ground.groundInfo[ii][jj].y;
                    }
                }

            for(jj = m_TargetN.y-10;jj<=m_TargetN.y;jj++)//右上块
                for(ii = m_TargetN.x;ii<=m_TargetN.x+10;ii++)
                {
                    if(ii<0 || ii>(IMAGE_WIDTH-1) || jj<0 || jj>(IMAGE_HEIGHT-1))
                        continue;
                    if(screenBuffer(ii, jj, P) == 12684)//rgb(100,100,100)
                    {
                        sumrightup++;
                        sumrightupx = sumrightupx + ground.groundInfo[ii][jj].x;
                        sumrightupy = sumrightupy + ground.groundInfo[ii][jj].y;
                    }
                }

            for(jj = m_TargetN.y;jj<=m_TargetN.y+10;jj++)//左下块
                for(ii = m_TargetN.x-10;ii<m_TargetN.x;ii++)
                {
                    if(ii<0 || ii>(IMAGE_WIDTH-1) || jj<0 || jj>(IMAGE_HEIGHT-1))
                        continue;
                    if(screenBuffer(ii, jj, P) == 12684)//rgb(100,100,100)
                    {
                        sumleftdown++;
                        sumleftdownx = sumleftdownx + ground.groundInfo[ii][jj].x;
                        sumleftdowny = sumleftdowny + ground.groundInfo[ii][jj].y;
                    }
                }

            for(jj = m_TargetN.y;jj<=m_TargetN.y+10;jj++)//右下块
                for(ii = m_TargetN.x;ii<m_TargetN.x+10;ii++)
                {
                    if(ii<0 || ii>(IMAGE_WIDTH-1) || jj<0 || jj>(IMAGE_HEIGHT-1))
                        continue;
                    if(screenBuffer(ii, jj, P) == 12684)//rgb(100,100,100)
                    {
                        sumrightdown++;
                        sumrightdownx = sumrightdownx + ground.groundInfo[ii][jj].x;
                        sumrightdowny = sumrightdowny + ground.groundInfo[ii][jj].y;
                    }
                }

            sumupx = sumleftupx + sumrightupx;
            sumdownx = sumleftdownx + sumrightdownx;
            sumup = sumleftup + sumrightup;
            sumdown = sumleftdown + sumrightdown;
            sumrighty = sumrightupy + sumrightdowny;
            sumlefty = sumleftupy + sumleftdowny;
            sumright = sumrightup + sumrightdown;
            sumleft = sumleftup + sumleftdown;

            if(sumup == 0 || sumdown == 0 )
                return false;
            
            if (fabs(double(sumupx) / double(sumup) - double(sumdownx) / double(sumdown)) < 0.5 || fabs(double(sumrighty) / double(sumright) - double(sumlefty) / double(sumleft)) < 0.5)
            {
                yerror = abs(double(sumleftdowny + sumrightdowny) / (double(sumleftdown) + double(sumrightdown)) - double(sumleftupy + sumrightupy) / (double(sumleftup) + double(sumrightup)));
                xerror = abs(double(sumrightupx + sumrightdownx) / (double(sumrightup) + double(sumrightdown)) - double(sumleftupx + sumleftdownx) / (double(sumleftup) + double(sumleftdown)));
                
                if (yerror > xerror)
                {
                    if(a!=c)  //k = b/(a-c);  //基于垂线段总和最小的最小二乘法
                    {
                        m_theta = atan((-sqrt(1 + k * k) - 1) / k) >= 0 ? atan((-sqrt(1 + k * k) - 1) / k) : atan((-sqrt(1 + k * k) - 1) / k) + Pi;  //Pi/2
                    }
                    else  //k = b/(2*a);  //常规最小二乘法
                    {
                        m_theta = atan(k) >= 0 ? atan(k) : atan(k) + Pi;
                    }
                }
                else
                {
                    if(a!=c)  //k = b/(a-c);  //基于垂线段总和最小的最小二乘法
                    {
                        m_theta = atan((sqrt(1 + k * k) - 1) / k) >= 0 ? atan((sqrt(1 + k * k) - 1) / k) : atan((sqrt(1 + k * k) - 1) / k) + Pi;  //0
                    }
                    else  //k = b/(2*a);  //常规最小二乘法
                    {
                        m_theta = atan(k) >= 0 ? atan(k) : atan(k) + Pi;
                    }
                }
            }
            else  //机器人小车的朝向接近平行于x轴或y轴，此时不可能用到常规最小二乘法
            {
                if (double(sumupx) / double(sumup) > double(sumdownx) / double(sumdown))              
                {
                    if(a!=c)  //k = b/(a-c);  //基于垂线段总和最小的最小二乘法
                    {
                        m_theta = atan((-sqrt(1 + k * k) - 1) / k >= 0 ? (-sqrt(1 + k * k) - 1) / k : (sqrt(1 + k * k) - 1) / k);
                    }
                    else  //k = b/(2*a);  //常规最小二乘法
                    {
                        m_theta = atan(k) >= 0 ? atan(k) : atan(k) + Pi;
                    }
                }
                else
                {
                    if(a!=c)  //k = b/(a-c);  //基于垂线段总和最小的最小二乘法
                    {
                        m_theta = atan((-sqrt(1 + k * k) - 1) / k <= 0 ? (-sqrt(1 + k * k) - 1) / k : (sqrt(1 + k * k) - 1) / k) + Pi;
                    }
                    else  //k = b/(2*a);  //常规最小二乘法
                    {
                        m_theta = atan(k) >= 0 ? atan(k) : atan(k) + Pi;
                    }
                }
            }
            //m_theta = m_theta + Pi/2;
            
            return true;
        }
        return false;
    }
    bool searchOppAndBall(int tab, int startX, int startY, int sizeMin, int sizeMax, cv::Mat& pStart)
    {
        int sum,sumx,sumy,x,y,y1;
        bool spanLeft, spanRight;
        double sumx1_W, sumy1_W;
        
        sum = sumx = sumy = 0;
        sumx1_W = sumy1_W = 0;
        x=startX;
        y=startY;
        m_xLeft = m_xRight = x;
        m_yTop = m_yBottom = y;
        
        pointV.clear();
        push(Point_img(x,y));
        
        while(pop(x,y))
        {
            y1 = y;
            while(findPixel(tab,x,y1,pStart) && y1>0)
                y1--;
            y1++;
            spanLeft = 0;
            spanRight = 0;
            while(findPixel(tab,x,y1,pStart) && y1<IMAGE_HEIGHT-1)
            {
                pStart.at<cv::Vec3b>(y1, x)[2] = 100;   // y1 行 x 列
                pStart.at<cv::Vec3b>(y1, x)[1] = 100;
                pStart.at<cv::Vec3b>(y1, x)[0] = 100;

                if(x <= m_xLeft)
                    m_xLeft = x;
                else if(x >= m_xRight)
                    m_xRight = x;
                if(y1 <= m_yTop)
                    m_yTop = y1;
                if(y1 >= m_yBottom)
                    m_yBottom = y1;

                sum++;
                sumx = sumx + x;
                sumy = sumy + y1;
                
                sumx1_W = sumx1_W + ground.groundInfo[x][y1].x;
                sumy1_W = sumy1_W + ground.groundInfo[x][y1].y;

                if(!spanLeft && x>0 && findPixel(tab,x-1,y1,pStart))
                {
                    if(!push(Point_img(x-1,y1)))
                        return false;
                    spanLeft = 1;
                }
                else if(spanLeft && x>0 && !findPixel(tab,x-1,y1,pStart))
                {
                    spanLeft = 0;
                }
                if(!spanRight && x< IMAGE_WIDTH &&findPixel(tab,x+1, y1,pStart))
                {
                    if(!push(Point_img(x+1,y1)))
                        return false;
                    spanRight = 1;
                }
                else if(spanRight && x<IMAGE_WIDTH && !findPixel(tab,x+1,y1,pStart))
                {
                    spanRight = 0;
                }
                y1++;
            }
        }

        //测试球或敌方机器人色块大小
        //std::cout << tab << "色块大小:" << sum << std::endl;
        if((sum > sizeMin - 1) && (sum < sizeMax + 1))
        {
            if(tab == BALL)
            {
                if((m_xRight - m_xLeft) < BALLSIZEXYMIN || (m_xRight - m_xLeft) > BALLSIZEXYMAX || (m_yBottom - m_yTop) < BALLSIZEXYMIN || (m_yBottom - m_yTop) > BALLSIZEXYMAX)
                    return false;
            }
            else
            {
                if ((m_xRight - m_xLeft) < OPPSIZEXYMIN || (m_xRight - m_xLeft) > OPPSIZEXYMAX || (m_yBottom - m_yTop) < OPPSIZEXYMIN || (m_yBottom - m_yTop) > OPPSIZEXYMAX)
                    return false;
            }
            
            m_TargetN.x = sumx/sum;
            m_TargetN.y = sumy/sum;
            m_TargetN.num = sum;
            
            m_TargetN_true.x = double(sumx1_W) / double(sum);
            m_TargetN_true.y = double(sumy1_W) / double(sum);

            return true;
        }
        return false;
    }
    message_interface_robosoccer::msg::PoseRobots identiRobo(int objectCount, cv::Mat& P)
    {
        message_interface_robosoccer::msg::PoseRobots robots_pose;
        
        int UnIdentifyCount = 0, NotCorrectCount = 0;
        double temptheta = 0.0, OrientAngle = 0.0;
        bool RobotFound[11];
        Point_img ReferPoint[4];
        bool blackID[4];
        int NotBlack = 0;
        int RobotID = 0;
        bool NoAngle = 0;
        double m_Length = 22;  //固定长度找特征点

        for(int i=0;i<11;i++)
            RobotFound[i] = false;
        for(int i=0;i<objectCount;i++)
        {
            temptheta = Pi/2 - atan(0.75) - NormalTheta[i];
            ReferPoint[0] = Point_img(TeamTarget[i].x + m_Length * cos(temptheta), TeamTarget[i].y + m_Length * sin(temptheta));
            ReferPoint[1] = Point_img(TeamTarget[i].x + m_Length * cos(temptheta + 2 * atan(0.75)), TeamTarget[i].y + m_Length * sin(temptheta + 2 * atan(0.75)));
            ReferPoint[2] = Point_img(TeamTarget[i].x + m_Length * cos(temptheta + Pi), TeamTarget[i].y + m_Length * sin(temptheta + Pi));
            ReferPoint[3] = Point_img(TeamTarget[i].x + m_Length * cos(temptheta + 2 * atan(0.75) + Pi), TeamTarget[i].y + m_Length * sin(temptheta + 2 * atan(0.75) + Pi));

            for(int j=0;j<4;j++)
                blackID[j] = 0;
            for(int j=0;j<4;j++)
                blackID[j] = findBlackID(ReferPoint[j].x,ReferPoint[j].y,j,P);
            
            if(blackID[0]&&blackID[1])
            {
                OrientAngle = NormalTheta[i] + Pi;
                RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
            }
            else if(blackID[2]&&blackID[3])
            {
                OrientAngle = NormalTheta[i];
                RobotID =findRobotID(ReferPoint[0],ReferPoint[1],P);
            }
            else //非正常情况处理，对应原程序比赛选项中的色标纠错
            {
                NoAngle = 1;
                NotBlack = 0;
                for(int j=0;j<4;j++)
                {
                    if(!blackID[j])
                        NotBlack++;
                }
                if(NotBlack==2)
                {
                    if(!blackID[0] && !blackID[2])
                    {
                        if (blackSum[0] > blackSum[2])
                        {
                            OrientAngle = NormalTheta[i] +Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        else if(blackSum[2]>blackSum[0])
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                    }
                    else if(!blackID[0] && !blackID[3])
                    {
                        if(blackSum[0]>blackSum[3])
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        else if(blackSum[3]>blackSum[0])
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                    }
                    else if(!blackID[1] && !blackID[2])
                    {
                        if(blackSum[1]>blackSum[2])
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        else if(blackSum[2]>blackSum[1])
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                    }
                    else if(!blackID[1] && !blackID[3])
                    {
                        if(blackSum[1]>blackSum[3])
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        else if(blackSum[3]>blackSum[1])
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                    }
                }
                else if(NotBlack == 3)
                {
                    if(!blackID[0]&&!blackID[1])
                    {
                        if(!blackID[2]&&blackSum[0]>blackSum[2]&&blackSum[1]>blackSum[2])
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        else if(!blackID[2]&&(blackSum[2]>blackSum[0] || blackSum[2]>blackSum[1]))
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                        
                        if(!blackID[3]&&blackSum[0]>blackSum[3]&&blackSum[1]>blackSum[3])
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        else if(!blackID[3]&&(blackSum[3]>blackSum[0] || blackSum[3]>blackSum[1]))
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                    }
                    else if(!blackID[2]&&!blackID[3])
                    {
                        if(!blackID[0]&&blackSum[2]>blackSum[0]&&blackSum[3]>blackSum[0])
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                        else if(!blackID[0]&&(blackSum[0]>blackSum[2]||blackSum[3]>blackSum[1]))
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                        
                        if(!blackID[1]&&blackSum[2]>blackSum[1]&&blackSum[3]>blackSum[1])
                        {
                            OrientAngle = NormalTheta[i];
                            RobotID = findRobotID(ReferPoint[0],ReferPoint[1],P);
                        }
                        else if(!blackID[1]&&(blackSum[1]>blackSum[2] || blackSum[1]>blackSum[3]))
                        {
                            OrientAngle = NormalTheta[i]+Pi;
                            RobotID = findRobotID(ReferPoint[2],ReferPoint[3],P);
                        }
                    }
                }
                else
                    continue;
            }
            
            if(RobotID>=0 && RobotID<=10)
            {
                robots_pose.pose[RobotID].theta = OrientAngle * 180/Pi;
                if(robots_pose.pose[RobotID].theta<0)
                    robots_pose.pose[RobotID].theta += 360;
                    
                //旧方法：色块重心----坐标转换---->物体世界坐标重心
                //robots_pose.pose[RobotID].x = ground.groundInfo[TeamTarget[i].x][TeamTarget[i].y].x;
                //robots_pose.pose[RobotID].y = ground.groundInfo[TeamTarget[i].x][TeamTarget[i].y].y;
                
                //新方法：色块所有像素世界坐标----求重心---->物体世界坐标重心，推荐使用，更准确
                robots_pose.pose[RobotID].x = TeamTarget_true[i].x;
                robots_pose.pose[RobotID].y = TeamTarget_true[i].y;

                if ((robots_pose.pose[RobotID].x - pose_processed_last.pose[RobotID].x) * (robots_pose.pose[RobotID].x - pose_processed_last.pose[RobotID].x) + (robots_pose.pose[RobotID].y - pose_processed_last.pose[RobotID].y) * (robots_pose.pose[RobotID].y - pose_processed_last.pose[RobotID].y) > 30)
                {
                    pose_notCorrectlyFound.pose[NotCorrectCount].x = robots_pose.pose[RobotID].x;
                    pose_notCorrectlyFound.pose[NotCorrectCount].y = robots_pose.pose[RobotID].y;
                    pose_notCorrectlyFound.pose[NotCorrectCount].theta = robots_pose.pose[RobotID].theta;
                    NotCorrectCount++;
                }
                else
                {
                    RobotFound[RobotID] = true;
                }
            }
            else
            {
                //旧方法：色块重心----坐标转换---->物体世界坐标重心
                //pose_unIdentified.pose[UnIdentifyCount].x = ground.groundInfo[TeamTarget[i].x][TeamTarget[i].y].x;
                //pose_unIdentified.pose[UnIdentifyCount].y = ground.groundInfo[TeamTarget[i].x][TeamTarget[i].y].y;
                
                //新方法：色块所有像素世界坐标----求重心---->物体世界坐标重心，推荐使用，更准确
                pose_unIdentified.pose[UnIdentifyCount].x = TeamTarget_true[i].x;
                pose_unIdentified.pose[UnIdentifyCount].y = TeamTarget_true[i].y;
                
                UnIdentifyCount++;
                if (NoAngle)
                    pose_unIdentified.pose[UnIdentifyCount].theta = NormalTheta[i] * 180 / Pi - 45;
                else
                    pose_unIdentified.pose[UnIdentifyCount].theta = OrientAngle * 180 / Pi - 45;
                    
                if (pose_unIdentified.pose[UnIdentifyCount].theta < 0)
                    pose_unIdentified.pose[UnIdentifyCount].theta += 360;
            }
        }
        if ((NotCorrectCount != 0 || UnIdentifyCount != 0))
            RobotInforFilter(&robots_pose, &RobotFound[0], UnIdentifyCount, NotCorrectCount);
            
        return robots_pose;
    }
    int countMin(int R, int G, int B, int N)
    {
        int temp = 0;
        if (N == 3)
        {
            if (R <= G && R <= B)
                temp = R;
            else if (G <= R && G <= B)
                temp = G;
            else if (B <= R && B <= G)
                temp = B;

        }
        else if (N == 2)
        {
            if (R <= G) temp = R;
            else temp = G;
        }
        return temp;
    }
    bool pop(int &x, int &y)
    {
        if(pointV.empty())
            return false;
        Point_img p =pointV.back();
        x = p.x;
        y = p.y;
        pointV.pop_back();
        pointV_num--;
        return true;
    }
    bool push(Point_img point)
    {
        if(pointV_num>=199)
            return false;
        pointV.push_back(point);
        pointV_num++;
        return true;
    }
    int screenBuffer(int m, int n, cv::Mat& P)
    {
        int r, g, b, data;
        r = P.at<cv::Vec3b>(n, m)[2]/8;   // n 行 m 列
        g = P.at<cv::Vec3b>(n, m)[1]/8;
        b = P.at<cv::Vec3b>(n, m)[0]/8;
        data = ((r & 0x1f) << 10 | (g & 0x1f) << 5 | (b & 0x1f));        //  &按位与；<<二进制左移
        return data;
    }
    int findBlackID(int m, int n, int num, cv::Mat& P)
    {
        bool IDCode;
        int ii,jj;
        int sumblack = 0;
        int H = 0,S = 0,I = 0;
        for(jj = n-1;jj<=n+1;jj++)
        {
            for(ii = m-1;ii<=m+1;ii++)
            {
                if(jj<0 || jj>(IMAGE_HEIGHT-1) || ii<0 || ii>(IMAGE_WIDTH-1))
                    continue;
                RGBToHS(ii, jj, P, H, S, I);
                if(!judgePixel(MEMB1,H,S,I)
                        &&!judgePixel(MEMB2,H,S,I)
                        &&(screenBuffer(ii, jj, P)!=12684))
                {
                    sumblack++;
                    if(sumblack >= 5)
                        break;
                }
            }
            if(sumblack >= 5)
                break;
        }

        if(sumblack >= 5)
            IDCode = 1;
        else
        {
            IDCode = 0;
            blackSum[num] = sumblack;
        }
        return IDCode;
    }
    int findRobotID(Point_img p1, Point_img p2, cv::Mat& P)
    {
        int roboID, RPID1, RPID2;
        int ii, jj;
        int H = 0,S = 0,I = 0;
        int sum1, sum2, sum0;
        
        sum1 = sum2 = sum0 = 0;
        for(jj=p1.y-2;jj<=p1.y+2;jj++)
        {
            for(ii=p1.x-2;ii<=p1.x+2;ii++)
            {
                if(jj < 0 || jj>(IMAGE_HEIGHT - 1) || ii < 0 || ii>(IMAGE_WIDTH - 1))
                    continue;
                RGBToHS(ii, jj, P, H, S, I);
                if(judgePixel(MEMB1,H,S,I))
                    sum1++;
                else if(judgePixel(MEMB2,H,S,I))
                    sum2++;
                else if(screenBuffer(ii, jj, P) != 12684)
                    sum0++;
            }
        }

        RPID1 = judgeColor(sum1,sum2,sum0);
        
        sum1 = sum2 = sum0 = 0;
        for(jj = p2.y-2;jj<=p2.y+2;jj++)
            for(ii = p2.x-2;ii<=p2.x+2;ii++)
            {
                if(jj < 0 || jj>(IMAGE_HEIGHT - 1) || ii < 0 || ii>(IMAGE_WIDTH - 1))
                    continue;
                RGBToHS(ii, jj, P, H, S, I);
                if(judgePixel(MEMB1,H,S,I))
                    sum1++;
                else if(judgePixel(MEMB2,H,S,I))
                    sum2++;
                else if(screenBuffer(ii, jj, P) != 12684)
                    sum0++;
            }

        RPID2 = judgeColor(sum1,sum2,sum0);
        
        if (RPID1 == 0 && RPID2 == 1)
            roboID = 0;
        else if (RPID1 == 1 && RPID2 == 0)
            roboID = 1;
        else if (RPID1 == 1 && RPID2 == 1)
            roboID = 2;
        else if (RPID1 == 0 && RPID2 == 2)
            roboID = 3;
        else if (RPID1 == 2 && RPID2 == 0)
            roboID = 4;
        else 
            roboID = -1;
        return roboID;
    }
    void RobotInforFilter(message_interface_robosoccer::msg::PoseRobots* PoseRobots, bool* ObjectFound, int UnKnown, int Wrong)
    {
	    int NearCount = 0;
	    int Dis = 0, MinDis = 0, tag = 0;
	    int i, j;
	    for (i = 0;i < 5;i++)
	    {
		    if (!ObjectFound[i])
		    {
			    NearCount = 0;
			    if (UnKnown > 0)
			    {
				    for (j = 0;j < UnKnown;j++)
				    {
					    Dis = (pose_unIdentified.pose[j].x - pose_processed_last.pose[i].x) * (pose_unIdentified.pose[j].x - pose_processed_last.pose[i].x) + (pose_unIdentified.pose[j].y - pose_processed_last.pose[i].y) * (pose_unIdentified.pose[j].y - pose_processed_last.pose[i].y);
					    if (Dis <= 30)
					    {
						    NearCount++;
						    if (NearCount == 1)
						    {
							    MinDis = Dis;
							    tag = j;
						    }
						    if (Dis < MinDis)
						    {
							    MinDis = Dis;
							    tag = j;
						    }
					    }
				    }
				    if (tag >= 0 && tag <= 4)
				    {
					    PoseRobots->pose[i].theta = pose_unIdentified.pose[tag].theta;
					    PoseRobots->pose[i].x = pose_unIdentified.pose[tag].x;
					    PoseRobots->pose[i].y = pose_unIdentified.pose[tag].y;
				    }
			    }
			    else if (Wrong > 0)
			    {
				    for (j = 0;j < Wrong;j++)
				    {
					    Dis = (pose_notCorrectlyFound.pose[j].x - pose_processed_last.pose[i].x) * (pose_notCorrectlyFound.pose[j].x - pose_processed_last.pose[i].x) + (pose_notCorrectlyFound.pose[j].y - pose_processed_last.pose[i].y) * (pose_notCorrectlyFound.pose[j].y - pose_processed_last.pose[i].y);
					    if (Dis <= 30)
					    {
						    NearCount++;
						    tag = j;
					    }
				    }
				    if (NearCount == 1)
				    {
					    PoseRobots->pose[i].theta = pose_notCorrectlyFound.pose[tag].theta;
					    PoseRobots->pose[i].x = pose_notCorrectlyFound.pose[tag].x;
					    PoseRobots->pose[i].y = pose_notCorrectlyFound.pose[tag].y;
				    }
			    }
		    }
	    }
    }
    void BallPosFilter(message_interface_robosoccer::msg::PoseRobots* PoseRobots, message_interface_robosoccer::msg::PoseRobots* PoseOppRobots)
    {
        message_interface_robosoccer::msg::PoseRobots full_robots_pose;
	    int RobotNearBall = 0;
	    int i=0, j=0, tag=11;
	    int TotalR=0, Count=0, minlength=0, length=0;
	    for (i = 0;i < 5;i++)
	    {
		    full_robots_pose.pose[i] = PoseRobots->pose[i];
		    full_robots_pose.pose[i + 5] = PoseOppRobots->pose[i];
	    }
	    //PoseRobots->pose[i+5]=PoseOppRobots->pose[i];
	    TotalR = 10;
	    Point_img RP[10];
	    bool NearBallflag[10];
	    for (i = 0;i < TotalR;i++)
	    {
		    NearBallflag[i] = 0;
		    if (((full_robots_pose.pose[i].x - pose_processed_last.pose[10].x) * (full_robots_pose.pose[i].x - pose_processed_last.pose[10].x)
			    + (full_robots_pose.pose[i].y - pose_processed_last.pose[10].y) * (full_robots_pose.pose[i].y - pose_processed_last.pose[10].y) <= 100))//cm
		    {
			    RobotNearBall++;
			    NearBallflag[i] = 1;
		    }
	    }
	    if (RobotNearBall == 1)
	    {
		    for (i = 0;i < TotalR;i++)
		    {
			    if (NearBallflag[i] && i < 5)
			    {
				    if (pose_processed_last.pose[10].x > pose_processed_last.pose[i].x && pose_processed_last.pose[10].y > pose_processed_last.pose[i].y)
				    {
					    PoseRobots->pose[10].x = full_robots_pose.pose[i].x + abs(5 * cos(full_robots_pose.pose[i].theta / 180 * Pi));
					    PoseRobots->pose[10].y = full_robots_pose.pose[i].y + abs(5 * sin(full_robots_pose.pose[i].theta / 180 * Pi));
				    }
				    else if (pose_processed_last.pose[10].x > pose_processed_last.pose[i].x && pose_processed_last.pose[10].y < pose_processed_last.pose[i].y)
				    {
					    PoseRobots->pose[10].x = full_robots_pose.pose[i].x + abs(5 * cos(full_robots_pose.pose[i].theta / 180 * Pi));
					    PoseRobots->pose[10].y = full_robots_pose.pose[i].y - abs(5 * sin(full_robots_pose.pose[i].theta / 180 * Pi));
				    }
				    else if (pose_processed_last.pose[10].x < pose_processed_last.pose[i].x && pose_processed_last.pose[10].y < pose_processed_last.pose[i].y)
				    {
					    PoseRobots->pose[10].x = full_robots_pose.pose[i].x - abs(5 * cos(full_robots_pose.pose[i].theta / 180 * Pi));
					    PoseRobots->pose[10].y = full_robots_pose.pose[i].y - abs(5 * sin(full_robots_pose.pose[i].theta / 180 * Pi));
				    }
				    else if (pose_processed_last.pose[10].x < pose_processed_last.pose[i].x && pose_processed_last.pose[10].y > pose_processed_last.pose[i].y)
				    {
					    PoseRobots->pose[10].x = full_robots_pose.pose[i].x - abs(5 * cos(full_robots_pose.pose[i].theta / 180 * Pi));
					    PoseRobots->pose[10].y = full_robots_pose.pose[i].y + abs(5 * sin(full_robots_pose.pose[i].theta / 180 * Pi));
				    }
				    break;
			    }
			    else if (NearBallflag[i] && i > 4)
				    PoseRobots->pose[10] = full_robots_pose.pose[i];
		    }
	    }
	    else if (RobotNearBall > 1)
	    {
		    Count = 0;
		    for (i = 0;i < TotalR;i++)
		    {
			    if (NearBallflag[i])
			    {
				    RP[Count].x = full_robots_pose.pose[i].x;
				    RP[Count].y = full_robots_pose.pose[i].y;
				    Count++;
			    }
		    }
		    for (i = 0;i < RobotNearBall;i++)
		    {
			    for (j = i + 1;j < RobotNearBall;j++)
			    {
				    if ((RP[i].x - RP[j].x) * (RP[i].x - RP[j].x) + (RP[i].y - RP[j].y) * (RP[i].y - RP[j].y) <= 100)
				    {
					    PoseRobots->pose[10].x = (double(RP[i].x) + double(RP[j].x)) / 2.0;
					    PoseRobots->pose[10].y = (double(RP[i].y) + double(RP[j].y)) / 2.0;
					    return;
				    }
			    }
			}
		    for (i = 0;i < TotalR;i++)
		    {
			    if (NearBallflag[i])
			    {
				    minlength = (full_robots_pose.pose[i].x - pose_processed_last.pose[10].x) * (full_robots_pose.pose[i].x - pose_processed_last.pose[10].x)
					    + (full_robots_pose.pose[i].y - pose_processed_last.pose[10].y) * (full_robots_pose.pose[i].y - pose_processed_last.pose[10].y);
				    tag = i;
				    break;
			    }
		    }
		    for (i = 0;i < TotalR;i++)
		    {
			    if (NearBallflag[i])
			    {
				    length = (full_robots_pose.pose[i].x - pose_processed_last.pose[10].x) * (full_robots_pose.pose[i].x - pose_processed_last.pose[10].x)
					    + (full_robots_pose.pose[i].y - pose_processed_last.pose[10].y) * (full_robots_pose.pose[i].y - pose_processed_last.pose[10].y);
				    if (length < minlength) 
				        tag = i;
			    }
		    }
		    if (tag < 5 && tag >= 0)
		    {
			    if (pose_processed_last.pose[10].x > pose_processed_last.pose[tag].x && pose_processed_last.pose[10].y > pose_processed_last.pose[tag].y)
			    {
				    PoseRobots->pose[10].x = full_robots_pose.pose[tag].x + abs(5 * cos(full_robots_pose.pose[tag].theta / 180 * Pi));
				    PoseRobots->pose[10].y = full_robots_pose.pose[tag].y + abs(5 * sin(full_robots_pose.pose[tag].theta / 180 * Pi));
			    }
			    else if (pose_processed_last.pose[10].x > pose_processed_last.pose[tag].x && pose_processed_last.pose[10].y < pose_processed_last.pose[tag].y)
			    {
				    PoseRobots->pose[10].x = full_robots_pose.pose[tag].x + abs(5 * cos(full_robots_pose.pose[tag].theta / 180 * Pi));
				    PoseRobots->pose[10].y = full_robots_pose.pose[tag].y - abs(5 * sin(full_robots_pose.pose[tag].theta / 180 * Pi));
			    }
			    else if (pose_processed_last.pose[10].x < pose_processed_last.pose[tag].x && pose_processed_last.pose[10].y < pose_processed_last.pose[tag].y)
			    {
				    PoseRobots->pose[10].x = full_robots_pose.pose[tag].x - abs(5 * cos(full_robots_pose.pose[tag].theta / 180 * Pi));
				    PoseRobots->pose[10].y = full_robots_pose.pose[tag].y - abs(5 * sin(full_robots_pose.pose[tag].theta / 180 * Pi));
			    }
			    else if (pose_processed_last.pose[10].x < pose_processed_last.pose[tag].x && pose_processed_last.pose[10].y > pose_processed_last.pose[tag].y)
			    {
				    PoseRobots->pose[10].x = full_robots_pose.pose[tag].x - abs(5 * cos(full_robots_pose.pose[tag].theta / 180 * Pi));
				    PoseRobots->pose[10].y = full_robots_pose.pose[tag].y + abs(5 * sin(full_robots_pose.pose[tag].theta / 180 * Pi));
			    }
		    }
		    else if (tag > 4 && tag < 11)
			    PoseRobots->pose[10] = full_robots_pose.pose[tag];
	    }
    }
    int judgeColor(int a, int b, int c)
    {
        int colorID;
        if(a>=10&&b>=10)
            return -1;
        if(a>=10)
            colorID = 1;
        else if(b>=10)
            colorID = 2;
        else if(c>=10)
            colorID = 0;
        else
            colorID = -1;
        return colorID;
    }
    bool judgePixel(int object, int h, int s, int i)
    {
        if(h == 0)
            return false;
        else
        {
            if(HSIThreshold[object*6+1]>=HSIThreshold[object*6+0])
            {
                if(h>=HSIThreshold[object*6+0]
                && h<=HSIThreshold[object*6+1]
                && s>=HSIThreshold[object*6+2]
                && s<=HSIThreshold[object*6+3]
                && i>=HSIThreshold[object*6+4]
                && i<=HSIThreshold[object*6+5])
                    return true;
                else
                    return false;
            }
            else
            {
                if(h >= HSIThreshold[object*6+0] || 
                        (  h<=HSIThreshold[object*6+1]
                        && s>=HSIThreshold[object*6+2]
                        && s<=HSIThreshold[object*6+3]
                        && i>=HSIThreshold[object*6+4]
                        && i<=HSIThreshold[object*6+5] )  )
                    return true;
                else
                    return false;
            }
        }
        return false;
    }
    void RGBToHS(int m, int n, cv::Mat& P, int& H, int& S, int& I)
    {
        int R, G, B;
        R = P.at<cv::Vec3b>(n, m)[2];   // n 行 m 列
        G = P.at<cv::Vec3b>(n, m)[1];
        B = P.at<cv::Vec3b>(n, m)[0];
        
        H = 10 * HLUT[R][G][B];
        S = int(100 * (1 - 3.0 * countMin(R, G, B, 3) / (double(R) + double(G) + double(B))));
        I = (int)(R + G + B) / 3;
    }
    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Vision_Processing>("vision_processing");
    rclcpp::spin(node);
//    while (rclcpp::ok())
//    {
//        rclcpp::spin_some(node);
//    }
    rclcpp::shutdown();
    return 0;
}
