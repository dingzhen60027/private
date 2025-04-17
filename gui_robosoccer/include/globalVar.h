#ifndef GLOBALVAR_H
#define GLOBALVAR_H

#ifndef PI
#define Pi 3.14159265358979323846
#endif // PI

#define IMAGE_WIDTH   1920
#define IMAGE_HEIGHT  1200
#define DISP_WIDTH    960
#define DISP_HEIGHT   600

#define GROUND_WIDTH  220
#define GROUND_HEIGHT 180
#define GROUND_IMAGE_WIDTH 800
#define GROUND_IMAGE_HEIGHT 500
#define GROUND_BITMAP_WIDTH 550
#define GROUND_BITMAP_HEIGHT 450
#define GROUND_BITMAP_X_OFFSET 124
#define GROUND_BITMAP_Y_OFFSET 22
#define SIMULATION_WIDTH 480
#define SIMULATION_HEIGHT 300

#define centerNum 16   //标定点个数

#define TEAM    0
#define MEMB1	1
#define MEMB2	2
#define MEMB3	3
#define MEMB4	4
#define MEMB5	5
#define BALL	6
#define OPP		7

#define COLORENLARGE_WIDTH 300
#define COLORENLARGE_HEIGHT 300

#define COLORDATA_WIDTH 200
#define COLORDATA_HEIGHT 200
#define COLORCIR 100

#define SHOWI_WIDTH 308  //306 <255*1.2> (+2)
#define SHOWI_HEIGHT 62  //60  (+2)

struct Point
{
    Point()
    {
        x = 0.0;
        y = 0.0;
    }
    Point(double x, double y)
    {
        this->x = x;
        this->y = y;
    }
    double x;
    double y;
};

struct Point_img
{
    Point_img()
    {
        x = 0;
        y = 0;
    }
    Point_img(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
    int x;
    int y;
};

extern Point calipoint[centerNum];       //标定点
extern int HSIThreshold[8][6];       //HSI阈值
extern int ***HLUT;                  //RGB转HSI查找表
extern int I_y[255];               //存储颜色分析I的结果
extern int robot_xy[360][12][2]; //机器人仿真图像关键点查找表

extern bool isOpenCamera;
extern bool isStartAcq;

extern bool isFinishDisp_1;
extern bool isFinishDisp_2;
extern bool isFinishDrawRobo;

extern bool isSimuing;

extern bool isAttack;//GUI设置变量：己方进攻/敌方进攻
extern bool isRightArea;//GUI设置变量：右半场/左半场
extern int StartMode;//GUI设置变量：比赛模式
extern bool isBackInitPose;//GUI设置变量：是否归位
extern bool isReInit;//GUI设置变量：是否初始化
extern bool isStartGame;//GUI设置变量：开始比赛/停止比赛

#endif // GLOBALVAR_H
