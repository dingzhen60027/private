#include <array>
#include <vector>

#define Pi 3.14159265358979323846
#define SQR(a) (a)*(a)

#define IMAGE_WIDTH   1920
#define IMAGE_HEIGHT  1200
#define GRIDSIZE 10 //Identifiyall遍历时的网格大小，原640*480时是4，现改用1920*1200时用10，越大越容易丢车丢球，太小时间成本长，容易丢帧，在保证不丢车和球的情况下越大越好

#define GROUND_WIDTH  220
#define GROUND_HEIGHT 180

#define GROUND_BITMAP_WIDTH 550
#define GROUND_BITMAP_HEIGHT 450
#define GROUND_BITMAP_X_OFFSET 124
#define GROUND_BITMAP_Y_OFFSET 22

#define Work_Distance 2500  //相机工作距离
#define centerNum 16        //标定点个数

//------------------------
//颜色阈值代号，例如：TEAM为队色，MEMB1为队号辅助色1,MEMB2为队号辅助色2...(不是说MEMBx是x号车特有的颜色)
#define TEAM    0
#define MEMB1	1
#define MEMB2	2
#define MEMB3	3
#define MEMB4	4
#define MEMB5	5
#define BALL	6
#define OPP		7
//------------------------

//------------------------
//色块大小阈值
#define TEAMSIZEMIN 50 //100 100
#define TEAMSIZEMAX 1000 //1000 700
#define OPPSIZEMIN 50 //35 50
#define OPPSIZEMAX 1000 //1000 700
#define BALLSIZEMIN 20 //40 10
#define BALLSIZEMAX 600 //900 600

#define TEAMSIZEXYMIN 3
#define TEAMSIZEXYMAX 45
#define OPPSIZEXYMIN 6
#define OPPSIZEXYMAX 80
#define BALLSIZEXYMIN 6
#define BALLSIZEXYMAX 50
//------------------------

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

struct RoboPoint_img
{
    RoboPoint_img()
    {
        x = 0;
        y = 0;
        num = 0;//色块大小
    }
    RoboPoint_img(int x, int y, int num)
    {
        this->x = x;
        this->y = y;
        this->num = num;
    }
    int x;
    int y;
    int num;
};

struct RGBTriple
{
    RGBTriple()
    {
        Red = 0;
        Green = 0;
        Blue = 0;
    }
    RGBTriple(unsigned char Red, unsigned char Green, unsigned char Blue)
    {
        this->Red = Red;
        this->Green = Green;
        this->Blue = Blue;
    }
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
};

typedef struct
{
    Point **groundInfo;
    Point_img Center;
    Point_img LeftTop;
    Point_img RightBottom;
    Point_img GateLeftTop;
    Point_img GateRightBottom;
    double M = 0.0;
    double E = 0.0;
}Ground;

typedef struct
{
    double Rx;
    double Ry;
    double Rz;
    double Tx;
    double Ty;
    double Tz;
    double f;
    double k1;
    double cx;
    double cy;
    double sx;
}Camera_calib_param;


