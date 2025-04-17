#define Pi 3.14159265358979323846

#define GROUND_WIDTH  220
#define GROUND_HEIGHT 180

#define		wallleft		0.0
#define		wallright		220.0
#define		wallbottom		0.0
#define		walltop			180.0

#define		CENTER_X		110.0
#define		CENTER_Y		90.0

#define ROBOTNUM 5

//比赛模式
#define NormalStart  0 //普通
#define PenaltyKick  1 //点球
#define GoalKick     2 //门球
#define FreeKick     3 //任意球
#define TackleBall   4 //争球
#define RetractCar   5 //收车

#define CLOCKWISE 1
#define ANTICLOCK -1
#define NOCLOCK  0
#define FOREWARD 1
#define BACKWARD -1

struct RobotPose
{
    RobotPose()
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }
    RobotPose(double x, double y, double theta)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }
    double x;
    double y;
    double theta;
};

typedef struct
{
    double x = 0.0;
    double y = 0.0;
}OppRobotPoint, BallPoint;

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

struct Velocity
{
    Velocity()
    {
        LeftValue = 0.0;
        RightValue = 0.0;
    }
    Velocity(double LeftValue, double RightValue)
    {
        this->LeftValue = LeftValue;
        this->RightValue = RightValue;
    }
    double LeftValue;
    double RightValue;
};


struct AngleParameter
{
	double Kp = 0.0;
	double Kd = 0.0;
	double AngleError = 0.0;//视觉所能分辨的角度
	double MaxAngleSpeed = 0.0;//小车的最大旋转边缘线速度
	double MaxAngle = 0.0;//小车能以最大角速度旋转的角度下限
};

struct MoveParameter
{
    double max_distance = 0.0;
    double max_distanceG = 0.0;
    double V_MAX = 0.0;
    double V_max = 0.0;
    double max_angle = 0.0;
    double kp4pospd = 0.0;
    double kd4pospd = 0.0;
    double kp4pospdG = 0.0;
    double kd4pospdG = 0.0;
};
