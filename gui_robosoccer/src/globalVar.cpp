#include "globalVar.h"

Point calipoint[centerNum];
int HSIThreshold[8][6];
int ***HLUT;
int I_y[255]={0};
int robot_xy[360][12][2];

bool isOpenCamera = false;
bool isStartAcq = false;

bool isFinishDisp_1 = true;
bool isFinishDisp_2 = true;
bool isFinishDrawRobo = true;

bool isSimuing = false;

bool isAttack = true;//GUI设置变量：己方进攻/敌方进攻
bool isRightArea = false;//GUI设置变量：右半场/左半场
int StartMode = 0;//GUI设置变量：比赛模式
bool isBackInitPose = true;//GUI设置变量：是否归位
bool isReInit = true;//GUI设置变量：是否初始化
bool isStartGame = false;//GUI设置变量：开始比赛/停止比赛
