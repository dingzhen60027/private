#include "rclcpp/rclcpp.hpp"
#include "message_interface_robosoccer/msg/pose_robots.hpp"
#include "message_interface_robosoccer/msg/velocity_robots.hpp"
#include "message_interface_robosoccer/msg/decision_var.hpp"

#include <cmath>

#include "strategy.h"

class Strategy : public rclcpp::Node
{
public:
    Strategy(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s已创建!", name.c_str());
        
        isAttack = true;
        isRightArea = false;
        StartMode = NormalStart;
        isBackInitPose = true;
        isReInit = true;
        isStartGame = false;
        
        distError = 4.0; //2.0
        m_Front = 0.0;
        
        m_AngleParameter.Kp = 18.0;
	    m_AngleParameter.Kd = 2.0;
	    m_AngleParameter.AngleError = 8.0 / 180.0 * Pi; //5.0
	    m_AngleParameter.MaxAngleSpeed = 80.0 / 2.0; 
	    m_AngleParameter.MaxAngle = 75.0 * Pi / 180.0;
	    
	    m_MoveParameter.V_MAX = 70.0;
        m_MoveParameter.V_max = 70.0;
        m_MoveParameter.max_distance = 40.0;
        m_MoveParameter.kp4pospd = 12;	//12
        m_MoveParameter.kd4pospd =  2;// 0.50;
        m_MoveParameter.max_distanceG = 25.0;
        m_MoveParameter.kp4pospdG =18.5;//18.50;//15.5
        m_MoveParameter.kd4pospdG =0;// 1.90;
	    /********************case变量********************/
	    //case1
	    isShootPoint_Arrived = false;
	    /************************************************/
        
        subscription_pose_ = this->create_subscription<message_interface_robosoccer::msg::PoseRobots>("robots_pose", 1, std::bind(&Strategy::pose_sub_callback, this, std::placeholders::_1));
        publish_velocity_ = this->create_publisher<message_interface_robosoccer::msg::VelocityRobots>("robots_velocity",1);
//        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),                  //轮速发布 暂定20ms
//                                          std::bind(&Strategy::timer_callback,this));

        subscription_decisionvar_ = this->create_subscription<message_interface_robosoccer::msg::DecisionVar>("decision_var", 1, std::bind(&Strategy::var_sub_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<message_interface_robosoccer::msg::PoseRobots>::SharedPtr subscription_pose_;
    rclcpp::Publisher<message_interface_robosoccer::msg::VelocityRobots>::SharedPtr publish_velocity_;
//    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<message_interface_robosoccer::msg::DecisionVar>::SharedPtr subscription_decisionvar_;
    
    message_interface_robosoccer::msg::PoseRobots robots_pose;
    message_interface_robosoccer::msg::PoseRobots robots_pose_old;
    message_interface_robosoccer::msg::VelocityRobots robots_velocity;
    
    RobotPose my_robot_pose[ROBOTNUM+1];
    OppRobotPoint opp_robot_point[ROBOTNUM+1];
    BallPoint ball_point;
    
    Velocity rbV[ROBOTNUM+1];
    
    int currentForm[6];
    int currentOrder[6];
    int currentResult[6];
    
    bool isAttack;//GUI设置变量：己方进攻/敌方进攻
    bool isRightArea;//GUI设置变量：右半场/左半场
    int StartMode;//GUI设置变量：比赛模式
    bool isBackInitPose;//GUI设置变量：是否归位
    bool isReInit;//GUI设置变量：是否初始化
    bool isStartGame;//GUI设置变量：开始比赛/停止比赛
    
    double distError;
    double m_Front;
	AngleParameter m_AngleParameter;
	MoveParameter m_MoveParameter;
	
	/******************综合运动变量******************/
	//case1
	bool isShootPoint_Arrived;
	/************************************************/
    void var_sub_callback(const message_interface_robosoccer::msg::DecisionVar::SharedPtr msg)
    {
        isAttack = msg->decisionvar[0];
        isRightArea = msg->decisionvar[1];
        StartMode = msg->decisionvar[2];
        isBackInitPose = msg->decisionvar[3];
        isReInit = msg->decisionvar[4];
        isStartGame = msg->decisionvar[5];
        
        if(isReInit == true)
        {
            //需要初始化的变量
            isShootPoint_Arrived = false;
        }
        
        RCLCPP_INFO(this->get_logger(), "成功接收策略变量!");
    }
    
    void pose_sub_callback(const message_interface_robosoccer::msg::PoseRobots::SharedPtr msg)
    {
        /********************坐标转换********************/
        for (int i=0; i<11; i++)
        {
            robots_pose_old.pose[i].x = robots_pose.pose[i].x;
            robots_pose_old.pose[i].y = robots_pose.pose[i].y;
            robots_pose_old.pose[i].theta = robots_pose.pose[i].theta;
            
            robots_pose.pose[i].x = msg->pose[i].x;
            robots_pose.pose[i].y = GROUND_HEIGHT-msg->pose[i].y;
            robots_pose.pose[i].theta = msg->pose[i].theta*Pi/180;
            
            if (robots_pose.pose[i].x==wallleft || robots_pose.pose[i].x==wallright || robots_pose.pose[i].y==wallbottom || robots_pose.pose[i].y==walltop)
            {
                robots_pose.pose[i].x = robots_pose_old.pose[i].x;
                robots_pose.pose[i].y = robots_pose_old.pose[i].y;
                robots_pose.pose[i].theta = robots_pose_old.pose[i].theta;
            }
        }
        if (isRightArea)
        {
            for (int i=0; i<11; i++)
            {
                robots_pose.pose[i].x = GROUND_WIDTH-robots_pose.pose[i].x;
                robots_pose.pose[i].y = GROUND_HEIGHT-robots_pose.pose[i].y;
                robots_pose.pose[i].theta = Pi+robots_pose.pose[i].theta;
                if (robots_pose.pose[i].theta > 2 * Pi) 
                    robots_pose.pose[i].theta -= 2 * Pi;
            }
        }
        
        for (int i=1; i<=ROBOTNUM; i++)
        {
            my_robot_pose[i].x = robots_pose.pose[i-1].x;
            my_robot_pose[i].y = robots_pose.pose[i-1].y;
            my_robot_pose[i].theta = robots_pose.pose[i-1].theta;
            
            opp_robot_point[i].x = robots_pose.pose[i+4].x;
            opp_robot_point[i].y = robots_pose.pose[i+4].y;
        }
        ball_point.x = robots_pose.pose[10].x;
        ball_point.y = robots_pose.pose[10].y;
        /************************************************/
		
		if (isBackInitPose)
	        RobotReturn2InitPt();//归位
        else
        {
	        MiroSot_DecisionMaking();//比赛
		}
		
		//发布轮速
		for (int i=0; i<ROBOTNUM; i++)
        {
            robots_velocity.velocity[i].left_wheel = rbV[i+1].LeftValue;
            robots_velocity.velocity[i].right_wheel = rbV[i+1].RightValue;
        }
        publish_velocity_->publish(robots_velocity);
        RCLCPP_INFO(this->get_logger(), "| 1号车速度设定：L:%f R:%f|", 
                    robots_velocity.velocity[0].left_wheel, robots_velocity.velocity[0].right_wheel);
    }
    
//    void timer_callback()
//    {
//        for (int i=0; i<ROBOTNUM; i++)
//        {
//            robots_velocity.velocity[i].left_wheel = rbV[i+1].LeftValue;
//            robots_velocity.velocity[i].right_wheel = rbV[i+1].RightValue;
//        }
//        
//        publish_velocity_->publish(robots_velocity);
//    }
    
    
    void RobotReturn2InitPt()
    {
	    RobotPose Robot_pt[6];
	    switch(StartMode)
	    {
	    case NormalStart://普通
		    {
			    if(isAttack)//我方开球
			    {
				    Robot_pt[1].x = CENTER_X;
				    Robot_pt[1].y = CENTER_Y + 4.0;
				    Robot_pt[1].theta = 5.4;
				    
				    Robot_pt[2].x = CENTER_X - 14.0;
				    Robot_pt[2].y = CENTER_Y - 5.0;
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 21;
				    Robot_pt[3].y = CENTER_Y + 10.0;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 21;
				    Robot_pt[4].y = CENTER_Y - 10.0;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }	
			    else//对方开球
			    {
				    Robot_pt[1].x = 80;
				    Robot_pt[1].y = CENTER_Y;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = 50;
				    Robot_pt[2].y = CENTER_Y ;
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = CENTER_Y + 10.0;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = CENTER_Y - 10.0;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
		    }
		    break;
	    case PenaltyKick://点球
		    {
			    if(!isAttack)//对方点球
			    {
				    Robot_pt[1].x = 114;
				    Robot_pt[1].y = 50;
				    Robot_pt[1].theta = Pi;
				    
				    Robot_pt[2].x = 114;
				    Robot_pt[2].y = 80;
				    Robot_pt[2].theta = Pi;
				    
				    Robot_pt[3].x = 114;
				    Robot_pt[3].y = 100;
				    Robot_pt[3].theta = Pi;
				    
				    Robot_pt[4].x = 114;
				    Robot_pt[4].y = 130;
				    Robot_pt[4].theta = Pi;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
			    else//我方点球
			    {
				    Robot_pt[1].x = 175;
				    Robot_pt[1].y = 90;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = 100;
				    Robot_pt[2].y = 65;
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 100;
				    Robot_pt[3].y = 115;
				    Robot_pt[3].theta = 0;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = 85;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y + 5.0;
				    Robot_pt[5].theta = Pi/2;
			    }
		    }
		    break;
	    case GoalKick://门球
		    {
			    if(!isAttack)//对方发球
			    {
				    Robot_pt[1].x = 100;
				    Robot_pt[1].y = 140;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = 100;
				    Robot_pt[2].y = 40;
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = 98;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = 82;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
			    else//我方发球
			    {
				    Robot_pt[1].x = 10;
				    Robot_pt[1].y = 110;
				    Robot_pt[1].theta =Pi/12;
				    
				    Robot_pt[2].x = 10;
				    Robot_pt[2].y = 150;
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = 70;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 60;
				    Robot_pt[4].y = 75;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = 60;
				    Robot_pt[5].theta = Pi/2;
			    }
		    }
		    break;
	    case FreeKick://任意球
		    {
			    //对方开任意球
			    if(!isAttack)
			    {
				    Robot_pt[1].x = ball_point.x - 22;
				    Robot_pt[1].y = ball_point.y;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = ball_point.x - 22;
				    if(ball_point.y>90)
					    Robot_pt[2].y = ball_point.y - 12;
				    else 
					    Robot_pt[2].y= ball_point.y + 12;
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = 100;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = 80;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
			    //我方开任意球
			    else
			    {
				    Robot_pt[1].x = ball_point.x - 15;
				    Robot_pt[1].y = ball_point.y;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = ball_point.x - 22;
				    if(ball_point.y>90)
					    Robot_pt[2].y = ball_point.y - 12;
				    else
					    Robot_pt[2].y = ball_point.y + 12;
				    
				    Robot_pt[2].theta = 0;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = 100;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = 80;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
		    }
		    break;
	    case TackleBall://争球
		    {
			    if(ball_point.x<CENTER_X && ball_point.y<CENTER_Y)
			    {
				    Robot_pt[1].x = 30;
				    Robot_pt[1].y = 30;
				    Robot_pt[1].theta = 0;
								    
				    Robot_pt[2].x = 45;
				    Robot_pt[2].y = CENTER_Y;
				    Robot_pt[2].theta = 3*Pi/2;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = CENTER_Y;
				    Robot_pt[3].theta = Pi/2;
								    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = CENTER_Y + 12.0;
				    Robot_pt[4].theta = Pi/2;

				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
			    else if(ball_point.x<CENTER_X && ball_point.y>CENTER_Y)
			    {
				    Robot_pt[1].x = 30;
				    Robot_pt[1].y = 150;
				    Robot_pt[1].theta = 0;
					    
				    Robot_pt[2].x = 45;
				    Robot_pt[2].y = CENTER_Y;
				    Robot_pt[2].theta = 3*Pi/2;
				    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = CENTER_Y - 12.0;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = CENTER_Y;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
			    else if(ball_point.x>CENTER_X && ball_point.y<CENTER_Y)
			    {
				    Robot_pt[1].x = 140;
				    Robot_pt[1].y = 30;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = 100;
				    Robot_pt[2].y = CENTER_Y - 30.0;
				    Robot_pt[2].theta = 0;
					    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = 75;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = 85;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
			    else
			    {
				    Robot_pt[1].x = 140;
				    Robot_pt[1].y = 150;
				    Robot_pt[1].theta = 0;
				    
				    Robot_pt[2].x = 100;
				    Robot_pt[2].y = CENTER_Y + 30.0;
				    Robot_pt[2].theta = 0;
					    
				    Robot_pt[3].x = 20;
				    Robot_pt[3].y = 95;
				    Robot_pt[3].theta = Pi/2;
				    
				    Robot_pt[4].x = 20;
				    Robot_pt[4].y = 105;
				    Robot_pt[4].theta = Pi/2;
				    
				    Robot_pt[5].x = 5;
				    Robot_pt[5].y = CENTER_Y;
				    Robot_pt[5].theta = Pi/2;
			    }
		    }
		    break;
	    case RetractCar://收车
		    {
                if(!isAttack)
			    {
                    for(int i=1;i<=5;i++)
				    {
					    Robot_pt[i].x = 210;
					    Robot_pt[i].y = 110.0 + 12.0*i;
					    Robot_pt[i].theta = 0;
				    }
			    }
			    else
			    {
                    for(int i=1;i<=5;i++)
				    {
					    Robot_pt[i].x = 10;
					    Robot_pt[i].y = 110.0 + 12.0*i;
					    Robot_pt[i].theta = 0;
				    }
			    }
		    }
		    break;
	    default:
		    break;
	    }
	    //--比赛用代码段--
	    if(isStartGame)
		{
	        for(int i=1;i<=ROBOTNUM;i++)
	        {
		        Point pt[ROBOTNUM+1];
		        pt[i].x = Robot_pt[i].x;
		        pt[i].y = Robot_pt[i].y;
		        if(distRobot2Pt(my_robot_pose[i],pt[i])>distError)
		        {
			        ToPositionN(&my_robot_pose[i],pt[i],60, 0, &rbV[i]);
		        }
		        else
			        s_TurnToAnglePD(&my_robot_pose[i],Robot_pt[i].theta,&rbV[i]);	
	        }
	    }
	    else
	    {
	        for(int i=1;i<=ROBOTNUM;i++)
	        {
		        rbV[i].LeftValue = 0;
		        rbV[i].RightValue = 0;	
	        }
	    }
	    //---------------
	    //--测试用代码段--
//	    Point pt;
//	    pt.x = 20;
//	    pt.y = 90;
//      double angle;
//      angle = 0;
//	    if(distRobot2Pt(my_robot_pose[1],pt)>distError)
//	    {
//		    ToPositionN(&my_robot_pose[1],pt,60, 0, &rbV[1]);
//	    }
//	    else
//		    s_TurnToAnglePD(&my_robot_pose[1],angle,&rbV[1]);	
	    //---------------
	    return;
    }
    void MiroSot_DecisionMaking()
    {
	    preProcess();//信息预处理，记录机器人小车和球的轨迹(最近几帧的位置，可用于预测)
	    
	    int areaNo;
	    areaNo = GetAreaNo(ball_point);//场地分区，得到球所在的区域号码(看球落在哪个区块上，采用先粗分后细分的方法)[1-32]
	    
	    int dmformNo;
	    dmformNo = taskDecompose(areaNo);//队形制定，根据球所在的区号确定机器人队形号码dmformNo(同时还能根据开球状态看使用哪种队形模式)   例如：6
	    formInterpret(dmformNo);//队形解释，根据队形号得到其所对应的角色号，结果按重要性排序存到数组currentForm中，按角色重要性排序而不是车号   例如：32、33、1、68
	    
	    charAllot();//角色分配(任务分配)，依据所需角色的重要性依次得到完成队形应该所需要的车号数组currentOrder   例如：3、4、1、2
	    robotManager();//机器人管理，将车号与角色号对应起来得到数组currentResult   例如：1、68、32、33
	    bestManager();//优化分配，优化机器人管理(固化后卫)，即守门角色车号固定(例如：角色12固定5号车)
	    
	    actProcess();//动作执行，更新小车轮速值
	    
	    return;
    }
    
    /********************比赛函数********************/
    void preProcess()
    {
        return;
    }
    int GetAreaNo(BallPoint ball)
    {
        int areaNo;
        
        //粗略分区
        if (ball.x>0 && ball.x<15 && ball.y>90 && ball.y<180)
            areaNo = 1;
        else if (ball.x>15 && ball.x<220 && ball.y>165 && ball.y<180)
            areaNo = 2;
        else if (ball.x>205 && ball.x<220 && ball.y>90 && ball.y<165)
            areaNo = 3;
        else if (ball.x>0 && ball.x<15 && ball.y>0 && ball.y<90)
            areaNo = 4;
        else if (ball.x>15 && ball.x<220 && ball.y>0 && ball.y<15)
            areaNo = 5;
        else if (ball.x>205 && ball.x<220 && ball.y>15 && ball.y<90)
            areaNo = 6;
        else if (ball.x>15 && ball.x<205 && ball.y>90 && ball.y<165)
            areaNo = 7;
        else if (ball.x>15 && ball.x<205 && ball.y>15 && ball.y<90)
            areaNo = 8;
        else
            areaNo = 0;
        
        return areaNo;
    }
    int taskDecompose(int areaNo)
    {
        int formNo;
        
        formNo = areaNo;
        
        return formNo;
    }
    void formInterpret(int formNo)
    {
        if (formNo == 1)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 2)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 3)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 4)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 5)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 6)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 7)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else if (formNo == 8)
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        else
        {
            currentForm[1] = 3;
            currentForm[2] = 4;
            currentForm[3] = 1;
            currentForm[4] = 2;
        }
        
        return;
    }
    void charAllot()
    {
        currentOrder[1] = 3;
        currentOrder[2] = 4;
        currentOrder[3] = 1;
        currentOrder[4] = 2;
        
        return;
    }
    void robotManager()
    {
        for (int i=1; i<=ROBOTNUM-1; i++)
            currentResult[currentOrder[i]] = currentForm[i];
        
        return;
    }
    void bestManager()
    {
        currentResult[ROBOTNUM] = 5;
        
        return;
    }
    void actProcess()
    {
		for(int i=1; i<=ROBOTNUM; i++)
		{
			rbV[i].LeftValue = 0;
			rbV[i].RightValue = 0;
		}
		if(isStartGame)
		{
			for(int i=1; i<=ROBOTNUM; i++)
			{
				charInterpret(i, currentResult[i], &rbV[i]);
			}
		}
        
        return;
    }
    /************************************************/
    /********************综合运动********************/
    //本函数用于比赛过程中根据队形确定的小车运动执行
    void charInterpret(int robotNo, int charNo, Velocity *rbV)//robotNo-小车车号,charNo-角色号,rbV-对应小车车号的左右轮速
    {
	    switch (charNo)
	    {
	    case 1://前锋
		    {   
			    Point shootGoal;
			    Point shootPoint;
			    
			    shootGoal.x = 220.0;
			    shootGoal.y = 90.0;
			    
			    double shootDist;
			    
			    /******************简单射门实现******************/
//			    double shootAngle;
//			    
//			    double dx,dy,angle;
//			    dx = shootGoal.x-ball_point.x;
//			    dy = shootGoal.y-ball_point.y;
//			    angle = atan2(dy,dx);
//			    angle = cn_AngleTrim2PI(angle);
//			    
//			    shootDist = 8.0;
//			    shootPoint.x = ball_point.x - shootDist*cos(angle);
//			    shootPoint.y = ball_point.y - shootDist*sin(angle);
//			    shootAngle = angle;
//			    
//			    if (shootPoint.x < 5)
//			    {
//			        shootPoint.x = 5;
//			    }
//			    if (shootPoint.x > 215)
//			    {
//			        shootPoint.x = 215;
//			    }
//			    if (shootPoint.y < 5)
//			    {
//			        shootPoint.y = 5;
//			    }
//			    if (shootPoint.y > 175)
//			    {
//			        shootPoint.y = 175;
//			    }
//			    
//			    if ((distRobot2Pt(my_robot_pose[robotNo], shootPoint) > distError) && (isShootPoint_Arrived == false))
//			    {
//				    if (ball_point.x>15 && ball_point.x<205 && ball_point.y>15 && ball_point.y<165)
//				    {
//				        ToPositionN(&my_robot_pose[robotNo], shootPoint, 60, 30, rbV);
//				    }
//				    else
//				    {
//				        shootPoint.x = ball_point.x;
//			            shootPoint.y = ball_point.y;
//			            
//			            if (shootPoint.x < 5)
//			            {
//			                shootPoint.x = 5;
//			            }
//			            if (shootPoint.x > 215)
//			            {
//			                shootPoint.x = 215;
//			            }
//			            if (shootPoint.y < 5)
//			            {
//			                shootPoint.y = 5;
//			            }
//			            if (shootPoint.y > 175)
//			            {
//			                shootPoint.y = 175;
//			            }
//			            
//				        ToPositionN(&my_robot_pose[robotNo], shootPoint, 30, 20, rbV);
//				    }
//			    }
//			    else
//				{
//			        isShootPoint_Arrived = true;
//			        
//			        double robotAngle = my_robot_pose[robotNo].theta;
//	                robotAngle = cn_AngleTrim2PI(robotAngle);
//	                double theta = robotAngle - angle;
//	                if (theta < -Pi)
//		                theta = theta + 2 * Pi;
//	                else if (theta > Pi)
//		                theta = theta - 2 * Pi;
//	                double theta_e = fabs(theta);
//	                double angleError = 10.0 / 180.0 * Pi;
//			        if (theta_e > angleError)
//	                {
//		                s_TurnToAnglePD(&my_robot_pose[robotNo], shootAngle, rbV);
//	                }
//	                else
//	                {
//	                    s_TurnToAnglePD(&my_robot_pose[robotNo], shootAngle, rbV);
//	                    rbV->LeftValue += 50;
//		                rbV->RightValue += 50;
//		                if (ball_point.x > 200)
//		                {
//		                    rbV->LeftValue = 0;
//			                rbV->RightValue = 0;
//		                }
//	                }
//		        }
		        /************************************************/
		        /****************中垂线法射门实现****************/
		        if (ball_point.x>110 && ball_point.x<205 && ball_point.y>15 && ball_point.y<165)//一定区域内触发
			    {
			        double dx1,dy1,angle1;
			        dx1 = shootGoal.x-ball_point.x;
			        dy1 = shootGoal.y-ball_point.y;
			        angle1 = atan2(dy1,dx1);
			        angle1 = cn_AngleTrim2PI(angle1);
			        double dx2,dy2,angle2;
			        dx2 = my_robot_pose[robotNo].x-ball_point.x;
			        dy2 = my_robot_pose[robotNo].y-ball_point.y;
			        angle2 = atan2(dy2,dx2);
			        angle2 = cn_AngleTrim2PI(angle2);
			        double Angle;
			        Angle = angle2-angle1;
			        Angle = cn_AngleTrim2PI(Angle);
			        double Theta;
			        Theta = Pi - Angle;
			        
			        Point ball;
			        ball.x = ball_point.x;
			        ball.y = ball_point.y;
			        
			        if (cos(Theta) == 0)
			        {
		                shootDist = 0;
			        }
			        else
			        {
			            shootDist = fabs((distRobot2Pt(my_robot_pose[robotNo], ball) / 2) / cos(Theta));
			        }
			        
			        shootPoint.x = ball_point.x-shootDist*cos(angle1);
			        shootPoint.y = ball_point.y-shootDist*sin(angle1);
			        
			        if (shootPoint.x < 5)
		            {
		                shootPoint.x = 5;
		            }
		            if (shootPoint.x > 215)
		            {
		                shootPoint.x = 215;
		            }
		            if (shootPoint.y < 5)
		            {
		                shootPoint.y = 5;
		            }
		            if (shootPoint.y > 175)
		            {
		                shootPoint.y = 175;
		            }
			        
			        ToPositionN(&my_robot_pose[robotNo], shootPoint, 50, 20, rbV); //60
			        //ToPositionPD(&my_robot_pose[robotNo],shootPoint, 30, 30, rbV);
			        if (ball_point.x > 205)
	                {
	                    rbV->LeftValue = 0;
		                rbV->RightValue = 0;
	                }
			    }
			    else
			    {
			        rbV->LeftValue = 0;
		            rbV->RightValue = 0;
		            
		            //ToPositionN(&my_robot_pose[robotNo], ball_point, 60, 30, rbV);
			    }
		        /************************************************/
		    
		    }
		    break;
		case 2://中锋
		    {
                rbV->LeftValue = 0;
			    rbV->RightValue = 0;
		    }
		    break;
		case 3://左后卫
		    {
			    rbV->LeftValue = 0;
			    rbV->RightValue = 0;
		    }
		    break;
		case 4://右后卫
		    {
			    rbV->LeftValue = 0;
			    rbV->RightValue = 0;
		    }
		    break;
		case 5://守门员
		    {
			    rbV->LeftValue = 0;
			    rbV->RightValue = 0;
		    }
		    break;
		default:
		    {
			    rbV->LeftValue = 0;
			    rbV->RightValue = 0;
		    }
		    break;
	    }
	    
	    return;
    }
    /************************************************/
    /********************组合运动********************/
    
    /************************************************/
    /********************运动基元********************/
    //本函数用于使小车到达指定点
    void ToPositionN(RobotPose* robot, Point directionpt, double samespeed, double samespeed_end, Velocity* pSpeed)
    {
        //--------------------------------------------------------
        if (samespeed >= 90)
	    {
		    samespeed=90;
	    }
	    if (samespeed_end >= 90)
	    {
		    samespeed=90;
	    }
        //--------------------------------------------------------
	    int vemax;
	    vemax = 110;
	    
	    double Dx,Dy;
	    double Anglerb2target;
	    
	    //坐标系变换
	    robot->y = 180.0 - robot->y;
	    robot->theta = 2*Pi - robot->theta;
	    directionpt.y =180.0 - directionpt.y;
	    robot->theta = cn_AngleTrim2PI(robot->theta);

	    Dx = directionpt.x - robot->x;
	    Dy = directionpt.y - robot->y;

	    if(distRobot2Pt(*robot,directionpt)<30)
		    samespeed = 60*distRobot2Pt(*robot,directionpt)/30 + samespeed_end;
		    //samespeed = samespeed*distRobot2Pt(*robot,directionpt)/20;

	    double kp4new;
	    kp4new = 15.0;

	    //计算targetpt<robotpt>到directionpt的角度
	    Anglerb2target = atan2(Dy ,Dx);
	    Anglerb2target = cn_AngleTrim2PI(Anglerb2target);

	    //计算下一帧的目标角度
	    double disiredAngle;
	    disiredAngle = Anglerb2target;
	    disiredAngle = cn_AngleTrim2PI(disiredAngle);
	    
	    //计算角度偏差<需要转动的角度>
	    double Angle_e;
	    Angle_e = disiredAngle - robot->theta;
	    Angle_e = cn_AngleTrim2PI(Angle_e);
	    
	    //计算左右轮速度差并计算出左右轮速度
	    double ka,limitation,a;
	    a = Pi*0.4;//72度
	    limitation = 100;
	    ka=samespeed;
	    double speed_e;
	    if(Angle_e <= Pi/2)//角度偏差在第一象限
	    {
		    speed_e = kp4new*Angle_e;
		    speed_e = Limit(speed_e,limitation);
		    if(a-fabs(Angle_e)>0)
			    samespeed = ka*(a-fabs(Angle_e))/a;
		    else samespeed = 0;
		    
		    pSpeed->LeftValue = samespeed + speed_e/2;
		    pSpeed->RightValue = samespeed - speed_e/2;
	    }
	    else if(Angle_e <= Pi)//角度偏差在第二象限
	    {
		    speed_e = kp4new*(Pi - Angle_e);
		    speed_e = Limit(speed_e,limitation);
		    if(a-fabs(Pi-Angle_e)>0)
			    samespeed = ka*(a-fabs(Pi-Angle_e))/a;
		    else samespeed = 0;

		    pSpeed->LeftValue = samespeed + speed_e/2;
		    pSpeed->RightValue = samespeed - speed_e/2;
		    pSpeed->LeftValue =- pSpeed->LeftValue;
		    pSpeed->RightValue =- pSpeed->RightValue;
	    }
	    else if(Angle_e<3*Pi/2)//角度偏差在第三象限
	    {
		    speed_e = kp4new*(Angle_e - Pi);
		    speed_e = Limit(speed_e,limitation);
		    if(a-fabs(Angle_e - Pi)>0)
			    samespeed = ka*(a-fabs(Angle_e - Pi))/a;
		    else samespeed = 0;

		    pSpeed->LeftValue = samespeed - speed_e/2;
		    pSpeed->RightValue = samespeed + speed_e/2;
		    pSpeed->LeftValue =- pSpeed->LeftValue;
		    pSpeed->RightValue =- pSpeed->RightValue;
	    }
	    else//角度偏差在第四象限
	    {
		    speed_e = kp4new*(2*Pi - Angle_e);
		    speed_e = Limit(speed_e,limitation);
		    if(a-fabs(2*Pi - Angle_e)>0)
			    samespeed = ka*(a-fabs(2*Pi - Angle_e))/a;
		    else samespeed = 0;

		    pSpeed->LeftValue = samespeed - speed_e/2;
		    pSpeed->RightValue = samespeed + speed_e/2;
	    }
	    
	    if(pSpeed->LeftValue>vemax)
	    {
		    pSpeed->LeftValue = vemax;
		    pSpeed->RightValue = pSpeed->LeftValue - fabs(speed_e);
	    }
	    if(pSpeed->RightValue>vemax)
	    {
		    pSpeed->RightValue = vemax;
		    pSpeed->LeftValue = pSpeed->RightValue - fabs(speed_e);
	    }
	    if(pSpeed->LeftValue<-vemax)
	    {
		    pSpeed->LeftValue = -vemax;
		    pSpeed->RightValue = pSpeed->LeftValue + fabs(speed_e);
	    }
	    if(pSpeed->RightValue<-vemax)
	    {
		    pSpeed->RightValue = -vemax;
		    pSpeed->LeftValue = pSpeed->RightValue + fabs(speed_e);
	    }
	    
	    //变回原坐标
	    robot->y = 180.0 - robot->y;
	    robot->theta = 2*Pi - robot->theta;
	    directionpt.y = 180.0 - directionpt.y;
	    robot->theta = cn_AngleTrim2PI(robot->theta);
	    
	    return;
    }
    
    //本函数用于使小车撞球
    int ToPositionPD(RobotPose* pROBOTPOSTURE,Point Target,double same_speed,double end_speed,Velocity* pLRWheelVelocity)
    {
        if (same_speed > 120)	//70
            same_speed = 120;

        //vBase 是使小车到达定点时保持一定的速度
        int clock_sign,move_sign;
        double theta,theta_e1;			//e1为当前角度误差
        //static double theta_e2 = 0;		//e2为上一周期角度误差
        double dx,dy,dx1,dy1,distance;
        double speed;

        //坐标变换，以小车中心为原点，小车朝向为y轴   （原理：矩阵的旋转变换）
        dx1=Target.x-pROBOTPOSTURE->x;
        dy1=Target.y-pROBOTPOSTURE->y;
        dx=dx1*cos(pROBOTPOSTURE->theta-Pi/2)+dy1*sin(pROBOTPOSTURE->theta-Pi/2);
        dy=-dx1*sin(pROBOTPOSTURE->theta-Pi/2)+dy1*cos(pROBOTPOSTURE->theta-Pi/2);
        theta=atan2(dy,dx);				//目标点的方向角（-Pi~Pi）
        distance = sqrt(dx*dx+dy*dy);

        /////////速度调节
        //远程高速
        //为了使到定点时保持一定的速度,用增大距离的方法
        if(end_speed>same_speed)
            end_speed=same_speed;
        if(distance > m_MoveParameter.max_distance)	//75.0*Pi/180.0  m_MoveParameter.max_distance=40
            speed = same_speed;
        else //近程线性减速
            speed = distance/m_MoveParameter.max_distance*same_speed;
        if(speed < end_speed) //低速不低
            speed = end_speed;
        /////////////////////////////////////

        if(distance>25)
        {
            if(fabs(fabs(theta)-Pi/2)<0.15*Pi)
            {
                theta = cn_AngleTrim2PI(theta);
                if(theta <= Pi/2)//第一象限
                {
                    move_sign = 1;
                    clock_sign = 1;
                    theta_e1 = Pi/2 - theta;
                }
                else if(theta <= Pi)//第二象限
                {
                    move_sign = 1;
                    clock_sign = -1;
                    theta_e1 = theta - Pi/2;
                }
                else if(theta <= 3*Pi/2)//第三象限
                {
                    move_sign = -1;
                    clock_sign = 1;
                    theta_e1 = 3*Pi/2 - theta;
                }
                else//第四象限
                {
                    move_sign = -1;
                    clock_sign = -1;
                    theta_e1 = theta - 3*Pi/2;
                }

                pLRWheelVelocity->LeftValue = speed*move_sign + clock_sign*(m_MoveParameter.kp4pospd*theta_e1);//m_MoveParameter.kp4pospd=12
                pLRWheelVelocity->RightValue = speed*move_sign - clock_sign*(m_MoveParameter.kp4pospd*theta_e1);
                //保存本周期角度误差，下一周期作微分用
                //theta_e2=theta_e1;//直接赋值可能调用会出错，会用到其他车号的数据
            }

            else//>0.15*Pi
            {
                double sp=45;
                double angle_e=fabs(fabs(theta)-Pi/2);
                double r=30/angle_e;//为什么这么赋值???
                double d=7.5;
                theta = cn_AngleTrim2PI(theta-Pi/2);
                if(theta<Pi/2)//第一项限
                {
                    pLRWheelVelocity->LeftValue = sp*(r-d/2)/r;
                    pLRWheelVelocity->RightValue = sp*(r+d/2)/r;
                }
                else if(theta<Pi)//第二项限
                {
                    pLRWheelVelocity->LeftValue = -sp*(r-d/2)/r;
                    pLRWheelVelocity->RightValue = -sp*(r+d/2)/r;
                }
                else if(theta<Pi*1.5)//第三项限
                {
                    pLRWheelVelocity->LeftValue = -sp*(r+d/2)/r;
                    pLRWheelVelocity->RightValue = -sp*(r-d/2)/r;
                }
                else//第四项限
                {
                    pLRWheelVelocity->LeftValue = sp*(r+d/2)/r;
                    pLRWheelVelocity->RightValue = sp*(r-d/2)/r;
                }
            }
        }

        else//distance<=25
        {
            if(fabs(fabs(theta)-Pi/2)>m_AngleParameter.MaxAngle)  //m_AngleParameter.MaxAngle = 75.0*Pi/180.0;
            {
                TurnToPointPD(pROBOTPOSTURE,Target,NOCLOCK,pLRWheelVelocity);
                pLRWheelVelocity->LeftValue /= 2.2;		//2
                pLRWheelVelocity->RightValue /= 2.2;
                return 0;
            }

            theta = cn_AngleTrim2PI(theta);
            if(theta <= Pi/2)//第一象限
            {
                move_sign = 1;
                clock_sign = 1;
                theta_e1 = Pi/2 - theta;
            }
            else if(theta <= Pi)//第二象限
            {
                move_sign = 1;
                clock_sign = -1;
                theta_e1 = theta - Pi/2;
            }
            else if(theta <= 3*Pi/2)//第三象限
            {
                move_sign = -1;
                clock_sign = 1;
                theta_e1 = 3*Pi/2 - theta;
            }
            else//第四象限
            {
                move_sign = -1;
                clock_sign = -1;
                theta_e1 = theta - 3*Pi/2;
            }

            pLRWheelVelocity->LeftValue = speed*move_sign + clock_sign*(m_MoveParameter.kp4pospd*theta_e1);
            pLRWheelVelocity->RightValue = speed*move_sign - clock_sign*(m_MoveParameter.kp4pospd*theta_e1);
            //保存本周期角度误差，下一周期作微分用
            //theta_e2=theta_e1;
        }
        return 0;
    }
    
    //本函数用于使小车快速转指向目标角度:先判断需要转动的角度，在误差范围内则不做任何动作，否则根据其大小确定转动速度
    //Robot为小车位置信息，dbAngle为目标角度(弧度)，pSpeed为返回的左右轮速
    //Kp、Kd为比例、微分调节参数
    void s_TurnToAnglePD(RobotPose* pRobot, double dbAngle, Velocity* pSpeed)
    {
	    double theta, theta1;
	    double SameSpeed;
	    dbAngle = cn_AngleTrim2PI(dbAngle);
	    pRobot->theta = cn_AngleTrim2PI(pRobot->theta);//车身在坐标系下的角度位姿
	    theta = pRobot->theta - dbAngle;//转向目标角度需要转动的夹角
	    if (theta < -Pi)
		    theta = 2 * Pi + theta;
	    else if (theta > Pi)
		    theta = theta - 2 * Pi;
	    theta1 = fabs(theta);
	    if (theta1 < m_AngleParameter.AngleError)//判断是否在角度误差限之内
	    {
		    pSpeed->LeftValue = 0;
		    pSpeed->RightValue = 0;
		    return;
	    }
	    //需要转动的角度在角度误差限之外
	    SameSpeed = m_AngleParameter.Kp * theta1 + m_AngleParameter.Kd * (theta1 - m_Front);
	    if (SameSpeed > m_AngleParameter.MaxAngleSpeed)
		    SameSpeed = m_AngleParameter.MaxAngleSpeed;
	    m_Front = theta1;
	    if (theta < 0)
	    {
		    pSpeed->LeftValue = -SameSpeed;
		    pSpeed->RightValue = SameSpeed;
	    }
	    else
	    {
		    pSpeed->LeftValue = SameSpeed;
		    pSpeed->RightValue = -SameSpeed;
	    }
	    return;
    }
    
    //本函数用于使小车的方向以给定的时钟方向快速转到所要求的角度方向
    //Robot为小车的位置信息，Angle为需要转向的角度，Speed为返回的左右轮速
    //pPD为比例、微分调节参数结构
    int TurnToAnglePD(RobotPose *pRobot,double dbAngle,int clock,Velocity *pSpeed)
    {
        double Difference,SameSpeed;
        int Quadrant;
        Difference=pRobot->theta-dbAngle;
        Difference = cn_AngleTrim2PI(Difference);
        if (Difference <= m_AngleParameter.AngleError)//判断是否在角度误差限之内
        {
            pSpeed->LeftValue=0.;
            pSpeed->RightValue=0.;
            return 1;
        }
        if (clock==ANTICLOCK)
            Difference=2*Pi-Difference;
        else if (clock == NOCLOCK)
        {
            if (Difference >= 0 &&  Difference < Pi/2)//判断角度差所在象限
                Quadrant=1;
            else if (Difference >= Pi/2 &&  Difference < Pi)
            {
                Quadrant=2;
                Difference=Pi-Difference;
            }
            else if (Difference >= Pi && Difference < 3*Pi/2)
            {
                Quadrant=3;
                Difference=Difference-Pi;
            }
            else
            {
                Quadrant=4;
                Difference=2*Pi-Difference;
            }
        }
        //此处进行PD调节
        if(clock==0)
            m_AngleParameter.Kp = 18.5;
        SameSpeed=m_AngleParameter.Kp*Difference/*+m_AngleParameter.Kd*(Difference-m_Front)*/;
        if (SameSpeed>m_AngleParameter.MaxAngleSpeed)
            SameSpeed=m_AngleParameter.MaxAngleSpeed;
        //m_Front=Difference;
        if (clock==CLOCKWISE)
        {
            pSpeed->LeftValue=SameSpeed;
            pSpeed->RightValue=-SameSpeed;
        }
        else if (clock == ANTICLOCK)
        {
            pSpeed->LeftValue=-SameSpeed;
            pSpeed->RightValue=SameSpeed;
        }
        else
        {
            switch(Quadrant)
            {
            case 1://顺时针旋转
            case 3:
                {
                    pSpeed->LeftValue=SameSpeed;
                    pSpeed->RightValue=-SameSpeed;
                    break;
                }
            case 2://逆时针旋转
            case 4:
                {
                    pSpeed->LeftValue=-SameSpeed;
                    pSpeed->RightValue=SameSpeed;
                    break;
                }
            }
        }
        return 1;
    }
    
    //本函数用于使小车快速转向指定点
    //Robot为小车位置信息，point为转向的点，pSpeed为返回的左右轮速
    //Kp、Kd为比例、微分调节参数
    int TurnToPointPD(RobotPose *pRobot,Point point,int clock,Velocity *pSpeed)
    {
        double Angle;
        int Result;
        Point Point1;
        Point1.x=pRobot->x;
        Point1.y=pRobot->y;
        Result=PointToPointDirectionAngle(Point1,point,&Angle);
        if (Result==0)
            return 0;
        Result=TurnToAnglePD(pRobot,Angle,clock,pSpeed);
        return (Result);
    }
    /************************************************/
    /********************辅助函数********************/
    double distRobot2Pt(RobotPose robot,Point point)//车到点的距离
    {
	    return sqrt((robot.x-point.x)*(robot.x-point.x) + (robot.y-point.y)*(robot.y-point.y));
    }
    double cn_AngleTrim2PI(double theta)//将角度转到区间 [0, 2*pi)
    {
	    double cn_pi = acos(-1);
	    if(theta<-100000000000 || theta >1000000)
		    theta = 0;
	    while(theta>=2*cn_pi)
		    theta -= 2*Pi;
	    while(theta<0)
		    theta += 2*Pi;
	    return theta;
    }
    double Limit(double value, double limitation)//限制数值函数
    {
	    double tempv;
	    tempv = value;
	    if (tempv > limitation)
		    tempv = limitation;
	    if (tempv < -limitation)
		    tempv = -limitation;
	    return tempv;
    }
    //本函数用于求解从点point1到点point2的方向角，成功返回1，否则，返回0
    //求得（0—2pi)之间的弧度方向角存于Angle
    int PointToPointDirectionAngle(Point Point1,Point Point2,double *pAngle)
    {
        double x,y;
        x=Point2.x-Point1.x;
        y=Point2.y-Point1.y;
        if (x==0 && y==0)
            return 0;
        *pAngle=atan2(y,x);
        if (*pAngle<0)
            *pAngle+=2*Pi;
        return 1;
    }
    /************************************************/
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Strategy>("strategy");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
