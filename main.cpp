#include<iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include "include/matplotlibcpp.h"
#include "include/DWA.h"
namespace plt = matplotlibcpp;
std::vector<double> motion(std::vector<double> now_state, double v, double w)
{
    now_state[2] += w*DWA_config::dt;
    now_state[0] += v*cos(now_state[2])*DWA_config::dt;
    now_state[1] += v*sin(now_state[2])*DWA_config::dt;
    now_state[3] = v;
    now_state[4] = w;
    return now_state;
}
void plot_robot(double robot_x, double robot_y)
{
    std::vector<double> x,y;
    for(int angle=0; angle<=360; angle++){
        x.push_back(DWA_config::robot_radius * cos(angle*M_PI/180) + robot_x);
        y.push_back(DWA_config::robot_radius * sin(angle*M_PI/180) + robot_y);
    }
    plt::plot(x,y);
}
void plot_trajectory(const std::vector<std::vector<double>> trajectory)
{
    std::vector<double> x,y;
    for(const auto& pos : trajectory){
        x.push_back(pos[0]);
        y.push_back(pos[1]);
    }
    plt::plot(x,y);
}
void obstract_line(std::vector<std::vector<double>>& ob, double x1, double x2, double y1, double y2)
{
    //終了処理
    const int pitch = 2;
    double dx, dy, dis, preX, preY;
    dx = x2 - x1;
    dy = y2 - y1;
    dis = sqrt(pow(dx, 2) + pow(dy, 2));
    preX = x1;
    preY = y1;
    double angle = atan2(y2-y1, x2-x1);
    std::vector<double> temp_array(2);
    while(dis > pitch)
    {
        temp_array[0] = pitch*cos(angle) + preX;
        temp_array[1] = pitch*sin(angle) + preY;
        ob.push_back(temp_array);
        preX = temp_array[0];
        preY = temp_array[1];
        dx = x2 - preX;
        dy = y2 - preY;
        dis = sqrt(pow(dx, 2) + pow(dy, 2));
    }
}
int main()
{
    const int n = 50;
    const double goalR = 2;
    std::vector<double> goal_x{45};
    std::vector<double> goal_y{45};
    std::vector<double> goal_x{42};
    std::vector<double> goal_y{40};
    std::vector<double> goal{goal_x[0], goal_y[0]};
    std::vector<double> ini_x{5};
    std::vector<double> ini_y{5};
    std::vector<double> nowState{ini_x[0], ini_y[0], M_PI/2, 0, 0}; //[x, y, yaw, vel, yaw_vel]
    std::vector<std::vector<double>> ob;
    std::vector<double> ob_x, ob_y;
    std::vector<std::vector<double>> ini_move_ob{{5,15},{25,25},{40,50},{15,30}};
    std::vector<std::vector<double>> ini_move_ob{{5,15},{25,25},{35,50},{15,30}};
    std::vector<std::vector<double>> move_ob;
    double linearVel = 0;
    double yawVel = 0;
    std::vector<std::vector<double>> trajectory;
    obstract_line(ob, 0, 0, 0, n);
    obstract_line(ob, 0, n, n, n);
    obstract_line(ob, n, n, n, 0);
    obstract_line(ob, n, 0, 0, 0);
    obstract_line(ob, 15, 15, 0, 25);
    obstract_line(ob, 35, 35, 15, 40);
    obstract_line(ob, 10, 27, 40, 30);
    move_ob = ini_move_ob;
    ctr::DWA dwa;

    int step = 0;
    while(sqrt(pow(goal[0]-nowState[0], 2) + pow(goal[0]-nowState[0], 2)) > goalR)
    while(sqrt(pow(goal[0]-nowState[0], 2) + pow(goal[1]-nowState[1], 2)) > goalR)
    {
        dwa.dwa_controll(nowState, goal, ob);
        dwa.get_result(linearVel, yawVel, trajectory);
        nowState = motion(nowState, linearVel, yawVel);
        move_ob[0][0] = ini_move_ob[0][0] + 10*abs(sin(step*M_PI/180));
        move_ob[0][1] = ini_move_ob[0][1];
        move_ob[1][0] = ini_move_ob[1][0];
        move_ob[1][1] = ini_move_ob[1][1] + 25*abs(sin(step*M_PI/180));
        move_ob[1][1] = ini_move_ob[1][1] + 15*abs(sin(step*M_PI/180));
        move_ob[2][0] = ini_move_ob[2][0];
        move_ob[2][1] = ini_move_ob[2][1] - 15*abs(sin(step*M_PI/180));
        move_ob[3][0] = ini_move_ob[3][0] - 10*abs(sin(step*M_PI/180));
        move_ob[3][1] = ini_move_ob[3][1];
        //clear move_ob from ob
        for(int i = 0; i < move_ob.size(); i++){
            ob.pop_back();
        }
        for(int i = 0; i < move_ob.size(); i++){
            ob.push_back(move_ob[i]);
        }
        
        ob_x.clear();
        ob_y.clear();
        for(int i=0; i<ob.size(); i++){
            ob_x.push_back(ob[i][0]);
            ob_y.push_back(ob[i][1]);
        }
        // Clear previous plot
        plt::clf();
        plt::scatter(ob_x, ob_y); //obstract
        plt::plot(goal_x, goal_y, "xb"); //goal
        plt::plot(ini_x, ini_y, "xb"); //start
        plot_robot(nowState[0], nowState[1]); //robot
        plot_trajectory(trajectory); //path
        // Set x-axis to interval [0,n]
        plt::xlim(-10, n+10);
        plt::ylim(-10, n+10);
        // Add graph title
        plt::title("Dynamic Window Aprroach");
        // Display plot continuously
        plt::pause(0.001);
        step +=3;
        step +=2;
    }

    return 0;
}