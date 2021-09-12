/**
* @file DWA.h
* @brief dynamic window approach
* @author Michikuni Eguchi
* @date 2021.8.9
* @details reference : https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py
*                      https://qiita.com/MENDY/items/16343a00d37d14234437
**/

#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "DWA_config.h"

namespace ctr{

class DWA
{
public:
    DWA();
    //set now state of robot, goal posion value, object position
    void dwa_controll(const std::vector<double>& motion_state, const std::vector<double>& goal_, const std::vector<std::vector<double>>& obstacle_);
    void get_result(double& resultLinearVel, double& resultAngularVel, std::vector<std::vector<double>>& resultTrajectory);
private:
    struct motionState{
        double x;
        double y;
        double yawAngle;
        double vel;
        double yawVel;
    };
    motionState state;

    ///////dwa config///////
    double maxSpeed;
    double minSpeed;
    double maxYaw_rate;
    double maxAccel;
    double maxYawVel_rate;
    double v_resolution;
    double w_resolution;
    double dt;
    double predictTime;
    double goalScore_gain;
    double obstacleScore_gain;
    double speedScore_gain;
    double robot_stuck_value;
    double robot_radius;
    /////////////////////////

    std::vector<double> goal;
    std::vector<std::vector<double>> obstacle;

    double bestLinearVel;
    double bestAngularVel;
    std::vector<std::vector<double>> bestTrajectory;

    //set dynamic window
    std::vector<double> calc_dynamicWindow();
    //calc best velocity and trajectory
    void calc_controll_and_trajectory(std::vector<double> dw);
    //predict trajectory with velocity and w
    std::vector<std::vector<double>> predict_trajectory(double v, double w);
    //calculate position in trajectory from velocity, angular velocity and dt
    motionState motion(motionState predict_state, double v, double w);
    //calculate goal score from goal position and robot position
    double goalScore(const std::vector<std::vector<double>>& trajectory);
    //calculate obstacle score from object position
    double obstacleScore(const std::vector<std::vector<double>>& trajectory);
    //calculate speed score from robot velocity
    double speedScore(const std::vector<std::vector<double>>& trajectory);
    //arrange angle -pi ~ +pi
    double arrangeAngle(double angle);
    //normalize (0~1)
    template <class T> std::vector<T> normalize(std::vector<T>& num);

};

DWA::DWA()
{
    //set config value
    maxSpeed = DWA_config::maxSpeed;
    minSpeed = DWA_config::minSpeed;
    maxYaw_rate = DWA_config::maxYaw_rate;
    maxAccel = DWA_config::maxAccel;
    maxYawVel_rate = DWA_config::maxYawVel_rate;
    v_resolution = DWA_config::v_resolution;
    w_resolution = DWA_config::w_resolution;
    dt = DWA_config::dt;
    predictTime = DWA_config::predictTime;
    goalScore_gain = DWA_config::goalScore_gain;
    obstacleScore_gain = DWA_config::obstacleScore_gain;
    speedScore_gain = DWA_config::speedScore_gain;
    robot_stuck_value = DWA_config::robot_stuck_value;
    robot_radius = DWA_config::robot_radius;
}

void DWA::dwa_controll(const std::vector<double>& motion_state, const std::vector<double>& goal_, const std::vector<std::vector<double>>& obstacle_)
{
    state.x = motion_state[0];
    state.y = motion_state[1];
    state.yawAngle = motion_state[2];
    state.vel = motion_state[3];
    state.yawVel = motion_state[4];
    goal = goal_;
    obstacle = obstacle_;

    std::vector<double> dynamic_window = calc_dynamicWindow();
    calc_controll_and_trajectory(dynamic_window);

}

void DWA::get_result(double& resultLinearVel, double& resultAngularVel, std::vector<std::vector<double>>& resultTrajectory)
{
    resultLinearVel = bestLinearVel;
    resultAngularVel = bestAngularVel;
    resultTrajectory = bestTrajectory;
}

std::vector<double> DWA::calc_dynamicWindow()
{
    std::vector<double> vs{minSpeed, maxSpeed, -maxYaw_rate, maxYaw_rate};

    std::vector<double> vd{state.vel - maxAccel*dt,
                           state.vel + maxAccel*dt,
                           state.yawVel - maxYawVel_rate*dt,
                           state.yawVel + maxYawVel_rate*dt};

    std::vector<double> dw{std::max(vs[0], vd[0]),
                           std::min(vs[1], vd[1]),
                           std::max(vs[2], vd[2]),
                           std::min(vs[3], vd[3])};

    return dw;
}

void DWA::calc_controll_and_trajectory(std::vector<double> dw)
{
    double goal_score, speed_score, obstacle_score, final_score;
    std::vector<double> goal_scores, speed_scores, obstacle_scores;
    std::vector<std::vector<double>> velocity_com; //[v,w],[v,w].....
    std::vector<double> tmp_array(2); //[v, w]
    std::vector<std::vector<double>> trajectory;


    double maxScore = -INFINITY;
    bestLinearVel = 0;
    bestAngularVel = 0;

    //search all trajectory in dynamic window
    for(double v = dw[0]; v <= dw[1]; v += v_resolution){
        for(double w = dw[2]; w <= dw[3]; w += w_resolution){
            trajectory = predict_trajectory(v, w);

            if(goalScore(trajectory) == -INFINITY){
                break;
            }
            tmp_array[0] = v;
            tmp_array[1] = w;
            velocity_com.push_back(tmp_array);

            //calculate each score
            goal_scores.push_back(goalScore(trajectory));
            speed_scores.push_back(speedScore(trajectory));
            obstacle_scores.push_back(obstacleScore(trajectory));
        }
    }

    //score normalize
    goal_scores = normalize(goal_scores);
    speed_scores = normalize(speed_scores);
    obstacle_scores = normalize(obstacle_scores);

    //select largest score trajectory
    for(int i=0; i<goal_scores.size(); i++){
        goal_score = goalScore_gain * goal_scores[i];
        speed_score = speedScore_gain * speed_scores[i];
        obstacle_score = obstacleScore_gain * obstacle_scores[i];

        final_score = goal_score + speed_score + obstacle_score;

        if(final_score > maxScore){
            maxScore = final_score;
            bestLinearVel = velocity_com[i][0];
            bestAngularVel = velocity_com[i][1];
            bestTrajectory = predict_trajectory(velocity_com[i][0], velocity_com[i][1]);

            //if robot stuck, avoid stuck by big turn
            if(bestLinearVel < robot_stuck_value && state.vel < robot_stuck_value){
                bestAngularVel = maxYawVel_rate/2;
            }
        }
    }



}

std::vector<std::vector<double>> DWA::predict_trajectory(double v, double w)
{
    std::vector<std::vector<double>> trajectory;
    motionState predict_state = state;
    std::vector<double> tmp_array(5);

    for(double t = 0; t <= predictTime; t += dt){
        predict_state = motion(predict_state, v, w);

        tmp_array[0] = predict_state.x;
        tmp_array[1] = predict_state.y;
        tmp_array[2] = predict_state.yawAngle;
        tmp_array[3] = predict_state.vel;
        tmp_array[4] = predict_state.yawVel;
        trajectory.push_back(tmp_array);

    }

    return trajectory;

}

DWA::motionState DWA::motion(motionState predict_state, double v, double w)
{
    predict_state.yawAngle += w*dt;
    predict_state.x += v*cos(predict_state.yawAngle)*dt;
    predict_state.y += v*sin(predict_state.yawAngle)*dt;
    predict_state.vel = v;
    predict_state.yawVel = w;

    return predict_state;
}

double DWA::goalScore(const std::vector<std::vector<double>>& trajectory)
{
    double dx = goal[0] - trajectory.back()[0];
    double dy = goal[1] - trajectory.back()[1];
    double score_angle = atan2(dy, dx) - trajectory.back()[2];
    score_angle = abs(arrangeAngle(score_angle));

    return M_PI - score_angle;
}

double DWA::obstacleScore(const std::vector<std::vector<double>>& trajectory)
{
    double r;
    double minR = INFINITY;
    double dx, dy;

    for(int i = 0; i < trajectory.size(); i++){
        for(int j = 0; j < obstacle.size(); j++){
            dx = trajectory[i][0] - obstacle[j][0];
            dy = trajectory[i][1] - obstacle[j][1];
            r = sqrt(pow(dx, 2) + pow(dy, 2));

            if(minR > r){
                minR = r;
            }
        }
    }

    if(minR < 1){
        return -INFINITY;
    }else{
        return minR;
    }
}

double DWA::speedScore(const std::vector<std::vector<double>>& trajectory)
{
    return trajectory.back()[3];
}

double DWA::arrangeAngle(double angle)
{
    while(angle>M_PI)
    {
        angle -= 2*M_PI;
    }
    while(angle<-M_PI)
    {
        angle += 2*M_PI;
    }

    return angle;
}

template <class T> std::vector<T> DWA::normalize(std::vector<T>& num)
{
    T xmin = *std::min_element(std::begin(num), std::end(num));
    T xmax = *std::max_element(std::begin(num), std::end(num));

    for(auto& n : num){
        n = (n - xmin) / (xmax - xmin);
    }

    return num;
}

} //namespace ctr