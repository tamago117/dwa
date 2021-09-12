#include<iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include "include/matplotlibcpp.h"
#include "include/a_star.h"

namespace plt = matplotlibcpp;
const double resolution = 1;
const double robotRadius = 2.0;

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
    std::vector<double> goal_x{40};
    std::vector<double> goal_y{40};
    std::vector<double> goal{goal_x[0], goal_y[0]};
    std::vector<double> ini_x{4};
    std::vector<double> ini_y{4};
    std::vector<std::vector<double>> ob;
    std::vector<double> ob_x, ob_y;
    std::vector<double> path_x, path_y;

    obstract_line(ob, 0, 0, 0, n);
    obstract_line(ob, 0, n, n, n);
    obstract_line(ob, n, n, n, 0);
    obstract_line(ob, n, 0, 0, 0);
    obstract_line(ob, 15, 15, 0, 25);
    obstract_line(ob, 35, 35, 15, 50);
    obstract_line(ob, 10, 30, 43, 13);

    ctr::a_star a_star(resolution, robotRadius);

    for(int i=0; i<ob.size(); i++){
        ob_x.push_back(ob[i][0]);
        ob_y.push_back(ob[i][1]);
    }

    a_star.planning(path_x, path_y, ini_x[0], ini_y[0], goal_x[0], goal_y[0], ob_x, ob_y);

    // Clear previous plot
    plt::clf();
    plt::scatter(ob_x, ob_y); //obstract
    plt::plot(goal_x, goal_y, "xb"); //goal
    plt::plot(ini_x, ini_y, "xb"); //start
    plt::plot(path_x, path_y); //path

    // Set x-axis to interval [0,n]
    plt::xlim(-10, n+10);
    plt::ylim(-10, n+10);

    // Add graph title
    plt::title("A*");
    // Display plot continuously
    //plt::pause(0.001);
    // show plots
    plt::show();

    return 0;
}