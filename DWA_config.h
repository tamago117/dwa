#pragma once
#include <math.h>

namespace DWA_config{

    const double maxSpeed = 2.0;
    const double minSpeed = -0.5;
    const double maxYaw_rate = 50 * M_PI/180;
    const double maxAccel = 0.8;
    const double maxYawVel_rate = 40 * M_PI/180;
    const double v_resolution = 0.02;
    const double w_resolution = 0.2 * M_PI/180;
    const double dt = 0.3;
    const double predictTime = 3.0;
    const double goalScore_gain = 0.3;
    const double obstacleScore_gain = 0.35;
    const double speedScore_gain = 0.2;
    const double robot_stuck_value = 0.001;
    const double robot_radius = 1.0;

}//namespace DWA