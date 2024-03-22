#ifndef __PURE_PURSUIT_CONFIG_HPP__
#define __PURE_PURSUIT_CONFIG_HPP__
#pragma once

typedef struct {
    double min_lookAhead_distance;
    double pure_pursuit_cte;
    double velocity_sector_1;
    double velocity_sector_2;
    double velocity_sector_3;
    double velocity_sector_4;
    double curvature_gain_ld;
    double velocity_sector1_ld;
    double velocity_sector2_ld;
    double velocity_sector3_ld;
    double velocity_sector4_ld;
    double wheel_base;
    double a3;
    double a2;
    double a1;
    double a0;
    double max_steer;
    double pure_pursuit_den;
    double time_delay;
    double velocity_coefficient;
} PurePursuitParameters;

#endif  // __PURE_PURSUIT_CONFIG_HPP__
