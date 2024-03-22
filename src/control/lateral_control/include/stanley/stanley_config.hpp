#ifndef __STANLEY_CONFIG_HPP__
#define __STANLEY_CONFIG_HPP__
#pragma once

typedef struct {
    double stanley_yaw_error_gain;
    double stanley_lookahead;
    double stanley_velocity_gain;
    double stanley_curvature_gain;
    double stanley_den_gain;
    double stanley_cte_gain;
    double backward_den_gain;
    double backward_cte_gain;
    double wheel_base;
    double a3;
    double a2;
    double a1;
    double a0;
    double max_steer;
} StanleyParameters;

#endif  // __STANLEY_CONFIG_HPP__