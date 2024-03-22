#ifndef __LATERAL_CONTROL_CONFIG_HPP__
#define __LATERAL_CONTROL_CONFIG_HPP__
#pragma once

typedef struct {
    int control_test_mode;
    int lateral_method;
    float target_steering;
    double combine_factor;
    double lowPassFilter_alpha;
    double pure_stanley_toggle_vel;
    int use_leastsquare;
    double max_steer;
    int backward_method;
    bool pure_pursuit_time_delay;
    double pure_pursuit_toggle_boundary;
} LateralControlParameters;

#endif  // __LATERAL_CONTROL_CONFIG_HPP__