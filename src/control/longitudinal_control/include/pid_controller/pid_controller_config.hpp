#ifndef __PID_CONTROLLER_CONFIG_HPP__
#define __PID_CONTROLLER_CONFIG_HPP__
#pragma once

typedef struct {
    double p_gain;
    double i_gain;
    double d_gain;
    double p_gain_brake;
    double i_gain_brake;
    double d_gain_brake;
    double brake_sector_1;
    double brake_sector_2;
    double brake_sector_3;
    double brake_sector_4;
    double brake_sector1_value;
    double brake_sector2_value;
    double brake_sector3_value;
    double brake_sector4_value;
    double brake_sector5_value;
} PIDControllerParameter;

#endif  // __TEMPLATE_ALGORITHM_CONFIG_HPP__