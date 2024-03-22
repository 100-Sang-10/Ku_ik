#ifndef __LONGITUDINAL_CONTROL_CONFIG_HPP__
#define __LONGITUDINAL_CONTROL_CONFIG_HPP__
#pragma once

typedef struct {
    int control_test_mode;
    float target_velocity;
    float target_steering;
} ControlParameters;

#endif  // __LONGITUDINAL_CONTROL_CONFIG_HPP__