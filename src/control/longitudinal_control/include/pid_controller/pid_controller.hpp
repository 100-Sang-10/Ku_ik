#ifndef __PID_CONTROLLER_HPP__
#define __PID_CONTROLLER_HPP__
#pragma once

// STD header
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS header
#include <ros/ros.h>

// Utility header
#include <util/ini_parser.hpp>
#include <util/longitudinal_control_type.hpp>

// Algorithm header

// Message header
#include <autoku_msgs/Trajectory.h>

// Config header
#include <pid_controller/pid_controller_config.hpp>

// Namespace
using namespace std;

class PIDController{
        // Constant variables
        const double TEMPLATEDOUBLE = 0.01;
    public:
        // Constructor
       	explicit PIDController();
        // Operator
        // Destructor
       	virtual ~PIDController();

       	bool Init();
      	void Update(VehicleState* vehicle_state,TargetState* target_state,autoku_msgs::Trajectory* best_trajectory);
		void ProcessINI();

        // - - - - - - - - - - - - - - Functions - - - - - - - - - - - - - - //
    public:
        // Algorithm functions 
        TemplateDataType TemplateAlgorithmFunction();
        VehicleControl PIDControl();
        double getVelocityError();

    private:    
        // Algorithm functions 
        TemplateDataType TemplateFunction();        

        // - - - - - - - - - - - - - - Variables - - - - - - - - - - - - - - //  
    private:      
        // Inputs
        TemplateDataType i_template_input_;
        VehicleState vehicle_state_;
        TargetState target_state_;
        autoku_msgs::Trajectory best_trajectory_;

        // Outputs
        TemplateDataType o_template_output_;
        VehicleControl control_cmd_;

        // Environments
        IniParser v_ini_parser_;

        // Configuration parameters
        // TemplateAlgorithmParameters template_algorithm_params_;
        PIDControllerParameter pid_gains_;
        
        // Variables
        double prevRosTime_ = 0;
        double prevError_ = 0;
        double integral_error_ = 0;
        double velocity_error_ = 0;
};

#endif // __TEMPLATE_ALGORITHM_HPP__