#ifndef __LONGITUDINAL_CONTROL_HPP__
#define __LONGITUDINAL_CONTROL_HPP__
#pragma once

// STD header
#include <mutex>
#include <vector>
#include <utility>
#include <memory>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// Utility header
#include <util/ini_parser.hpp>
#include <util/longitudinal_control_type.hpp>

// Message header
#include <autoku_msgs/VehicleLongitudinalControl.h>
#include <autoku_msgs/VehicleState.h>
#include <autoku_msgs/Trajectory.h>
#include <autoku_msgs/PlatformState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>


// Algorithm header
#include <pid_controller/pid_controller.hpp>

// Config header
#include <longitudinal_control_config.hpp>

// Namespace
using namespace ros; 
using namespace tf;
using namespace std;

class LongitudinalControl{
        // Constant variables
        const double TEMPLATEDOUBLE = 0.01;
    public:
        // Constructor
       	explicit LongitudinalControl();
        // Operator
        // Destructor
       	virtual ~LongitudinalControl();

       	void Init();
      	void Run();
      	void Publish();
		void Terminate();
		void ProcessINI();

        // - - - - - - - - - - - - - - Functions - - - - - - - - - - - - - - //
    private:    
        // Callback functions         
        void getVehicleLocalState(const autoku_msgs::VehicleState::ConstPtr& msg);
        void getLocalTrajectory(const autoku_msgs::Trajectory::ConstPtr& msg);
        
        // Get functions for subscribe variables      
        TargetState GetTargetState();
        VehicleState GetVehicleState();

        // Transform functions
        
		// Update functions for publish variables
        void UpdateControlCommand(const VehicleControl& command_update);
        void UpdateVeloccityError(double current_velocity_error);
        // - - - - - - - - - - - - - - Variables - - - - - - - - - - - - - - //    
    private:    
    	// Subscriber
        Subscriber vehicle_local_state_subscriber_;
        Subscriber local_trajectory_subscriber_;

        // Publisher
        Publisher control_command_publisher_;
        Publisher control_command_arduino_publisher_;
        Publisher visulize_command_publisher_;
        Publisher visulize_target_publisher_;
        Publisher velocity_error_publisher_;
        Publisher system_time_publisher_;

        // Inputs
        autoku_msgs::Trajectory best_trajectory_;
        autoku_msgs::VehicleState vehicle_local_state_;

        string mode_;

        // Mutex
        mutex mutex_trajectory_;
        mutex mutex_state_;
        mutex mutex_target_state_;

        // Outputs
        autoku_msgs::VehicleLongitudinalControl control_command_output_;
        std_msgs::Float32 control_command_arduino_output_;
        std_msgs::Float32 visualize_command_output_;
        std_msgs::Float32 visualize_target_output_;
        std_msgs::Float32 velocity_error_output_;
        std_msgs::Float64 o_system_time_;

        // Environments
        IniParser v_ini_parser_;
        TransformListener tf_listener_;

        unique_ptr<PIDController> pid_controller_algorithm_;

        // Configuration parameters
        ControlParameters control_params_;
};

#endif // __LONGITUDINAL_CONTROL_HPP__