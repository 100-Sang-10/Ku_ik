#ifndef __LATERAL_CONTROL_HPP__
#define __LATERAL_CONTROL_HPP__
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
#include <util/lateral_control_type.hpp>

// Message header
#include <autoku_msgs/VehicleLateralControl.h>
#include <autoku_msgs/PlatformState.h>
#include <autoku_msgs/VehicleState.h>
#include <autoku_msgs/Trajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

// Algorithm header
#include <pure_pursuit/pure_pursuit.hpp>
#include <pure_pursuit_time_delay/pure_pursuit_time_delay.hpp>
#include <stanley/stanley.hpp>
#include <interpolating_path/interpolating_path.hpp>

// Config header
#include <lateral_control_config.hpp>

// Namespace
using namespace ros; 
using namespace tf;
using namespace std;

class LateralControl{
        // Constant variables
    public:
        // Constructor
       	explicit LateralControl();
        // Operator
        // Destructor
       	virtual ~LateralControl();

       	void Init();
      	void Run();
      	void Publish();
		void Terminate();
		void ProcessINI();
        double clipMaxSteer(double steer);
        void combineSteeringCommand();

        // - - - - - - - - - - - - - - Functions - - - - - - - - - - - - - - //
    private:    
        // Callback functions         
        void getVehicleLocalState(const autoku_msgs::VehicleState::ConstPtr& msg);
        void getLocalTrajectory(const autoku_msgs::Trajectory::ConstPtr& msg);
        void getPlatformState(const autoku_msgs::PlatformState::ConstPtr& msg);
        void getLoopCount(const std_msgs::Int32::ConstPtr& msg);

        // Get functions for subscribe variables      
        VehicleLocalState GetVehicleLocalState();

        // Transform functions
		// Update functions for publish variables
        void UpdateControlCommand(const VehicleControl& command_update);
        void UpdateLookaheadPoint(const LookAheadState lookahead_point,const LookAheadState closest_point, VehicleLocalState vehicle_pose);
        void UpdateMaxtrackError(double current_track_error);
        void UpdateFrontDistCircle(double traj_front_dist);
        void UpdateRvizPath();
        // Local Function
        
        // - - - - - - - - - - - - - - Variables - - - - - - - - - - - - - - //    
    private:    
    	// Subscriber
        Subscriber vehicle_local_state_subscriber_;
        Subscriber platform_state_subscriber_;
        Subscriber local_trajectory_subscriber_;
        Subscriber loop_count_subscriber_;

        // Publisher
        Publisher control_command_publisher_;
        Publisher control_command_arduino_publisher_;
        Publisher lookahead_point_publisher_;
        Publisher closest_point_publisher_;
        Publisher cross_track_error_publisher_;
        Publisher yaw_error_publisher_;
        Publisher max_track_error_publisher_;
        Publisher interpolating_path_publisher_;
        Publisher traj_front_circle_publisher_;
        Publisher system_time_publisher_;

        // Inputs
        autoku_msgs::VehicleState vehicle_local_state_;
        autoku_msgs::PlatformState platform_state_;
        autoku_msgs::Trajectory best_trajectory_;
        std_msgs::Int32 loop_count_;

        // Mutex
        mutex mutex_local_state_;
        mutex mutex_platform_state_;
        mutex mutex_trajectory_;
        mutex mutex_loop_count_;

        // Outputs
        autoku_msgs::VehicleLateralControl control_command_output_;
        std_msgs::Float32 control_command_arduino_output_;
        visualization_msgs::Marker lookahead_point_;
        visualization_msgs::Marker closest_point_;
        visualization_msgs::Marker predicted_pose_after_time_delay_;
        visualization_msgs::Marker front_dist_circle_;
        visualization_msgs::MarkerArray interpolating_path_marker_;
        std_msgs::Float32 cross_track_error_;
        std_msgs::Float32 yaw_error_;
        std_msgs::Float32 max_track_error_;
        std_msgs::Float64 o_system_time_;

        // Environments
        IniParser v_ini_parser_;
        TransformListener tf_listener_;

        unique_ptr<PurePursuitController> pure_pursuit_algorithm_;
        unique_ptr<PurePursuitTimeDelay> pure_pursuit_time_delay_algorithm_;
        unique_ptr<StanleyController> stanley_method_;

        VehicleControl pure_pursuit_command;
        VehicleControl stanley_command;
        VehicleControl combined_command;
        
        bool prevAuto_ = false;
        bool currentAuto_ = false;

        double currentCurvature = 0;
        double prevCurvature = 0;
        double lowPassFilter_alpha = 0.5;

        string mode_;

        // Configuration parameters
        LateralControlParameters control_params_;
};

#endif // __LATERAL_CONTROL_HPP__