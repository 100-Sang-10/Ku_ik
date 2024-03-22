#ifndef __PURE_PURSUIT_HPP__
#define __PURE_PURSUIT_HPP__
#pragma once

// STD header
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <boost/optional.hpp>
#include <fstream>
// #include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
  
// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Message header
#include <autoku_msgs/Trajectory.h>
#include <autoku_msgs/TrajectoryPoint.h>
#include <geometry_msgs/PointStamped.h>

// Utility header
#include <util/ini_parser.hpp>
#include <util/lateral_control_type.hpp>

// Algorithm header


// Config header
#include <pure_pursuit/pure_pursuit_config.hpp>

// Namespace
using namespace std;

class PurePursuitController{
        // Constant variables
        const double TEMPLATEDOUBLE = 0.01;
    public:
        // Constructor
       	explicit PurePursuitController();
        // Operator
        // Destructor
       	virtual ~PurePursuitController();

       	bool Init();
      	void Update(VehicleLocalState* vehicle_pose, autoku_msgs::Trajectory* best_trajectory, int* loop_count);
		void ProcessINI();

        // - - - - - - - - - - - - - - Functions - - - - - - - - - - - - - - //
    public:
        // Algorithm functions 
        TemplateDataType TemplateAlgorithmFunction();
        VehicleControl PurePursuit();
        LookAheadState GetLookaheadPoint();
        LookAheadState getClosestPoint();
        double getCrossTrackError();
        double getYawError();
        double getFrontTrackError();
        double getRearTrackError();
        double trueToPlatformCommand(double steer);

    private:    
        // Algorithm functions 
        TemplateDataType TemplateFunction();
        void getLookAheadPoint();
        void findCrossTrackError();
        double setLookAheadDistance();
        double degreeToRadian(double degree);
        double radianToDegree(double radian);
        void findClosestPoint();
        // - - - - - - - - - - - - - - Variables - - - - - - - - - - - - - - //  
    private:      
        // Inputs
        VehicleLocalState vehicle_pose_;
        // LookAheadState lookahead_state_;
        autoku_msgs::Trajectory best_trajectory_;
        int loop_count_;

        // Outputs
        TemplateDataType o_template_output_;
        VehicleControl control_cmd_;

        // Environments
        IniParser v_ini_parser_;

        // Configuration parameters
        PurePursuitParameters pure_pursuit_params_;

        // Variables
        bool isForward_;
        LookAheadState lookahead_point_;
        LookAheadState closest_point_;
        LookAheadState front_closest_;
        LookAheadState ego_front_world_;
        double cross_track_error_ = 0;     
        double rear_track_error_ = 0;  
        double prev_lookAhead_point_s_ = 0;
        autoku_msgs::TrajectoryPoint prevTrajectoryFirst_;
        tf::TransformListener tfListenr;

};

#endif // __PURE_PURSUIT_HPP__