#ifndef __STANLEY_HPP__
#define __STANLEY_HPP__
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
#include <geometry_msgs/PoseStamped.h>

// Utility header
#include <util/ini_parser.hpp>
#include <util/lateral_control_type.hpp>

// Algorithm header


// Config header
#include <stanley/stanley_config.hpp>

// Namespace
using namespace std;

class StanleyController{
        // Constant variables
        const double TEMPLATEDOUBLE = 0.01;
    public:
        // Constructor
       	explicit StanleyController();
        // Operator
        // Destructor
       	virtual ~StanleyController();

       	bool Init();
      	void Update(VehicleLocalState* vehicle_pose,autoku_msgs::Trajectory* best_trajectory,int* loop_count);
		void ProcessINI();

        // - - - - - - - - - - - - - - Functions - - - - - - - - - - - - - - //
    public:
        // Algorithm functions 
        TemplateDataType TemplateAlgorithmFunction();
        VehicleControl Stanley();
        void findClosestPoint();
        void findYawError();
        void findCrossTrackError();
        LookAheadState getClosestPoint();
        double getCrossTrackError();
        double getYawError();
        double trueToPlatformCommand(double steer);  
    private:    
        // Algorithm functions 
        TemplateDataType TemplateFunction();
        double degreeToRadian(double degree);
        double radianToDegree(double radian);
              

        // - - - - - - - - - - - - - - Variables - - - - - - - - - - - - - - //  
    private:      
        // Inputs
        TemplateDataType i_template_input_;
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
        StanleyParameters stanley_params_;

        // Variables
        bool isForward_;
        LookAheadState closest_point_;
        double yaw_error_;
        double cross_track_error_;
        autoku_msgs::TrajectoryPoint prevTrajectoryFirst_;
        tf::TransformListener tfListenr;


};

#endif // __STANLEY_HPP__