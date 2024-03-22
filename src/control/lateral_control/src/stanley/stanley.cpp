#include "stanley/stanley.hpp"

StanleyController::StanleyController() {
    
}

StanleyController::~StanleyController(){

}

bool StanleyController::Init(){
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
	ProcessINI();

    return true;
}

void StanleyController::Update(VehicleLocalState* vehicle_pose,autoku_msgs::Trajectory* best_trajectory,int* loop_count){	
    vehicle_pose_ = *vehicle_pose;
    best_trajectory_ = *best_trajectory;
    loop_count_ = *loop_count;
    if(best_trajectory_.point.front().speed >= 0){
        isForward_ = true;
    }else{
        isForward_ = false;
    }
	ProcessINI();
}

void StanleyController::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("lateral stanley", "stanley_yaw_error_gain",
                                    stanley_params_.stanley_yaw_error_gain);
        v_ini_parser_.ParseConfig("lateral stanley", "stanley_lookahead",
                                    stanley_params_.stanley_lookahead);
        v_ini_parser_.ParseConfig("lateral stanley", "stanley_velocity_gain",
                                    stanley_params_.stanley_velocity_gain);                            
        v_ini_parser_.ParseConfig("lateral stanley", "stanley_den_gain",
                                    stanley_params_.stanley_den_gain);   
        v_ini_parser_.ParseConfig("lateral stanley", "stanley_curvature_gain",
                                    stanley_params_.stanley_curvature_gain);                                                      
        v_ini_parser_.ParseConfig("lateral stanley", "stanley_cte_gain",
                                    stanley_params_.stanley_cte_gain);                            
        v_ini_parser_.ParseConfig("lateral stanley", "backward_den_gain",
                                    stanley_params_.backward_den_gain); 
        v_ini_parser_.ParseConfig("lateral stanley", "backward_cte_gain",
                                    stanley_params_.backward_cte_gain); 
        v_ini_parser_.ParseConfig("lateral", "wheel_base",
                                    stanley_params_.wheel_base);
        v_ini_parser_.ParseConfig("lateral", "max_steer",
                                    stanley_params_.max_steer);                            
        v_ini_parser_.ParseConfig("lateral", "a3",
                                    stanley_params_.a3);
        v_ini_parser_.ParseConfig("lateral", "a2",
                                    stanley_params_.a2);
        v_ini_parser_.ParseConfig("lateral", "a1",
                                    stanley_params_.a1);
        v_ini_parser_.ParseConfig("lateral", "a0",
                                    stanley_params_.a0);

        ROS_WARN("[Stanley Algorithm] Ini file is updated!");
    }
}

void StanleyController::findClosestPoint(){
    
    // Set Min Distance for Get Closest Point with Ego Front
    double min_distance = HUGE_VAL;
    double max_distance = -HUGE_VAL;
    
    // Want to use Lookahead Point, Not Closest.
    double lookahead_distance = stanley_params_.stanley_lookahead 
                                + (vehicle_pose_.speed/3.6)*stanley_params_.stanley_velocity_gain;
    
    if(loop_count_ > 0){

        // After Make Loop, Use Point's Distance
        // Find Closest Point
        for(int i=0;i<best_trajectory_.point.size();i++){
            
            if(best_trajectory_.point[i].distance >= lookahead_distance + stanley_params_.wheel_base){
                
                // Trajectory is already sorted by distance, so when we get point, Break for loop.
                closest_point_.x = best_trajectory_.point[i].x;
                closest_point_.y = best_trajectory_.point[i].y;
                closest_point_.target_yaw = best_trajectory_.point[i].yaw;

                break;
            }
            else{
                // If Path's distance is below lookahead distance, select most far point.
                closest_point_.x = best_trajectory_.point.back().x;
                closest_point_.y = best_trajectory_.point.back().y;
                closest_point_.target_yaw = best_trajectory_.point.back().yaw;            
            }
        }
    }else{
        // Before Make Loop, Use Euclidian Distance
        geometry_msgs::PointStamped ego_front_zero;
        ego_front_zero.header.frame_id = "/ego_front";
        ego_front_zero.point.x = 0.;
        ego_front_zero.point.y = 0.;
        ego_front_zero.point.z = 0.;
        geometry_msgs::PointStamped ego_front_zero_world;
        try{
            tfListenr.transformPoint("/world",ego_front_zero,ego_front_zero_world);
        }catch (tf::TransformException &ex) {
            // ROS_ERROR(ex.what());
        }
        // Assumption that Path is always longer that ego front.(lookahead distance)
        // Trajectory is longer than lookahead distance
        for(int i=0;i<best_trajectory_.point.size();i++){

            // Get Euclidian Distance from Ego Front to each trajectory point
            double temp_distance = 
                    sqrt(pow(ego_front_zero_world.point.x - best_trajectory_.point[i].x,2) +
                        pow(ego_front_zero_world.point.y - best_trajectory_.point[i].y,2));
            
            // Check if closest point is forward than ego front.
            geometry_msgs::PointStamped trajectory_world;
            trajectory_world.header.frame_id = "/world";
            trajectory_world.point.x = best_trajectory_.point[i].x;
            trajectory_world.point.y = best_trajectory_.point[i].y;
            trajectory_world.point.z = 0.;
            geometry_msgs::PointStamped trajectory_ego_front;
            try{
                tfListenr.transformPoint("/ego_front",trajectory_world,trajectory_ego_front);
            }catch (tf::TransformException &ex) {
                // ROS_ERROR(ex.what());
            }
            // Find Closest Point from ego front and forward than ego front
            if(temp_distance < min_distance && trajectory_ego_front.point.x > 0){
                closest_point_.x = best_trajectory_.point[i].x;
                closest_point_.y = best_trajectory_.point[i].y;
                closest_point_.target_yaw = best_trajectory_.point[i].yaw;
                closest_point_.distance = temp_distance; // Distance between Lookahead and Vehicle Pose
                min_distance = temp_distance;
            }              
        }
    }
}

LookAheadState StanleyController::getClosestPoint(){
    return closest_point_;
}

double StanleyController::getCrossTrackError(){
    return cross_track_error_;
}

double StanleyController::getYawError(){
    return yaw_error_;
}

void StanleyController::findCrossTrackError(){
    
    double cross_track_error;
    // Transform Closest Point (World) to Ego to find lateral deviation
    geometry_msgs::PointStamped closest_point_world;
    closest_point_world.header.frame_id = "/world";
    closest_point_world.point.x = closest_point_.x;
    closest_point_world.point.y = closest_point_.y;
    closest_point_world.point.z = 0.;
    geometry_msgs::PointStamped closest_point_ego;
    try{
    tfListenr.transformPoint("/ego_front",closest_point_world,closest_point_ego);
    }catch (tf::TransformException &ex) {
        // ROS_ERROR(ex.what());
    }

    // If Backward(180 degree rotation), error's direction is upside down.
    if(isForward_){
        cross_track_error = closest_point_ego.point.y;
    }else{
        cross_track_error = -closest_point_ego.point.y;
    }

    cross_track_error_ = cross_track_error;
}

void StanleyController::findYawError(){
    // Return Value
    double yaw_error;

    // Transform closest Pose to calculate yaw error
    geometry_msgs::PoseStamped closest_pose_world;
    closest_pose_world.header.frame_id = "/world";
    closest_pose_world.pose.position.x = closest_point_.x;
    closest_pose_world.pose.position.y = closest_point_.y;
    closest_pose_world.pose.position.z = 0.;
    tf::Quaternion q;
    q.setRPY(0.0,0.0,closest_point_.target_yaw);
    ROS_WARN_STREAM("TARGET YAW WORLD : " << closest_point_.target_yaw*(180/M_PI));
    closest_pose_world.pose.orientation.x = q[0];
    closest_pose_world.pose.orientation.y = q[1];
    closest_pose_world.pose.orientation.z = q[2];
    closest_pose_world.pose.orientation.w = q[3];
    geometry_msgs::PoseStamped closest_pose_ego;
    try{
        tfListenr.transformPose("/ego_front",closest_pose_world,closest_pose_ego);
    }catch (tf::TransformException &ex) {
        ROS_WARN("Catch Yaw Pose Transform Error!");
        // ROS_ERROR(ex.what());
    }

    // Get Ego Front Coordinate Yaw from Pose Quaternion
    tf::Quaternion q_ego(
        closest_pose_ego.pose.orientation.x,
        closest_pose_ego.pose.orientation.y,
        closest_pose_ego.pose.orientation.z,
        closest_pose_ego.pose.orientation.w);
    q_ego.normalize();
    tf::Matrix3x3 m(q_ego);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_ERROR_STREAM("TARGET YAW EGO : " << yaw*(180/M_PI));

    yaw_error = yaw;

    // Normalize Yaw to -180 ~ 180
    if(yaw_error > M_PI){
        yaw_error -= 2*M_PI;
    }else if(yaw_error < -M_PI){
        yaw_error += 2*M_PI;
    }
    if(!isForward_){
        yaw_error  = yaw_error + M_PI;
        // Normalize Yaw to -180 ~ 180
        if(yaw_error > M_PI){
            yaw_error -= 2*M_PI;
        }else if(yaw_error < -M_PI){
            yaw_error += 2*M_PI;
        }
    }
        
    yaw_error_ = yaw_error;
}

VehicleControl StanleyController::Stanley(){
    
    // Find Closest Point, Update Yaw Error and Cross Track Error
    findClosestPoint();
    findYawError();
    findCrossTrackError();

    // Stanley command Gain is different via forward and backward
    if(isForward_){
        
        // Forward Stanley
        control_cmd_.steer = 
        stanley_params_.stanley_yaw_error_gain*yaw_error_
        + atan2(stanley_params_.stanley_cte_gain * cross_track_error_,vehicle_pose_.speed/3.6 + stanley_params_.stanley_den_gain + 1.0);
        
        // Steer Direction
        control_cmd_.steer = -radianToDegree(control_cmd_.steer);
    }else{

        // Backward Stanley
        control_cmd_.steer = 
        stanley_params_.stanley_yaw_error_gain*yaw_error_ 
        + atan2(stanley_params_.backward_cte_gain * cross_track_error_,vehicle_pose_.speed/3.6 + stanley_params_.backward_den_gain + 1.0);
        
        // Steer Direction
        control_cmd_.steer = -radianToDegree(control_cmd_.steer);
        
        // Backward Direction is upside down.
        control_cmd_.steer = -control_cmd_.steer;
    }
    
    return control_cmd_;
}

double StanleyController::degreeToRadian(double degree){
    return degree * M_PI / 180;
}

double StanleyController::radianToDegree(double radian){
    return radian * 180 / M_PI;
}

double StanleyController::trueToPlatformCommand(double steer){
    
    // Calculate True2Command Value
    double command_steer = stanley_params_.a3*pow(steer,3) + stanley_params_.a2*pow(steer,2) 
            + stanley_params_.a1*steer + stanley_params_.a0;

    return command_steer;
}