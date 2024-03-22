#include "pure_pursuit/pure_pursuit.hpp"

PurePursuitController::PurePursuitController() {
    
}

PurePursuitController::~PurePursuitController(){

}

bool PurePursuitController::Init(){
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
	ProcessINI();

    return true;
}

void PurePursuitController::Update(VehicleLocalState* vehicle_pose,autoku_msgs::Trajectory* best_trajectory,int* loop_count){	
    vehicle_pose_ = *vehicle_pose;
    best_trajectory_ = *best_trajectory;
    loop_count_ = *loop_count;
    if(best_trajectory_.point.front().speed >= 0){
        isForward_ = true;
    }else{
        isForward_ = false;
        vehicle_pose_.yaw  = vehicle_pose_.yaw + M_PI;
        // Normalize Yaw to -180 ~ 180
        if(vehicle_pose_.yaw > M_PI){
            vehicle_pose_.yaw -= 2*M_PI;
        }else if(vehicle_pose_.yaw < -M_PI){
            vehicle_pose_.yaw += 2*M_PI;
        }
    }
    ProcessINI();
}

void PurePursuitController::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "min_lookahead",
                                    pure_pursuit_params_.min_lookAhead_distance);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector_1",
                                    pure_pursuit_params_.velocity_sector_1);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector_2",
                                    pure_pursuit_params_.velocity_sector_2);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector_3",
                                    pure_pursuit_params_.velocity_sector_3);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector_4",
                                    pure_pursuit_params_.velocity_sector_4);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector1_ld",
                                    pure_pursuit_params_.velocity_sector1_ld);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector2_ld",
                                    pure_pursuit_params_.velocity_sector2_ld);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector3_ld",
                                    pure_pursuit_params_.velocity_sector3_ld);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "velocity_sector4_ld",
                                    pure_pursuit_params_.velocity_sector4_ld);                                                                                    
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "curvature_gain_ld",
                                    pure_pursuit_params_.curvature_gain_ld);
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "pure_pursuit_cte",
                                    pure_pursuit_params_.pure_pursuit_cte);                                                        
        v_ini_parser_.ParseConfig("lateral pure_pursuit", "pure_pursuit_den",
                                    pure_pursuit_params_.pure_pursuit_den);  
        v_ini_parser_.ParseConfig("lateral", "wheel_base",
                                    pure_pursuit_params_.wheel_base);
        v_ini_parser_.ParseConfig("lateral", "a3",
                                    pure_pursuit_params_.a3);
        v_ini_parser_.ParseConfig("lateral", "a2",
                                    pure_pursuit_params_.a2);
        v_ini_parser_.ParseConfig("lateral", "a1",
                                    pure_pursuit_params_.a1);
        v_ini_parser_.ParseConfig("lateral", "a0",
                                    pure_pursuit_params_.a0);
        v_ini_parser_.ParseConfig("lateral", "max_steer",
                                    pure_pursuit_params_.max_steer);
        ROS_WARN("[Pure Pursuit Algorithm] Ini file is updated!");
    }
}

VehicleControl PurePursuitController::PurePursuit(){
    // Find Lookahead Point
    getLookAheadPoint();
    findCrossTrackError();
    /**
     * @brief Pure Pursuit Algorithm
     * 1. Find Cross Track Error (lookahead triangle)
     * 2. Fine Steer Value for instance radius r
     */
    control_cmd_.steer = 
        atan2(2*pure_pursuit_params_.wheel_base * cross_track_error_,pow(lookahead_point_.distance,2))
        + atan2(pure_pursuit_params_.pure_pursuit_cte * getFrontTrackError(),(vehicle_pose_.speed/3.6) + pure_pursuit_params_.pure_pursuit_den + 1.0);
    
    // Transform Radian Value to Degree
    control_cmd_.steer = -radianToDegree(control_cmd_.steer);

    ROS_ERROR_STREAM("Pure Pursuit Steer : " << control_cmd_.steer);

    if(!isForward_){
        control_cmd_.steer = -control_cmd_.steer;
    }

    return control_cmd_;
}

LookAheadState PurePursuitController::GetLookaheadPoint(){
    return lookahead_point_;
}

LookAheadState PurePursuitController::getClosestPoint(){
    return closest_point_;
}

void PurePursuitController::findClosestPoint(){
    double min_distance = HUGE_VAL;

    // Trajectory is longer than lookahead distance
    for(int i=0;i<best_trajectory_.point.size();i++){

        // Get Euclidian Distance from vehicle pose(Ego Frame. Rear) to each trajectory point
        double temp_distance = 
                sqrt(pow(vehicle_pose_.x - best_trajectory_.point[i].x,2) +
                    pow(vehicle_pose_.y - best_trajectory_.point[i].y,2));
        
        // Find Lookahead Point that have distance over lookahead distance and min
        if(temp_distance < min_distance){
            closest_point_.x = best_trajectory_.point[i].x;
            closest_point_.y = best_trajectory_.point[i].y;
            closest_point_.target_yaw = best_trajectory_.point[i].yaw;
            closest_point_.distance = temp_distance; // Distance between Lookahead and Vehicle Pose
            min_distance = temp_distance;
        }              
    }
}

void PurePursuitController::getLookAheadPoint(){
    // Find Closest Point and Save to Global Variable
    findClosestPoint();
    
    // Set Lookahead Distance from INI
    double lookAhead_distance = setLookAheadDistance();

    // Set Min Distance to find min distance point in points over lookahead distance
    double min_distance = HUGE_VAL;
    double max_distance = -HUGE_VAL;

    // Calculate distance for last trajectory point for select process that get lookahead point
    double last_trajectory_point_distance = 
                    sqrt(pow(vehicle_pose_.x - best_trajectory_.point.back().x,2) +
                     pow(vehicle_pose_.y - best_trajectory_.point.back().y,2));
    if(isForward_){
        // Select process for short path that shorter than lookahead distance
        if(loop_count_ == 0){
            if(last_trajectory_point_distance > lookAhead_distance){
            
                // Trajectory is longer than lookahead distance
                for(int i=0;i<best_trajectory_.point.size();i++){

                    // Get Euclidian Distance from vehicle pose(Ego Frame. Rear) to each trajectory point
                    double temp_distance = 
                            sqrt(pow(vehicle_pose_.x - best_trajectory_.point[i].x,2) +
                                pow(vehicle_pose_.y - best_trajectory_.point[i].y,2));

                    // Check if closest point is forward than ego frame.
                    geometry_msgs::PointStamped trajectory_world;
                    trajectory_world.header.frame_id = "/world";
                    trajectory_world.point.x = best_trajectory_.point[i].x;
                    trajectory_world.point.y = best_trajectory_.point[i].y;
                    trajectory_world.point.z = 0.;
                    geometry_msgs::PointStamped trajectory_ego_frame;
                    try{
                        tfListenr.transformPoint("/ego_frame",trajectory_world,trajectory_ego_frame);
                    }catch (tf::TransformException &ex) {
                        // ROS_ERROR(ex.what());
                    }

                    // Find Lookahead Point that have distance over lookahead distance and min
                    if(temp_distance >= lookAhead_distance && temp_distance < min_distance && trajectory_ego_frame.point.x > 0){
                        lookahead_point_.x = best_trajectory_.point[i].x;
                        lookahead_point_.y = best_trajectory_.point[i].y;
                        lookahead_point_.target_yaw = best_trajectory_.point[i].yaw;
                        lookahead_point_.distance = temp_distance; // Distance between Lookahead and Vehicle Pose
                        min_distance = temp_distance;
                    }              
                }
            }else{

                // Trajectory is shorter than lookahead distance
                for(int i=0;i<best_trajectory_.point.size();i++){

                    // Get Euclidian Distance from vehicle pose(Ego Frame. Rear) to each trajectory point
                    double temp_distance = 
                            sqrt(pow(vehicle_pose_.x - best_trajectory_.point[i].x,2) +
                                pow(vehicle_pose_.y - best_trajectory_.point[i].y,2));
                    
                    // Check if closest point is forward than ego frame.
                    geometry_msgs::PointStamped trajectory_world;
                    trajectory_world.header.frame_id = "/world";
                    trajectory_world.point.x = best_trajectory_.point[i].x;
                    trajectory_world.point.y = best_trajectory_.point[i].y;
                    trajectory_world.point.z = 0.;
                    geometry_msgs::PointStamped trajectory_ego_frame;
                    try{
                        tfListenr.transformPoint("/ego_frame",trajectory_world,trajectory_ego_frame);
                    }catch (tf::TransformException &ex) {
                        // ROS_ERROR(ex.what());
                    }

                    // Find Lookahead Point that have distance over lookahead distance and min
                    if(temp_distance <= lookAhead_distance && temp_distance > max_distance && trajectory_ego_frame.point.x > 0){
                        lookahead_point_.x = best_trajectory_.point[i].x;
                        lookahead_point_.y = best_trajectory_.point[i].y;
                        lookahead_point_.target_yaw = best_trajectory_.point[i].yaw;
                        lookahead_point_.distance = temp_distance; // Distance between Lookahead and Vehicle Pose
                        max_distance = temp_distance;
                    }              
                }
            }
        }else{
            for(int i=0;i<best_trajectory_.point.size();i++){
            
                if(best_trajectory_.point[i].distance >= lookAhead_distance){
                    
                    // Trajectory is already sorted by distance, so when we get point, Break for loop.
                    lookahead_point_.x = best_trajectory_.point[i].x;
                    lookahead_point_.y = best_trajectory_.point[i].y;
                    lookahead_point_.target_yaw = best_trajectory_.point[i].yaw;
                    double temp_distance = 
                            sqrt(pow(vehicle_pose_.x - best_trajectory_.point[i].x,2) +
                                pow(vehicle_pose_.y - best_trajectory_.point[i].y,2));
                    lookahead_point_.distance = temp_distance;
                    break;
                }
                else{
                    // If Path's distance is below lookahead distance, select most far point.
                    lookahead_point_.x = best_trajectory_.point.back().x;
                    lookahead_point_.y = best_trajectory_.point.back().y;
                    lookahead_point_.target_yaw = best_trajectory_.point.back().yaw;            
                    double temp_distance = 
                            sqrt(pow(vehicle_pose_.x - best_trajectory_.point[i].x,2) +
                                pow(vehicle_pose_.y - best_trajectory_.point[i].y,2));
                    lookahead_point_.distance = temp_distance;
                }
            }   
        }
        
    }else{
        
        // In Backward, Change ref point to Ego Front
        // Not used in Track Algorithm
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
        ego_front_world_.x = ego_front_zero_world.point.x;
        ego_front_world_.y = ego_front_zero_world.point.y;
        // Trajectory is shorter than lookahead distance
        for(int i=0;i<best_trajectory_.point.size();i++){
            double temp_distance = 
                        sqrt(pow(ego_front_zero_world.point.x - best_trajectory_.point[i].x,2) +
                            pow(ego_front_zero_world.point.y - best_trajectory_.point[i].y,2));
            if(best_trajectory_.point[i].distance >= pure_pursuit_params_.velocity_sector1_ld){
                
                //Trajectory is already sorted by distance, so when we get point, Break for loop.
                lookahead_point_.x = best_trajectory_.point[i].x;
                lookahead_point_.y = best_trajectory_.point[i].y;
                lookahead_point_.target_yaw = best_trajectory_.point[i].yaw;
                lookahead_point_.distance = temp_distance;
                break;
            }
            else{
                lookahead_point_.x = best_trajectory_.point.back().x;
                lookahead_point_.y = best_trajectory_.point.back().y;
                lookahead_point_.target_yaw = best_trajectory_.point.back().yaw;
                lookahead_point_.distance = temp_distance;            
            } 
        }
    }
}

double PurePursuitController::setLookAheadDistance(){
    
    // Return Value
    double lookAhead_distance;
    
    // Set Lookahead Distance by dicretizing velocity sector
    if(vehicle_pose_.speed < pure_pursuit_params_.velocity_sector_1){
        // Sector 1 , Y = ((ld1 - min)/v1)*speed + min
        lookAhead_distance = 
            ((pure_pursuit_params_.velocity_sector1_ld - pure_pursuit_params_.min_lookAhead_distance)/pure_pursuit_params_.velocity_sector_1) * vehicle_pose_.speed 
            + pure_pursuit_params_.min_lookAhead_distance;
    }else if(vehicle_pose_.speed >= pure_pursuit_params_.velocity_sector_1 && vehicle_pose_.speed < pure_pursuit_params_.velocity_sector_2){
        // Sector 2, Y = ((ld2 - ld1)/(v2 - v1))*(speed - v1) + ld1
        lookAhead_distance = 
            ((pure_pursuit_params_.velocity_sector2_ld - pure_pursuit_params_.velocity_sector1_ld)/(pure_pursuit_params_.velocity_sector_2 - pure_pursuit_params_.velocity_sector_1))*(vehicle_pose_.speed - pure_pursuit_params_.velocity_sector_1) 
            + pure_pursuit_params_.velocity_sector1_ld;
    }else if(vehicle_pose_.speed >= pure_pursuit_params_.velocity_sector_2 && vehicle_pose_.speed < pure_pursuit_params_.velocity_sector_3){
        // Sector 3, Y = ((ld3 - ld2)/(v3 - v2))*(speed - v2) + ld2
        lookAhead_distance = 
            ((pure_pursuit_params_.velocity_sector3_ld - pure_pursuit_params_.velocity_sector2_ld)/(pure_pursuit_params_.velocity_sector_3 - pure_pursuit_params_.velocity_sector_2))*(vehicle_pose_.speed - pure_pursuit_params_.velocity_sector_2) 
            + pure_pursuit_params_.velocity_sector2_ld;        
    }else if(vehicle_pose_.speed >= pure_pursuit_params_.velocity_sector_3 && vehicle_pose_.speed < pure_pursuit_params_.velocity_sector_4){
        // Sector 4, Y = ((ld4 - ld3)/(v4 - v3))*(speed - v3) + ld3
        lookAhead_distance = 
            ((pure_pursuit_params_.velocity_sector4_ld - pure_pursuit_params_.velocity_sector3_ld)/(pure_pursuit_params_.velocity_sector_4 - pure_pursuit_params_.velocity_sector_3))*(vehicle_pose_.speed - pure_pursuit_params_.velocity_sector_3) 
            + pure_pursuit_params_.velocity_sector3_ld;            
    }else{
        // Sector 4, Y = ld4
        lookAhead_distance = pure_pursuit_params_.velocity_sector4_ld;
    }
    
    ROS_WARN_STREAM("Speed : " << vehicle_pose_.speed << "LD : " << lookAhead_distance);
    
    return lookAhead_distance;
}

double PurePursuitController::getFrontTrackError(){
    // Set Min Distance for Get Closest Point with Ego Front
    double min_distance = HUGE_VAL;
    
    // Find Closest Point
    for(int i=0;i<best_trajectory_.point.size();i++){
        
        if(best_trajectory_.point[i].distance >= pure_pursuit_params_.wheel_base){
            
            // Trajectory is already sorted by distance, so when we get point, Break for loop.
            front_closest_.x = best_trajectory_.point[i].x;
            front_closest_.y = best_trajectory_.point[i].y;
            front_closest_.target_yaw = best_trajectory_.point[i].yaw;

            break;
        }
        else{
            // If Path's distance is below lookahead distance, select most far point.
            front_closest_.x = best_trajectory_.point.back().x;
            front_closest_.y = best_trajectory_.point.back().y;
            front_closest_.target_yaw = best_trajectory_.point.back().yaw;            
        }
    }
    
    double cross_track_error;
    // Transform Closest Point (World) to Ego to find lateral deviation
    geometry_msgs::PointStamped front_closest_world;
    front_closest_world.header.frame_id = "/world";
    front_closest_world.point.x = front_closest_.x;
    front_closest_world.point.y = front_closest_.y;
    front_closest_world.point.z = 0.;
    geometry_msgs::PointStamped front_closest_ego;
    try{
    tfListenr.transformPoint("/ego_front",front_closest_world,front_closest_ego);
    }catch (tf::TransformException &ex) {
        // ROS_ERROR(ex.what());
    }

    // If Backward(180 degree rotation), error's direction is upside down.
    if(isForward_){
        cross_track_error = front_closest_ego.point.y;
    }else{
        cross_track_error = -front_closest_ego.point.y;
    }

    return cross_track_error;
}

void PurePursuitController::findCrossTrackError(){
    geometry_msgs::PointStamped lookahead_point_world;
    lookahead_point_world.header.frame_id = "/world";
    lookahead_point_world.point.x = lookahead_point_.x;
    lookahead_point_world.point.y = lookahead_point_.y;
    lookahead_point_world.point.z = 0.0;
    geometry_msgs::PointStamped lookahead_point_ego;
    try{
        tfListenr.transformPoint("/ego_frame",lookahead_point_world,lookahead_point_ego);
    }catch (tf::TransformException &ex) {
            // ROS_ERROR(ex.what());
    }
    double cross_track_error = lookahead_point_ego.point.y;
    // cross_track_error_ = cross_track_error;
    if(isForward_){
        cross_track_error_ = cross_track_error;
    }else{
        cross_track_error_ = -cross_track_error;
    }
}

double PurePursuitController::getCrossTrackError(){
    return cross_track_error_;
}

double PurePursuitController::getRearTrackError(){
    return rear_track_error_;
}

double PurePursuitController::getYawError(){
    double yaw_error;
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
    tfListenr.transformPose("/ego_frame",closest_pose_world,closest_pose_ego);
    }catch (tf::TransformException &ex) {
        // ROS_ERROR(ex.what());
    }
    tf::Quaternion q_ego(
        closest_pose_ego.pose.orientation.x,
        closest_pose_ego.pose.orientation.y,
        closest_pose_ego.pose.orientation.z,
        closest_pose_ego.pose.orientation.w);
    q_ego.normalize();
    tf::Matrix3x3 m(q_ego);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_error = yaw;
    if(yaw_error > M_PI){
        yaw_error -= 2*M_PI;
    }else if(yaw_error < -M_PI){
        yaw_error += 2*M_PI;
    }
    yaw_error = radianToDegree(yaw_error);
    rear_track_error_ = closest_pose_ego.pose.position.y;
    return  yaw_error;
}

double PurePursuitController::degreeToRadian(double degree){
    return degree * M_PI / 180;
}

double PurePursuitController::radianToDegree(double radian){
    return radian * 180 / M_PI;
}

double PurePursuitController::trueToPlatformCommand(double steer){
    
    // Calculate True2Command Value
    double command_steer = pure_pursuit_params_.a3*pow(steer,3) + pure_pursuit_params_.a2*pow(steer,2) 
            + pure_pursuit_params_.a1*steer + pure_pursuit_params_.a0;

    return command_steer;
}