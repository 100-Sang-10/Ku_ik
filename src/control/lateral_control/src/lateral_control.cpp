#include "lateral_control.hpp"

LateralControl::LateralControl(){}

LateralControl::~LateralControl(){}

void LateralControl::Init(){
    // Node initialization
    NodeHandle nh;
    NodeHandle nh_p("~");

    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    // Variable init
    control_command_output_.steer = 0;
    control_command_arduino_output_.data = 0.0;
    max_track_error_.data = 0.;
    loop_count_.data = 0;
    // Subscriber init
    vehicle_local_state_subscriber_ = nh.subscribe("/vehicle_state",10, &LateralControl::getVehicleLocalState,this);
    local_trajectory_subscriber_ = nh.subscribe("/trajectory",10,&LateralControl::getLocalTrajectory,this);
    loop_count_subscriber_ = nh.subscribe("loop_count",10,&LateralControl::getLoopCount,this);
    // Publisher init
    control_command_publisher_ = nh.advertise<autoku_msgs::VehicleLateralControl>("/communication/lat_control_cmd",1000);
    control_command_arduino_publisher_ = nh.advertise<std_msgs::Float32>("/arduino/lat_control_cmd",1000);
    lookahead_point_publisher_ = nh.advertise<visualization_msgs::Marker>("/hmi/control/lookahead_point",10);
    closest_point_publisher_ = nh.advertise<visualization_msgs::Marker>("/hmi/contorl/closest_point",10);
    cross_track_error_publisher_ = nh.advertise<std_msgs::Float32>("/hmi/contorl/cross_track_error",10);
    yaw_error_publisher_ = nh.advertise<std_msgs::Float32>("/hmi/contorl/yaw_error",10);
    max_track_error_publisher_ = nh.advertise<std_msgs::Float32>("/hmi/contorl/max_track_error",10);
    system_time_publisher_ = nh.advertise<std_msgs::Float64>("sys/lateral_control", 1);
    traj_front_circle_publisher_ = nh.advertise<visualization_msgs::Marker>("/hmi/control/front_dist_circle",10);


    nh_p.getParam("mode",mode_);
    ROS_FATAL(" MODE = %s",mode_.c_str());
    // Algorithm init
    pure_pursuit_algorithm_ = make_unique<PurePursuitController>();
    pure_pursuit_algorithm_->Init();
    pure_pursuit_time_delay_algorithm_ = make_unique<PurePursuitTimeDelay>();
    pure_pursuit_time_delay_algorithm_->Init();
    stanley_method_ = make_unique<StanleyController>();
    stanley_method_->Init();
    ProcessINI();
}

void LateralControl::Run(){	
    ROS_INFO("Lateral Control Running ...\n");
	ProcessINI();
    
    // Get functions for subscribe variables
	VehicleLocalState vehicle_pose = GetVehicleLocalState();
    LookAheadState lookahead_point;
    LookAheadState closest_point;
    VehicleLocalState time_delayed_pose;
    double traj_front_dist;

    // Algorithm
    if(best_trajectory_.point.size()>0){

        double target_velocity = best_trajectory_.point.front().speed;
        traj_front_dist = sqrt(pow(best_trajectory_.point.front().x - vehicle_pose.x,2) + pow(best_trajectory_.point.front().y - vehicle_pose.y,2));
        ROS_INFO("TRAJ FRONT DISTANCE : %f", traj_front_dist);

        if(control_params_.pure_pursuit_time_delay) {
            if ((vehicle_pose.speed > control_params_.pure_stanley_toggle_vel)
            ||  (traj_front_dist >= control_params_.pure_pursuit_toggle_boundary)) {

                pure_pursuit_time_delay_algorithm_->Update(&vehicle_pose, &best_trajectory_, &loop_count_.data);
                pure_pursuit_command = pure_pursuit_time_delay_algorithm_->PurePursuit();
                if (mode_ == "real")
                {
                    pure_pursuit_command.steer = pure_pursuit_time_delay_algorithm_->trueToPlatformCommand(pure_pursuit_command.steer);
                }
                lookahead_point = pure_pursuit_time_delay_algorithm_->GetLookaheadPoint();
                closest_point = pure_pursuit_time_delay_algorithm_->getClosestPoint();
                time_delayed_pose = pure_pursuit_time_delay_algorithm_->getPredictedPose();
                yaw_error_.data = pure_pursuit_time_delay_algorithm_->getYawError();
                cross_track_error_.data = pure_pursuit_time_delay_algorithm_->getRearTrackError();
                ROS_INFO("PURE PURSUIT TIME DELAY METHOD");
                UpdateControlCommand(pure_pursuit_command);
            }
            else {
                stanley_method_->Update(&vehicle_pose, &best_trajectory_,&loop_count_.data);
                stanley_command = stanley_method_->Stanley();
                if(mode_ == "real") {
                    stanley_command.steer = stanley_method_->trueToPlatformCommand(stanley_command.steer);
                }
                closest_point = stanley_method_->getClosestPoint();
                cross_track_error_.data = stanley_method_->getCrossTrackError();
                yaw_error_.data = stanley_method_->getYawError()*180/M_PI;
                ROS_INFO("STANLEY METHOD");
                UpdateControlCommand(stanley_command);
            }
        }
        else {
            if((vehicle_pose.speed > control_params_.pure_stanley_toggle_vel) 
            // || (control_params_.backward_method == 0 && target_velocity <= 0)
            || (traj_front_dist >= control_params_.pure_pursuit_toggle_boundary)) { // || target_velocity <0

                pure_pursuit_algorithm_->Update(&vehicle_pose,&best_trajectory_,&loop_count_.data);
                pure_pursuit_command = pure_pursuit_algorithm_->PurePursuit();
                if(mode_ == "real"){
                    pure_pursuit_command.steer = pure_pursuit_algorithm_->trueToPlatformCommand(pure_pursuit_command.steer);
                }
                lookahead_point = pure_pursuit_algorithm_->GetLookaheadPoint();
                closest_point = pure_pursuit_algorithm_->getClosestPoint();
                yaw_error_.data = pure_pursuit_algorithm_->getYawError();
                cross_track_error_.data = pure_pursuit_algorithm_->getRearTrackError();
                ROS_INFO("PURE PURSUIT METHOD");
                UpdateControlCommand(pure_pursuit_command);
            }
            else {
                stanley_method_->Update(&vehicle_pose, &best_trajectory_,&loop_count_.data);
                stanley_command = stanley_method_->Stanley();
                if(mode_ == "real") {
                    stanley_command.steer = stanley_method_->trueToPlatformCommand(stanley_command.steer);
                }
                closest_point = stanley_method_->getClosestPoint();
                cross_track_error_.data = stanley_method_->getCrossTrackError();
                yaw_error_.data = stanley_method_->getYawError()*180/M_PI;
                ROS_INFO("STANLEY METHOD");
                UpdateControlCommand(stanley_command);
            }
        }
    }
    else{
        ROS_WARN("Wait For Trajectory  ...");
    }        
    UpdateLookaheadPoint(lookahead_point,closest_point, time_delayed_pose);
    UpdateMaxtrackError(cross_track_error_.data);
    UpdateFrontDistCircle(traj_front_dist);
    o_system_time_.data = ros::Time::now().toSec();
    
}

void LateralControl::Publish(){
    if(mode_ == "sim") {
        control_command_publisher_.publish(control_command_output_);
    }
    else {
        control_command_arduino_publisher_.publish(control_command_arduino_output_);
    }
    
    lookahead_point_publisher_.publish(lookahead_point_);
    closest_point_publisher_.publish(closest_point_);
    cross_track_error_publisher_.publish(cross_track_error_);
    yaw_error_publisher_.publish(yaw_error_);
    max_track_error_publisher_.publish(max_track_error_);
    interpolating_path_publisher_.publish(interpolating_path_marker_);
    system_time_publisher_.publish(o_system_time_);
}

void LateralControl::Terminate(){}

void LateralControl::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("lateral", "pure_stanley_toggle_vel",
                                    control_params_.pure_stanley_toggle_vel);
        v_ini_parser_.ParseConfig("lateral", "max_steer",
                                    control_params_.max_steer);                                                        
        v_ini_parser_.ParseConfig("lateral", "backward_method",
                                    control_params_.backward_method);
        v_ini_parser_.ParseConfig("lateral", "is_pure_pursuit_delay_considered",
                                    control_params_.pure_pursuit_time_delay);
        v_ini_parser_.ParseConfig("lateral", "pure_pursuit_toggle_boundary",
                                    control_params_.pure_pursuit_toggle_boundary);
        ROS_WARN("[Lateral Control] Ini file is updated!\n");
    }
}

double LateralControl::clipMaxSteer(double steer){
    double command_steer;
    
    // Clip Steer By Min, Max Steer
    if(steer > control_params_.max_steer){
        command_steer = control_params_.max_steer;
    }else if(steer < -control_params_.max_steer){
        command_steer = -control_params_.max_steer;
    }else{
        command_steer = steer;
    }

    return command_steer;
}


// Callback functions

void LateralControl::getLoopCount(const std_msgs::Int32::ConstPtr& msg){
    mutex_loop_count_.lock();
    loop_count_ = *msg;
    mutex_loop_count_.unlock();
}

void LateralControl::getVehicleLocalState(const autoku_msgs::VehicleState::ConstPtr& msg){
    mutex_local_state_.lock();
    vehicle_local_state_ = *msg;
    mutex_local_state_.unlock();
}

void LateralControl::getLocalTrajectory(const autoku_msgs::Trajectory::ConstPtr& msg){
    mutex_trajectory_.lock();
    best_trajectory_ = *msg;
    mutex_trajectory_.unlock();
}

void LateralControl::getPlatformState(const autoku_msgs::PlatformState::ConstPtr& msg){
    mutex_platform_state_.lock();
    platform_state_ = *msg;
    currentAuto_ = platform_state_.automode;
    mutex_platform_state_.unlock();
}


// Get functions

VehicleLocalState LateralControl::GetVehicleLocalState(){
    VehicleLocalState vehicle_pose;
    vehicle_pose.x = vehicle_local_state_.x;
    vehicle_pose.y = vehicle_local_state_.y;
    vehicle_pose.yaw = vehicle_local_state_.yaw;
    vehicle_pose.speed = vehicle_local_state_.speed*3.6;

    return vehicle_pose;
}

// Transform functions

void LateralControl::UpdateLookaheadPoint(const LookAheadState lookahead_point,const LookAheadState closest_point, VehicleLocalState vehicle_pose){
    lookahead_point_.header.frame_id = "world";
    lookahead_point_.header.stamp = ros::Time::now();
    lookahead_point_.lifetime = ros::Duration(0.1);
    lookahead_point_.ns = "Lateral Control";
    lookahead_point_.id = 0;
    lookahead_point_.type = visualization_msgs::Marker::SPHERE;
    lookahead_point_.pose.position.x = lookahead_point.x;
    lookahead_point_.pose.position.y = lookahead_point.y;
    lookahead_point_.scale.x = 0.6;
    lookahead_point_.scale.y = 0.6;
    lookahead_point_.scale.z = 0.6;
    lookahead_point_.color.r = 1.0;
    lookahead_point_.color.g = 0.0;
    lookahead_point_.color.b = 0.0;
    lookahead_point_.color.a = 1.0;

    closest_point_.header.frame_id = "world";
    closest_point_.header.stamp = ros::Time::now();
    closest_point_.lifetime = ros::Duration(0.1);
    closest_point_.ns = "Lateral Control";
    closest_point_.id = 1;
    closest_point_.type = visualization_msgs::Marker::SPHERE;
    closest_point_.pose.position.x = closest_point.x;
    closest_point_.pose.position.y = closest_point.y;
    closest_point_.scale.x = 0.4;
    closest_point_.scale.y = 0.4;
    closest_point_.scale.z = 0.4;
    closest_point_.color.r = 0.0;
    closest_point_.color.g = 0.0;
    closest_point_.color.b = 1.0;
    closest_point_.color.a = 1.0;

    predicted_pose_after_time_delay_.header.frame_id = "world";
    predicted_pose_after_time_delay_.header.stamp = ros::Time::now();
    predicted_pose_after_time_delay_.lifetime = ros::Duration(0.1);
    predicted_pose_after_time_delay_.ns = "Lateral Control";
    predicted_pose_after_time_delay_.id = 2;
    predicted_pose_after_time_delay_.type = visualization_msgs::Marker::SPHERE;
    predicted_pose_after_time_delay_.pose.position.x = vehicle_pose.x;
    predicted_pose_after_time_delay_.pose.position.y = vehicle_pose.y;
    predicted_pose_after_time_delay_.scale.x = 1.0;
    predicted_pose_after_time_delay_.scale.y = 1.0;
    predicted_pose_after_time_delay_.scale.z = 1.0;
    predicted_pose_after_time_delay_.color.r = 1.0;
    predicted_pose_after_time_delay_.color.g = 1.0;
    predicted_pose_after_time_delay_.color.b = 0.0;
    predicted_pose_after_time_delay_.color.a = 1.0;
}

void LateralControl::UpdateFrontDistCircle(double traj_front_dist){
    
    front_dist_circle_.header.frame_id = "/ego_frame";
    front_dist_circle_.header.stamp = ros::Time::now();
    front_dist_circle_.lifetime = ros::Duration(0.1);
    front_dist_circle_.ns = "Lateral Control";
    front_dist_circle_.id = 1;
    front_dist_circle_.type = visualization_msgs::Marker::SPHERE;
    front_dist_circle_.pose.position.x = 0.;
    front_dist_circle_.pose.position.y = 0.;
    front_dist_circle_.scale.x = traj_front_dist;
    front_dist_circle_.scale.y = traj_front_dist;
    front_dist_circle_.scale.z = 0.02;
    front_dist_circle_.color.r = 1.0;
    front_dist_circle_.color.g = 1.0;
    front_dist_circle_.color.b = 0.0;
    front_dist_circle_.color.a = 1.0;
}

void LateralControl::UpdateControlCommand(const VehicleControl& command_update){
    
    if(best_trajectory_.point.size()>0){
        if(mode_ == "sim") {
            control_command_output_.steer = clipMaxSteer(command_update.steer);
            ROS_INFO_STREAM("COMMAND STEER : " << control_command_output_.steer);
        }
        else {
            control_command_arduino_output_.data = clipMaxSteer(command_update.steer);
            ROS_INFO_STREAM("COMMAND STEER : " << control_command_arduino_output_.data);
        }
        
        
    }
}

void LateralControl::UpdateMaxtrackError(double current_track_error){
    if(current_track_error > max_track_error_.data){
        max_track_error_.data = current_track_error;
        ROS_INFO("Max Cross Track Error is Update!");
    }
}

int main(int argc, char **argv){
    std::string node_name = "lateral_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize [Lateral Control]node, get parameters...");

    LateralControl lateral_control;
    lateral_control.Init();
    ros::Rate loop_rate(100);
    while(ros::ok()){
        
        lateral_control.Run();
        lateral_control.Publish();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    lateral_control.Terminate();
    return 0;
}