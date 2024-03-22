#include "longitudinal_control.hpp"

LongitudinalControl::LongitudinalControl() {}

LongitudinalControl::~LongitudinalControl(){}

void LongitudinalControl::Init(){
    // Node initialization
    NodeHandle nh;
    NodeHandle nh_p("~");
    
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());
    
    // Variable init
    control_command_output_.speed = 1.0;
    control_command_output_.gear = 2;
    control_command_output_.brake = 0;
    control_command_arduino_output_.data = 0.0;

    // Subscriber init
    local_trajectory_subscriber_ = nh.subscribe("/trajectory",10, &LongitudinalControl::getLocalTrajectory,this);
    vehicle_local_state_subscriber_ = nh.subscribe("/vehicle_state", 10, &LongitudinalControl::getVehicleLocalState, this);

    // Publisher init
    control_command_publisher_ = nh.advertise<autoku_msgs::VehicleLongitudinalControl>("/communication/lon_control_cmd",1000);
    control_command_arduino_publisher_ = nh.advertise<std_msgs::Float32>("/arduino/lon_control_cmd",1000);
    visulize_command_publisher_ = nh.advertise<std_msgs::Float32>("/hmi/control/current_speed",10);
    visulize_target_publisher_ = nh.advertise<std_msgs::Float32>("/hmi/control/target_speed",10);
    velocity_error_publisher_ = nh.advertise<std_msgs::Float32>("/hmi/control/velocity_error",10);
    system_time_publisher_ = nh.advertise<std_msgs::Float64>("sys/longitudinal_control", 1);

    nh_p.getParam("mode",mode_);
    ROS_FATAL(" MODE = %s",mode_.c_str());
    // Algorithm init
    pid_controller_algorithm_ = make_unique<PIDController>();
    pid_controller_algorithm_->Init();
    ProcessINI();
}

void LongitudinalControl::Run(){
    ROS_INFO("Longitudinal Control Running ...\n");
    ProcessINI();
    // Get functions for subscribe variables
    TargetState targetstate =  GetTargetState();
    VehicleState vehiclestate = GetVehicleState();
    double velocity_error;
    // Update variables
    // Algorithm
    VehicleControl command;
    if(control_params_.control_test_mode == 0 || control_params_.control_test_mode == 1){
        pid_controller_algorithm_->Update(&vehiclestate,&targetstate, &best_trajectory_);
        command = pid_controller_algorithm_->PIDControl();
        velocity_error = pid_controller_algorithm_->getVelocityError();
    }
	// Update output
    UpdateControlCommand(command);   
    UpdateVeloccityError(velocity_error);
    o_system_time_.data = ros::Time::now().toSec();
}

void LongitudinalControl::Publish(){
    if(mode_ == "sim") {
        control_command_publisher_.publish(control_command_output_);
    }
    else {
        control_command_arduino_publisher_.publish(control_command_arduino_output_);
    }
    visulize_command_publisher_.publish(visualize_command_output_);
    visulize_target_publisher_.publish(visualize_target_output_);
    velocity_error_publisher_.publish(velocity_error_output_);
    system_time_publisher_.publish(o_system_time_);
}

void LongitudinalControl::Terminate(){}

void LongitudinalControl::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("test_mode", "control_test",
                                    control_params_.control_test_mode);
        ROS_WARN("[Longitudinal Control] Ini file is updated!\n");
        
    }
}

// Callback functions

void LongitudinalControl::getLocalTrajectory(const autoku_msgs::Trajectory::ConstPtr& msg) {
    mutex_trajectory_.lock();
    best_trajectory_ = *msg;
    // ROS_INFO("Subscribe Vehicle State");
    mutex_trajectory_.unlock();
}

void LongitudinalControl::getVehicleLocalState(const autoku_msgs::VehicleState::ConstPtr& msg){
    mutex_state_.lock();
    vehicle_local_state_ = *msg;
    // ROS_INFO("Subscribe Vehicle State");
    mutex_state_.unlock();
}

// Get functions

TargetState LongitudinalControl::GetTargetState() {
    TargetState target_state;
    mutex_target_state_.lock();
    if(best_trajectory_.point.size()>0){
        
        if(best_trajectory_.point.size()<2){
            target_state.target_speed = best_trajectory_.point[0].speed; 
        }
        else if(best_trajectory_.point.size()<3){ 
            target_state.target_speed = best_trajectory_.point[1].speed; 
        }
        else if(best_trajectory_.point.size()<4){
            target_state.target_speed = best_trajectory_.point[2].speed; 
        }
        else if(best_trajectory_.point.size()<5){
            target_state.target_speed = best_trajectory_.point[3].speed; 
        }
        else if(best_trajectory_.point.size()<6){
            target_state.target_speed = best_trajectory_.point[4].speed; 
        }
        else if(best_trajectory_.point.size()<7){
            target_state.target_speed = best_trajectory_.point[5].speed; 
        }
        else if(best_trajectory_.point.size()<8){
            target_state.target_speed = best_trajectory_.point[6].speed; 
        }
        else{
            target_state.target_speed = best_trajectory_.point[7].speed;
        }
        visualize_target_output_.data = target_state.target_speed;
    }
    mutex_target_state_.unlock();

    return target_state;
}

VehicleState LongitudinalControl::GetVehicleState() {
    VehicleState vehicle_state;
    mutex_state_.lock();
    vehicle_state.speed_kmh = (double)vehicle_local_state_.speed * 3.6; 
    mutex_state_.unlock();

    return vehicle_state;
}

void LongitudinalControl::UpdateControlCommand(const VehicleControl& command_update){
    TargetState targetstate =  GetTargetState();
    if(mode_ == "sim") {
        if(targetstate.target_speed <0){
            control_command_output_.gear = 0;
        }else{
            control_command_output_.gear = 2;
        }
        control_command_output_.speed = command_update.speed_kmh;
        control_command_output_.brake = command_update.brake;

        if(control_params_.control_test_mode == 0){
            ROS_INFO("----------------COMPETITION MODE : PID CONTROLLER----------------");
        }else if(control_params_.control_test_mode == 1){
            ROS_INFO("----------------TEST MODE : PID CONTROLLER---------------- \n");
        }
        ROS_INFO("Command Speed : %.3f [km/h]",control_command_output_.speed);
        ROS_INFO("Command Brake : %d ",control_command_output_.brake);
        ROS_INFO("Current Speed : %.3f [km/h]",vehicle_local_state_.speed);
    }
    else {
        // arduino
        if (targetstate.target_speed >= 0.2)
        {
            if (command_update.speed_kmh > 0.000 && command_update.brake < 0.0001)
            {
                control_command_arduino_output_.data = 200.0 + command_update.speed_kmh;
            }
            else if (command_update.speed_kmh < 0.0001 && command_update.brake > 0.000)
            {
                control_command_arduino_output_.data = command_update.brake;
            }
        }
        else if (targetstate.target_speed <= -0.2)
        {
            if (fabs(command_update.speed_kmh) > 0.0001 && command_update.brake < 0.0001)
            {
                control_command_arduino_output_.data = -200 -command_update.speed_kmh;
            }
            else if (fabs(command_update.speed_kmh) < 0.0001 && command_update.brake > 0.000)
            {
                control_command_arduino_output_.data = -command_update.brake;
            }
        }
        else {
            control_command_arduino_output_.data = 199.0;
        }

        ROS_INFO("command_update.speed_kmh : %.3f, command_update.brake : %.3f", command_update.speed_kmh, command_update.brake);
    }
    visualize_command_output_.data = (double)vehicle_local_state_.speed*3.6;
}

void LongitudinalControl::UpdateVeloccityError(double current_velocity_error){
    velocity_error_output_.data = current_velocity_error;
}

int main(int argc, char **argv){
    std::string node_name = "longitudinal_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize [Longitudinal Control]node, get parameters...");
    
    LongitudinalControl longitudinal_control;
    longitudinal_control.Init();
    ros::Rate loop_rate(100);
    while(ros::ok()){
        longitudinal_control.Run();
        longitudinal_control.Publish();

        ros::spinOnce();
        loop_rate.sleep();
    }
    longitudinal_control.Terminate();
    return 0;
}