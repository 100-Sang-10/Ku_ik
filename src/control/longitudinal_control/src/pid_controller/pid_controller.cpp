#include "pid_controller/pid_controller.hpp"

PIDController::PIDController() {
    
}

PIDController::~PIDController(){

}

bool PIDController::Init(){
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/control.ini");
    v_ini_parser_.Init((dir+ini_dir).c_str());

    // Parameters initialization
	ProcessINI();

    return true;
}
void PIDController::Update(VehicleState* vehicle_state,TargetState* target_state,autoku_msgs::Trajectory* best_trajectory){	
    vehicle_state_ = *vehicle_state;
    target_state_ = *target_state;
    best_trajectory_ = *best_trajectory;
	ProcessINI();
}

void PIDController::ProcessINI(){
    if (v_ini_parser_.IsFileUpdated()){
        v_ini_parser_.ParseConfig("longitudinal", "p_gain",
                                    pid_gains_.p_gain);
        v_ini_parser_.ParseConfig("longitudinal", "i_gain",
                                    pid_gains_.i_gain);
        v_ini_parser_.ParseConfig("longitudinal", "d_gain",
                                    pid_gains_.d_gain);
        v_ini_parser_.ParseConfig("longitudinal", "p_gain_brake",
                                    pid_gains_.p_gain_brake);
        v_ini_parser_.ParseConfig("longitudinal", "i_gain_brake",
                                    pid_gains_.i_gain_brake);
        v_ini_parser_.ParseConfig("longitudinal", "d_gain_brake",
                                    pid_gains_.d_gain_brake);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector_1",
                                    pid_gains_.brake_sector_1);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector_2",
                                    pid_gains_.brake_sector_2);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector_3",
                                    pid_gains_.brake_sector_3);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector_4",
                                    pid_gains_.brake_sector_4);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector1_value",
                                    pid_gains_.brake_sector1_value);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector2_value",
                                    pid_gains_.brake_sector2_value);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector3_value",
                                    pid_gains_.brake_sector3_value);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector4_value",
                                    pid_gains_.brake_sector4_value);
        v_ini_parser_.ParseConfig("longitudinal", "brake_sector5_value",
                                    pid_gains_.brake_sector5_value);
        
        ROS_WARN("[PID Controller] Ini file is updated!");
    }
}

VehicleControl PIDController::PIDControl(){
    if(target_state_.target_speed < 0){
        control_cmd_.gear = 0;
    }else{
        control_cmd_.gear = 2;
    }
    double dt_ = ros::Time::now().toSec() - prevRosTime_;
    if(dt_ <= 0){
        dt_ = 0.000001;
    }
    prevRosTime_ = ros::Time::now().toSec();
    // ROS_INFO("Target Speed : %f \n Vehicle Speed :%f", target_state_.target_speed,vehicle_state_.speed_kmh);
    double velocity_error = abs(target_state_.target_speed) - vehicle_state_.speed_kmh;
    velocity_error_ = velocity_error;
    double derror = (velocity_error + prevError_) * dt_ / 2; 

    if (velocity_error * prevError_ < 0.0) {
        integral_error_ = 0.0;
    }
    else {
        integral_error_ += derror;
    }

    // ROS_WARN("Error Integrate : %f", integral_error_);
    // ROS_INFO("Integral Error : %f", integral_error_);
    double derivative_error = (velocity_error - prevError_) / dt_ ;
    if(velocity_error > 0){
        control_cmd_.brake = 0;
        if (velocity_error > 2.0)
        {
            control_cmd_.speed_kmh = 0.3 * pid_gains_.p_gain * velocity_error +
                                     0.3 * pid_gains_.i_gain * integral_error_ +
                                     0.3 * pid_gains_.d_gain * derivative_error;
        }
        else
        {
            control_cmd_.speed_kmh = pid_gains_.p_gain * velocity_error +
                                     pid_gains_.i_gain * integral_error_ +
                                     pid_gains_.d_gain * derivative_error;
        }
    }
    else if(velocity_error <= 0){
        control_cmd_.speed_kmh = 0;
        if (velocity_error < 0 && velocity_error >= -1 && vehicle_state_.speed_kmh > 1)
        {
            control_cmd_.brake = 0.0;
        }
        // else {
        //     control_cmd_.brake = -(pid_gains_.p_gain_brake * velocity_error +
        //                         pid_gains_.i_gain_brake * integral_error_ +
        //                         pid_gains_.d_gain_brake * derivative_error);
        // }

        else
        {
            if (velocity_error < 0 && velocity_error >= -1 && vehicle_state_.speed_kmh > 1)
            {
                control_cmd_.brake = 10;
            }
            else if (velocity_error < 0 && velocity_error >= -1 && vehicle_state_.speed_kmh < 1)
            {
                control_cmd_.brake = 5;
            }
            else if (velocity_error < -1 && velocity_error >= -3)
            {
                control_cmd_.brake = (int)16 * pid_gains_.p_gain_brake;
            }
            else if (velocity_error < -3 && velocity_error >= -5)
            {
                control_cmd_.brake = (int)20 * pid_gains_.p_gain_brake;
            }
            else if (velocity_error < -5 && velocity_error >= -10)
            {
                control_cmd_.brake = (int)30 * pid_gains_.p_gain_brake;
            }
            else if (velocity_error < -10)
            {
                control_cmd_.brake = (int)40 * pid_gains_.p_gain_brake;
            }
        }
    }
    
    if (fabs(target_state_.target_speed) <= 0.01)
    {
        control_cmd_.speed_kmh = 0;
        control_cmd_.brake = 150;
    }
    
    prevError_ = velocity_error;
    
    return control_cmd_;
}

double PIDController::getVelocityError(){
    return velocity_error_;
}