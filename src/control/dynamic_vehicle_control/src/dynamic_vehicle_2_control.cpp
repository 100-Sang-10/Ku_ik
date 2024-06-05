#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <std_msgs/Int64.h>
#include <WGS84toCartesian.hpp>

#define LOOK_AHEAD_DISTANCE 2
#define TARGET_SPEED_KPH  15
#define init_position_x -173.0
#define init_position_y  -84.5

class DynamicVehicleControl2 {
  private:
    ros::NodeHandle nh;

    ros::Publisher waypoint_pub;
    ros::Publisher control_pub;
    ros::Subscriber speed_sub;
    ros::Subscriber odom_sub;
    ros::Publisher purepursuit_point_pub;

    std::array<double, 2> WSG84Reference{0, 0};
    std::vector<std::vector<double>> waypoint_vec;
    double dynamic_vehicle_x = 0.0;
    double dynamic_vehicle_y = 0.0;
    double dynamic_vehicle_yaw = 0.0;
    int purepursuit_waypoint_count = 0;
    double next_point_x = init_position_x;
    double next_point_y = init_position_y;
    double angle, accelerator, stop;
    double purepursuit_current_co_tf_x, purepursuit_current_co_tf_y;
    double Ld, wheelbase, purepursuit_d;
    visualization_msgs::Marker purepursuit_point;

    carla_msgs::CarlaEgoVehicleControl control_msg;
    geometry_msgs::Pose curr_pose;
    float curr_speed;
    
  public:
    DynamicVehicleControl2();
    ~DynamicVehicleControl2();

    void SpeedCallback(const std_msgs::Float32::ConstPtr speed_msg);
    void PoseCallback(const nav_msgs::Odometry::ConstPtr odom_msg);

    std::vector<double> WGS84toCartesian(double input_lat, double input_long);
    void waypoint();
    std::vector<std::string> divide(std::string xy_str, char divider);
    double to_num(std::string s);
    void visualize_waypoint();
    double cal_yaw(const nav_msgs::Odometry::ConstPtr& odom_msg);
    std::pair<double, double> coordinate_tf(double input_x, double input_y);
    void next_point();
    void pure_pursuit();
    void purepursuit_next_point();
    void purepursuit_point_marker();
    void PID();
    void Control();

    void Print();
    void Publish();
    void Run();
};

DynamicVehicleControl2::DynamicVehicleControl2() {
    waypoint_pub = nh.advertise<visualization_msgs::Marker>("/dynamic_vehicle_2/waypoint_marker", 100);
    control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/dynamic_vehicle_2/vehicle_control_cmd", 1000);
    speed_sub = nh.subscribe("/carla/dynamic_vehicle_2/speedometer", 10, &DynamicVehicleControl2::SpeedCallback, this);
    odom_sub = nh.subscribe("/carla/dynamic_vehicle_2/odometry", 10, &DynamicVehicleControl2::PoseCallback, this);
    purepursuit_point_pub = nh.advertise<visualization_msgs::Marker>("/dynamic_vehicle_2/point_marker", 100);
}

DynamicVehicleControl2::~DynamicVehicleControl2() {}

void DynamicVehicleControl2::SpeedCallback(const std_msgs::Float32::ConstPtr speed_msg) {
    curr_speed = speed_msg->data;
}

void DynamicVehicleControl2::PoseCallback(const nav_msgs::Odometry::ConstPtr odom_msg) {
    dynamic_vehicle_x = odom_msg->pose.pose.position.x;
    dynamic_vehicle_y = odom_msg->pose.pose.position.y;
    dynamic_vehicle_yaw = cal_yaw(odom_msg);
}

std::vector<double> DynamicVehicleControl2::WGS84toCartesian(double input_lat, double input_long) {
    std::array<double, 2> WGS84Position {input_lat, input_long};
    std::array<double, 2> cartesian_position {wgs84::toCartesian(WSG84Reference, WGS84Position)};

    std::vector<double> XY;
    XY.push_back(cartesian_position[0]);
    XY.push_back(cartesian_position[1]);
    
    // ROS_INFO("x = %f", cartesian_position[0]);
    // ROS_INFO("y = %f", cartesian_position[1]);
    
    return XY;
}

void DynamicVehicleControl2::waypoint() {
    // std::ifstream in("../Ku_ik/resources/dynamic_vehicle_2_waypoint.txt");
    std::ifstream in("/home/kichang/Ku_ik/src/control/dynamic_vehicle_control/resources/dynamic_vehicle_2_waypoint.txt");
    
    if (!in.is_open()) {
        ROS_ERROR("waypoint file not found!");
    }
    else {
        std::cout << "file reading complete!" << "\n";
    }
    
    std::string lat_long;
    while (in) {
        getline(in, lat_long);  // latitude, longitude data 문자열에 한줄씩 저장
        std::vector<double> ll;
        std::vector<std::string> ll_vec = divide(lat_long, ',');  //한 줄씩 있는 latitude, longitude data를 ,로 x와 y 좌표 나눔
        std::vector<double> map_XY;  // vector include waypoint (x, y)
        
        for (typename std::vector<std::string>::iterator itr = ll_vec.begin(); itr != ll_vec.end(); ++itr) {
            ll.push_back(to_num(*itr));
            
            double input_lat, input_long;
            input_lat = ll[0];
            input_long = ll[1];
            map_XY = WGS84toCartesian(input_lat, input_long);
            // map_XY.push_back(index);
        }
        waypoint_vec.push_back(map_XY);
    }
}

std::vector<std::string> DynamicVehicleControl2::divide(std::string xy_str, char divider){
    std::istringstream s(xy_str);
    std::string buffer;
    std::vector<std::string> result;

    while(getline(s,buffer,divider)){
        result.push_back(buffer);
    }

    return result;
}

double DynamicVehicleControl2::to_num(std::string s){
    std::istringstream ss(s);
    double x;
    ss >> x;

    return x;
}

void DynamicVehicleControl2::visualize_waypoint(){
    std::vector<double> waypoint;
    visualization_msgs::Marker points, line_strip;

    points.header.frame_id = line_strip.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.scale.z = 0.5;
    line_strip.scale.x = 0.3;
    line_strip.scale.y = 0.3;
    line_strip.scale.z = 0.3;

    // Points are green
    points.color.g = 1.0;
    points.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    for(std::vector<std::vector<double>>::iterator itr = waypoint_vec.begin(); itr != waypoint_vec.end(); ++itr) {
        waypoint = *itr;
    
        geometry_msgs::Point p;
        p.x = waypoint[0];
        p.y = waypoint[1];
        p.z = 0.058036078;

        points.points.push_back(p);   
        line_strip.points.push_back(p);
    }
    waypoint_pub.publish(points);
    waypoint_pub.publish(line_strip);
}

double DynamicVehicleControl2::cal_yaw(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    tf::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

std::pair<double, double> DynamicVehicleControl2::coordinate_tf(double input_x, double input_y) {
    double x, y, a, d, output_x, output_y;

    x = input_x - dynamic_vehicle_x;
    y = input_y - dynamic_vehicle_y;
    a = atan2(y, x) - dynamic_vehicle_yaw;
    d = sqrt( pow(x, 2) + pow(y, 2) );
    
    output_x = d * cos(a);
    output_y = d * sin(a);

    return std::make_pair(output_x, output_y);
}

void DynamicVehicleControl2::next_point() {
    // ROS_INFO("next_point start");
    int count = 0;
    
    std::vector<double> waypoint;

    for(std::vector<std::vector<double>>::iterator itr = waypoint_vec.begin(); itr != waypoint_vec.end(); ++itr) {
        // ROS_INFO("count = %i", count);
        // ROS_INFO("purepursuit_waypoint_count = %i", purepursuit_waypoint_count);
        if(count == purepursuit_waypoint_count){
            break;
        }
        // ROS_INFO("waypoint_vec for");

        waypoint = *itr;

        next_point_x = waypoint[0];
        next_point_y = waypoint[1];

        count++;
    }

    purepursuit_waypoint_count++;

    if (purepursuit_waypoint_count == waypoint_vec.size()) {
        purepursuit_waypoint_count = 0;
    }
}

void DynamicVehicleControl2::pure_pursuit(){
    angle = - atan2( 2 * purepursuit_current_co_tf_y * wheelbase, pow(purepursuit_d, 2) ) * (2 / M_PI);

    if (angle >= 1.0) {
        angle = 1.0;
    }
    else if (angle <= -1.0) {
        angle = -1.0;
    }
}

void DynamicVehicleControl2::purepursuit_next_point(){
    wheelbase = 2.900078125019439;     // wheelbase
    double velocity = curr_speed * 3.6;
    // Ld = 0 + (0.25 * velocity);  // TODO:tunnings
    Ld = LOOK_AHEAD_DISTANCE;
    // if (Ld <= 2.0) {
    //     Ld = 2.0;
    // }

    while(1) {
        std::pair<double, double> p = coordinate_tf(next_point_x, next_point_y);
        purepursuit_current_co_tf_x = p.first;
        purepursuit_current_co_tf_y = p.second;
        purepursuit_current_co_tf_x += 1.38193658598696995;                                                 // from rear wheel
        purepursuit_d = sqrt( pow(purepursuit_current_co_tf_x, 2) + pow(purepursuit_current_co_tf_y, 2) );  // target distance from rear wheel
        if (purepursuit_d <= Ld) {
            next_point();
        }
        else {
            break;
        }
    }
}

void DynamicVehicleControl2::purepursuit_point_marker() {
    purepursuit_point.header.frame_id = "map";
    purepursuit_point.header.stamp = ros::Time::now();
    purepursuit_point.action = visualization_msgs::Marker::ADD;
    purepursuit_point.pose.orientation.w = 1.0;
    purepursuit_point.id = 0;
    purepursuit_point.type = visualization_msgs::Marker::POINTS;

    purepursuit_point.scale.x = 1.0;
    purepursuit_point.scale.y = 1.0;
    purepursuit_point.scale.z = 1.0;

    // purepursuit_point is red
    purepursuit_point.color.g = 1.0;
    purepursuit_point.color.a = 1.0;
    
    purepursuit_point.points.clear();
    geometry_msgs::Point point;
    point.x = next_point_x;
    point.y = next_point_y;
    purepursuit_point.points.push_back(point);

    purepursuit_point_pub.publish(purepursuit_point);
}

void DynamicVehicleControl2::PID(){
    double p, i, d, Kp, Ki, Kd, error, target_speed_ms, previous_error, delta_error, error_integral, error_derivative;
    
    Kp = 0.7;
    Ki = 0.15;
    Kd = 0.001;
    
    double delta_time = 0.01;   
    double high = 0.1;
    target_speed_ms = TARGET_SPEED_KPH / 3.6;

    error = target_speed_ms - curr_speed;
    delta_error = error - previous_error;
    error_integral = error_integral + (error * delta_time);
    if ( error_integral >= (high / Ki) ) {
        error_integral = high / Ki;
    }
    error_derivative = delta_error / delta_time;
    if (delta_error > 10.0) {
        delta_error = 0.0;
    }
    previous_error = error;

    p = Kp * error;
    i = Ki * error_integral;
    d = Kd * error_derivative;
    accelerator = (p + i + d);

    if (accelerator >= 1.0) {
        accelerator = 1.0;
    }
    else if (accelerator < 0.0) {
        accelerator = 0.0;
    }

    if (accelerator < 0 && error < 0) {
        stop = -accelerator;
        if (stop >= 1) {
            stop = 1.0;
        }
        else if (stop <= 0.0) {
            stop = 0.0;
        }
    }
    else {
        stop = 0.0;
    }
}

void DynamicVehicleControl2::Control() {    
    // control_msg.header.stamp = ros::Time::now();
    // control_msg.header.frame_id = "map";
    control_msg.throttle = accelerator;      //0.0 ~ 1.0
    control_msg.steer = angle;               //-1.0 ~ 1.0
    control_msg.brake = stop;                //0.0 ~ 1.0
    // control_msg.hand_brake = false;
    // control_msg.reverse = false;
    // control_msg.gear = 1;                    // D = 1,  N = 0,  R = -1
    // control_msg.manual_gear_shift = false;
}

void DynamicVehicleControl2::Print() {
    // ROS_INFO("curr_speed = %f", curr_speed);
    // ROS_INFO("throttle = %f", accelerator);
    // ROS_INFO("steer = %f", angle);
    // ROS_INFO("brake = %f", stop);
}

void DynamicVehicleControl2::Publish() {
    visualize_waypoint();
    purepursuit_point_marker();
    control_pub.publish(control_msg);
}

void DynamicVehicleControl2::Run() {
    purepursuit_next_point();
    pure_pursuit();
    PID();
    Control();
    Publish();
    Print();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_vehicle_2_control");
    DynamicVehicleControl2 dynamic_vehicle_2_control;
    dynamic_vehicle_2_control.waypoint();
    ros::Rate loop_rate(30);

    while(ros::ok()) {   
        dynamic_vehicle_2_control.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}