#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int64.h>
#include <WGS84toCartesian.hpp>

#define SPEED  30
#define sliding_window_dis 5.0
#define init_position_x -54.9099540710449
#define init_position_y 77.2210540771484;

class PointControl {
  private:
    ros::NodeHandle nh;
    ros::Publisher waypoint_pub;
    std::vector<std::vector<double>> waypoint_vec;
    ros::Subscriber current_coordinate_sub;
    ros::Subscriber current_speed_sub;
    ros::Publisher control_pub;
    double angle, co_tf_x, co_tf_y, accelerator, mid_co_tf_x, mid_co_tf_y;
    double next_point_x = init_position_x;
    double next_point_y = init_position_y;
    double mid_angle, waypoint_angle;
    int waypoint_count = 0;
    
    // use for convert WSG84 to Cartesian
    std::array<double, 2> WSG84Reference{0, 0};
    ros::Subscriber sub_midpoint;
    geometry_msgs::PoseArray midpoint;
    int following_state = 0;
    ros::Publisher point_pub;
    visualization_msgs::Marker line_mid_point;
    ros::Publisher purepursuit_point_pub;
    visualization_msgs::Marker purepursuit_point;
    ros::Subscriber sub_state;
    double mid_point_marker_x = 0.0;
    double mid_point_marker_y = 0.0;
    double ego_vehicle_x = 0.0;
    double ego_vehicle_y = 0.0;
    double ego_yaw = 0.0;
    double sliding_window_error;
    ros::Publisher sliding_window_error_pub;
    std_msgs::Float32 sliding_window_error_msg;
    double sliding_window_near_point_x;
    double sliding_window_near_point_y;

    double lat_error, long_error;
    ros::Publisher crosstrack_error_pub;     // cross track error
    std_msgs::Float32 crosstrack_error_msg;  // cross track error

  public:
    PointControl();
    std::vector<double> WGS84toCartesian(double input_lat, double input_long);  // (latitude, longitude) to (x, y)
    void waypoint();  //waypoint 담은 vector 함수
    std::vector<std::string> divide(std::string xy_str, char divider);  //벡터에 저장된 문자열 x,y 나눠주는 함수
    double to_num(std::string s);  //좌표 숫자로 바꿔주는 함수
    void visualize_waypoint();  //waypoint marker 띄우는 함수
    void odom_Callback(const nav_msgs::Odometry::ConstPtr& odom_msg);  //현재 좌표와 타겟좌표
    double cal_yaw(const nav_msgs::Odometry::ConstPtr& odom_msg);  //yaw값 구하는 함수
    void coordinate_tf();  //좌표 변환 함수
    void next_point();  //다음 waypoint 
    void pure_pursuit();
    void speed_Callback(const std_msgs::Float32::ConstPtr& speed_msg);  //현재 속도
    void PID(const std_msgs::Float32::ConstPtr& speed_msg);
    void control();

    void point_Callback(const geometry_msgs::PoseArray::ConstPtr& line_msg);
    void calc_midpoint();
    void mid_point_marker();
    void purepursuit_point_marker();
    void state_Callback(const std_msgs::Int64::ConstPtr& state_msg);
    void calc_sliding_window_near_point();
    void calc_sliding_window_error();
    void lateral_error();

    void publish();
};