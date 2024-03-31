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

#define SPEED  30
#define sliding_window_dis 5.0

class PointControl {
  private:
    ros::NodeHandle nh;
    ros::Publisher waypoint_pub;
    std::vector<std::vector<double>> waypoint_vec;
    ros::Subscriber current_coordinate_sub;
    ros::Subscriber current_speed_sub;
    ros::Publisher control_pub;
    double angle, co_tf_x, co_tf_y, accelerator, mid_co_tf_x, mid_co_tf_y;
    double next_point_x = -54.9099540710449;
    double next_point_y = 77.2210540771484;
    int waypoint_count = 0;

    ros::Subscriber sub_midpoint;
    geometry_msgs::PoseArray midpoint;
    int following_state = 0;
    ros::Publisher point_pub;
    visualization_msgs::Marker line_mid_point;
    ros::Subscriber sub_state;
    double mid_point_marker_x = 0.0;
    double mid_point_marker_y = 0.0;
    double ego_vehicle_x = 0.0;
    double ego_vehicle_y = 0.0;
    double ego_yaw = 0.0;

  public:
    PointControl();
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
    void state_Callback(const std_msgs::Int64::ConstPtr& state_msg);
};