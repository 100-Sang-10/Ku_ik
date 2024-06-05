#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
#include <armadillo>
#include <chrono>
#include <thread>
#include <detection_msgs/SensorFusion.h>
#include <eigen3/Eigen/Dense>

#define SPEED_KPH  30
#define sliding_window_dis 5.0
#define init_position_x -138.765396118  // not_waypoint_test
#define init_position_y 186.384170532  // not_waypoint_test
// #define init_position_x 188.886138916  // not_waypoint_test
// #define init_position_y -91.5933456421  // not_waypoint_test
// #define init_position_x -54.9099540710449  // waypoint_test
// #define init_position_y 77.2210540771484  // waypoint_test
// #define init_position_x -184.5  // waypoint_test
// #define init_position_y 63.0  // waypoint_test

#define LOCAL_X                 0
#define LOCAL_Y                 1
#define IDX_DIFF                20
#define MAX_LATERAL_ACCEL_MS2   1.5  // 3.5
#define MAX_SPEED_KPH           30
#define START_END_SPEED_KPH     20
#define WINDOW_SIZE             10
#define THRESHOLD_SIZE          1
#define FCA_DISTANCE            1.5

#define DELIVERY_STOP_DISTANCE  5.0
#define A_ZONE_X                37.1479187012   // -77.0
#define A_ZONE_Y                35.9507026672   // 186.4
#define B_ZONE_X                -133.051208496  // -10.0
#define B_ZONE_Y                -47.4016838074  // 186.4
#define C_ZONE_X                -90.6944885254
#define C_ZONE_Y                -96.712928772
#define D_ZONE_X                87.8212203979
#define D_ZONE_Y                -86.132522583

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
    int following_state = 20;
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

    ros::Subscriber center_marker_sub;
    geometry_msgs::PoseArray center_points;
    // bool global_planning = true;  // waypoint_test
    bool global_planning = false;  // not_waypoint_test
    bool waypoint_stop = false;

    double current_speed;                              // speedometer
    int purepursuit_waypoint_count = 0;
    double Ld, wheelbase, purepursuit_d;
    double purepursuit_current_co_tf_x, purepursuit_current_co_tf_y;
    
    std::vector<double> velocity_vec;
    std::vector<double> filter_vec;
    double target_speed_ms = 30.0 / 3.6;
    int velocity_container_count = 0;
    double stop = 0.0;
    double start_end_speed_ms = 0.0;
    bool delivery_zone = false;
    bool delivery_time = false;
    bool delivery_end = false;
    double distance_a, distance_b, distance_c, distance_d;

    ros::Subscriber object_sub;
    detection_msgs::SensorFusion fusion_msg;
    bool obeject_detection = false;
    double pedestrian_distance = 100;
    double dynamic_vehicle_distance = 100;
    double prev_dynamic_vehicle_distance;

    ros::Publisher lattice_pub;
    std::vector<nav_msgs::Path> out_path;
    ros::Subscriber sub_avoid_state;
    int avoid_state = 70;

  public:
    PointControl();
    void ObjectCallback(const detection_msgs::SensorFusion::ConstPtr& obj_msg);
    void OsmCallback(const geometry_msgs::PoseArray::ConstPtr& center_line_msg);
    void ReadCenterLine();
    std::vector<double> WGS84toCartesian(double input_lat, double input_long);  // (latitude, longitude) to (x, y)
    void waypoint();  //waypoint 담은 vector 함수
    std::vector<std::string> divide(std::string xy_str, char divider);  //벡터에 저장된 문자열 x,y 나눠주는 함수
    double to_num(std::string s);  //좌표 숫자로 바꿔주는 함수
    void visualize_waypoint();  //waypoint marker 띄우는 함수
    void odom_Callback(const nav_msgs::Odometry::ConstPtr& odom_msg);  //현재 좌표와 타겟좌표
    double cal_yaw(const nav_msgs::Odometry::ConstPtr& odom_msg);  //yaw값 구하는 함수
    std::pair<double, double> coordinate_tf(double input_x, double input_y);    // transform coordinate from vehicle
    void next_point();
    void next_speed();
    bool DeliveryZone();
    void DeliveryStop();
    void pure_pursuit();
    void purepursuit_next_point();
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

    void SetVelocityProfile(std::vector<std::vector<double>>& container);
    std::vector<double> MovingAveFilter(const std::vector<double>& data);
    std::vector<double> reconstructionFilter(const std::vector<double>& data);
    
    void ObjectDetection();
    void PedestrianStop();
    void DynamicVehicleVelocity();

    void Dot(double result[3][1], double A[3][3], double B[3][1]);
    void LatticePlanning();
    void LatticeIndex();
    void AvoidStateCallback(const std_msgs::Int64::ConstPtr& avoid_state_msg);
    void Avoid();

    void Print();
    void publish();
    void Run();
};