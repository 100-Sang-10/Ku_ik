#include "point_control.hpp"

PointControl::PointControl() {
    waypoint_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    current_coordinate_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &PointControl::odom_Callback, this);
    current_speed_sub = nh.subscribe("/carla/ego_vehicle/speedometer", 100, &PointControl::speed_Callback, this);
    control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);

    sub_midpoint = nh.subscribe("/mid_point", 100, &PointControl::point_Callback, this);
    point_pub = nh.advertise<visualization_msgs::Marker>("/point_marker", 100);
    sub_state = nh.subscribe("/state", 100, &PointControl::state_Callback, this);
}

void PointControl::waypoint() {
    std::ifstream in("/home/baek/Carla_Internship/catkin_ws/src/waypoint_publisher/xy.txt");  
    
    if (!in.is_open()) {
        ROS_ERROR("x,y file not found!");
    }
    else {
        std::cout << "file reading complete!" << "\n";
    }
    
    std::string xy_filter;
    while (in) {
        getline(in, xy_filter);  //xy_filter 문자열에 한줄씩 저장
        std::vector<double> xy;
        std::vector<std::string> xy_vec = divide(xy_filter, '\t');  //한 줄씩 있는 xy_filter를 tab으로 x와 y 좌표 나눔
        for (typename std::vector<std::string>::iterator itr = xy_vec.begin(); itr != xy_vec.end(); ++itr) {
            xy.push_back(to_num(*itr));
        }
        waypoint_vec.push_back(xy);
    }
}

std::vector<std::string> PointControl::divide(std::string xy_str, char divider){
    std::istringstream s(xy_str);
    std::string buffer;
    std::vector<std::string> result;

    while(getline(s,buffer,divider)){
        result.push_back(buffer);
    }

    return result;
}

double PointControl::to_num(std::string s){
    std::istringstream ss(s);
    double x;
    ss >> x;

    return x;
}

void PointControl::visualize_waypoint(){
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

void PointControl::odom_Callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    // ROS_INFO("vehicle coordinate on map ");
    // ROS_INFO("ego_vehicle.x = %f", odom_msg->pose.pose.position.x);
    // ROS_INFO("ego_vehicle.y = %f \n", odom_msg->pose.pose.position.y);

    ego_vehicle_x = odom_msg->pose.pose.position.x;
    ego_vehicle_y = odom_msg->pose.pose.position.y;
    ego_yaw = cal_yaw(odom_msg);

    // ROS_INFO("waypoint coordinate from vehicle ");
    // ROS_INFO("target.x: %f", co_tf_x);
    // ROS_INFO("target.y: %f \n", co_tf_y);

    pure_pursuit();

    // ROS_INFO("contorl vehicle ");    
    // ROS_INFO("throttle = %f", accelerator);
    // ROS_INFO("steering angle: %f \n", angle);
    // ROS_INFO("-----------------------------------\n");
}

double PointControl::cal_yaw(const nav_msgs::Odometry::ConstPtr& odom_msg) {

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

void PointControl::coordinate_tf() {
    double current_point_x, current_point_y, x, y, a, d;
    double ld = 5.0;  //look ahead distance

    current_point_x = next_point_x;
    current_point_y = next_point_y;
    x = current_point_x - ego_vehicle_x;
    y = current_point_y - ego_vehicle_y;
    a = atan2(y, x) - ego_yaw;
    d = sqrt( pow(x, 2) + pow(y, 2) );
    
    co_tf_x = d * cos(a) + 1.44999998807907104;  //뒷바퀴 기준
    co_tf_y = d * sin(a);

    if (d <= ld) {
        next_point();
    }
}

void PointControl::next_point() {
    std::vector<double> waypoint;
    
    std::vector<std::vector<double>>::iterator itr = waypoint_vec.begin() + waypoint_count;
    waypoint = *itr;

    next_point_x = waypoint[0];
    next_point_y = waypoint[1];

    waypoint_count++;


    // ROS_INFO("next_point");
}

void PointControl::pure_pursuit(){
    double mid_angle, waypoint_angle;

    calc_midpoint();
    coordinate_tf();

    double L, waypoint_d, mid_d;
    L = 2.66503413435;  //wheelbase
    waypoint_d = sqrt( pow(co_tf_x, 2) + pow(co_tf_y, 2) );  //target distance from 뒷바퀴
    waypoint_angle = - atan2( 2 * co_tf_y * L, pow(waypoint_d, 2) ) * (2 / M_PI);

    mid_d = sqrt( pow(mid_co_tf_x, 2) + pow(mid_co_tf_y, 2) );  //target distance from 뒷바퀴
    mid_angle = - atan2( 2 * mid_co_tf_y * L, pow(mid_d, 2) ) * (2 / M_PI);

    if (following_state == 21) {
        angle = mid_angle;
        ROS_INFO("follow_line");
    }
    else if (following_state == 20) {
        angle = waypoint_angle;
        ROS_INFO("follow_waypoint");
    }
    ROS_INFO("following_state = %i", following_state);
    ROS_INFO("-----------------------------------\n");
}

void PointControl::speed_Callback(const std_msgs::Float32::ConstPtr& speed_msg) {
    PID(speed_msg);
}

void PointControl::PID(const std_msgs::Float32::ConstPtr& speed_msg){
    double p, i, d, Kp, Ki, Kd, target_speed, current_speed, error, previous_error, delta_error, error_integral, error_derivative;

    Kp = 0.7;
    Ki = 0.15;
    Kd = 0.001;
    
    double delta_time = 0.01;   
    double high = 0.1;
    target_speed = SPEED / 3.6;

    current_speed = speed_msg->data;
    error = target_speed - current_speed;
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
}

void PointControl::control() {
    carla_msgs::CarlaEgoVehicleControl move;

    move.header.stamp = ros::Time::now();
    move.header.frame_id = "map";
    move.throttle = accelerator;      //0.0 ~ 1.0
    move.steer = angle;               //-1.0 ~ 1.0
    move.brake = 0.0;                 //0.0 ~ 1.0
    move.hand_brake = false;
    move.reverse = false;
    move.gear = 1;                    // D = 1,  N = 0,  R = -1
    move.manual_gear_shift = false;

    control_pub.publish(move);
}

void PointControl::point_Callback(const geometry_msgs::PoseArray::ConstPtr& line_msg) {
    midpoint = *line_msg;
}

void PointControl::calc_midpoint() {
    int num = midpoint.poses.size();
    // ROS_INFO("midpoint size = %i", num);
    double sum_x, sum_y;
    double pixel_x, pixel_y;
    double dis_x, dis_y;

    for (int i = 0; i < num - 1; i++) {
        double x, y;
        x = midpoint.poses[i].position.x;
        y = midpoint.poses[i].position.y;
        sum_x += x;
        sum_y += y;
        // ROS_INFO("x = %f", x);
        // ROS_INFO("y = %f", y);
    }
    pixel_x = sum_x / (num - 1);
    pixel_y = sum_y / (num - 1);
    dis_x = ((5.25/800)*500) - (pixel_y * (5.25/800));
    dis_y = 5.25 - (pixel_x * (5.25/800));
    mid_co_tf_x = dis_x + sliding_window_dis;
    mid_co_tf_y = dis_y - (5.25/2);

    // ROS_INFO("mid_co_tf_x = %f", mid_co_tf_x);
    // ROS_INFO("mid_co_tf_y = %f", mid_co_tf_y);
}

void PointControl::mid_point_marker() {
    double x, y, d_x, d_y, a, d;
    
    x = mid_co_tf_x - 1.206373665536404;  //뒷바퀴 기준
    y = mid_co_tf_y;
    d = sqrt( pow(x, 2) + pow(y, 2) );
    a = atan2(y, x) + ego_yaw;
    d_x = d * cos(a);
    d_y = d * sin(a);

    mid_point_marker_x = d_x + ego_vehicle_x;
    mid_point_marker_y = d_y + ego_vehicle_y;

    // ROS_INFO("mid_point_marker_x = %f", mid_point_marker_x);
    // ROS_INFO("mid_point_marker_y = %f", mid_point_marker_y);

    line_mid_point.header.frame_id = "map";
    line_mid_point.header.stamp = ros::Time::now();
    line_mid_point.action = visualization_msgs::Marker::ADD;
    line_mid_point.pose.orientation.w = 1.0;
    line_mid_point.id = 1;
    line_mid_point.type = visualization_msgs::Marker::POINTS;

    line_mid_point.scale.x = 1.0;
    line_mid_point.scale.y = 1.0;
    line_mid_point.scale.z = 1.0;

    // line_mid_point is red
    line_mid_point.color.r = 1.0;
    line_mid_point.color.a = 1.0;
    
    line_mid_point.points.clear();
    geometry_msgs::Point point;
    point.x = mid_point_marker_x;
    point.y = mid_point_marker_y;
    line_mid_point.points.push_back(point);

    point_pub.publish(line_mid_point);
}

void PointControl::state_Callback(const std_msgs::Int64::ConstPtr& state_msg) {
    following_state = state_msg->data;
}

int main( int argc, char** argv ) {
    ros::init(argc, argv, "point_control");
    PointControl point_control;
    point_control.waypoint();
    ros::Rate loop_rate(30);  //1초에 30번

    while(ros::ok()) {
        point_control.visualize_waypoint();
        point_control.mid_point_marker();
        point_control.control();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};