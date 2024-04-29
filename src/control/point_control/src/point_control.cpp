#include "point_control.hpp"

PointControl::PointControl() {
    waypoint_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    current_coordinate_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &PointControl::odom_Callback, this);
    current_speed_sub = nh.subscribe("/carla/ego_vehicle/speedometer", 100, &PointControl::speed_Callback, this);
    control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 100);

    sub_midpoint = nh.subscribe("/mid_point", 100, &PointControl::point_Callback, this);
    point_pub = nh.advertise<visualization_msgs::Marker>("/point_marker", 100);
    purepursuit_point_pub = nh.advertise<visualization_msgs::Marker>("/purepursuit_point_marker", 100);
    sub_state = nh.subscribe("/state", 100, &PointControl::state_Callback, this);
    sliding_window_error_pub = nh.advertise<std_msgs::Float32>("/sliding_window_error",100);

    center_marker_sub = nh.subscribe("/center_line_point", 100, &PointControl::OsmCallback, this);
}

void PointControl::OsmCallback(const geometry_msgs::PoseArray::ConstPtr& center_line_msg){
    center_points = *center_line_msg;
    // int num = center_points.poses.size();
    // for(int i = 0; i < num; i++) {
    //     geometry_msgs::Pose c;
    //     geometry_msgs::Point p;
    //     c = center_points.poses[i];
    //     p = c.position;
    //     double x = p.x;
    //     double y = p.y;
    //     ROS_INFO("x = %f, y = %f", x, y);
    // }
}

void PointControl::ReadCenterLine() {
    int num = center_points.poses.size();
    // ROS_INFO("num = %i", num);
    if (num != 0) {
        global_planning = true;
        if (!waypoint_stop) {
            for(int i = 0; i < num; i++) {
                geometry_msgs::Pose c;
                geometry_msgs::Point p;
                std::vector<double> map_XY;
                c = center_points.poses[i];
                p = c.position;
                map_XY.push_back(p.x);
                map_XY.push_back(p.y);
                // ROS_INFO("x = %f, y = %f", map_XY[0], map_XY[1]);
                waypoint_vec.push_back(map_XY);
            }
            waypoint_stop = true;
        }
    }
}

std::vector<double> PointControl::WGS84toCartesian(double input_lat, double input_long) {
    std::array<double, 2> WGS84Position {input_lat, input_long};
    std::array<double, 2> cartesian_position {wgs84::toCartesian(WSG84Reference, WGS84Position)};

    std::vector<double> XY;
    XY.push_back(cartesian_position[0]);
    XY.push_back(cartesian_position[1]);
    
    // ROS_INFO("x = %f", cartesian_position[0]);
    // ROS_INFO("y = %f", cartesian_position[1]);
    
    return XY;
}

void PointControl::waypoint() {
    std::ifstream in("/home/baek/git/Ku_ik/resources/waypoint_center.txt");

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
    // waypoint_pub.publish(points);
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
    if (global_planning) {
        pure_pursuit();
    }

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

std::pair<double, double> PointControl::coordinate_tf(double input_x, double input_y) {
    double x, y, a, d, output_x, output_y;

    x = input_x - ego_vehicle_x;
    y = input_y - ego_vehicle_y;
    a = atan2(y, x) - ego_yaw;
    d = sqrt( pow(x, 2) + pow(y, 2) );
    
    output_x = d * cos(a);
    output_y = d * sin(a);

    return std::make_pair(output_x, output_y);
}

void PointControl::next_point() {
    int count = 0;
    
    std::vector<double> waypoint;

    for(std::vector<std::vector<double>>::iterator itr = waypoint_vec.begin(); itr != waypoint_vec.end(); ++itr) {
        if(count == purepursuit_waypoint_count){
            break;
        }

        waypoint = *itr;

        next_point_x = waypoint[0];
        next_point_y = waypoint[1];

        count++;
    }

    purepursuit_waypoint_count++;
}

void PointControl::pure_pursuit(){
    waypoint_angle = - atan2( 2 * purepursuit_current_co_tf_y * wheelbase, pow(purepursuit_d, 2) ) * (2 / M_PI);  // steering: -1.0 ~ 1.0
    // waypoint_angle *= 1.221730351448059;                                                              // steering ratio

    double mid_d;
    calc_midpoint();
    mid_d = sqrt( pow(mid_co_tf_x, 2) + pow(mid_co_tf_y, 2) );  //target distance from 뒷바퀴
    mid_angle = - atan2( 2 * mid_co_tf_y * wheelbase, pow(mid_d, 2) ) * (2 / M_PI);

    if (following_state == 21) {
        angle = mid_angle;
        // ROS_INFO("follow_line");
    }
    else if (following_state == 20) {
        angle = waypoint_angle;
        // ROS_INFO("follow_waypoint");
    }
    // ROS_INFO("following_state = %i", following_state);
    // ROS_INFO("next_point_x = %f", next_point_x);
    // ROS_INFO("next_point_y = %f", next_point_y);
    // ROS_INFO("-----------------------------------\n");

    if (angle >= 1.0) {
        angle = 1.0;
    }
    else if (angle <= -1.0) {
        angle = -1.0;
    }
}

void PointControl::purepursuit_next_point(){
    wheelbase = 2.66503413435;     // wheelbase
    // Ld = -1.0 + (0.125 * current_speed);  // look ahead distance
    Ld = 5.0;
    if (Ld <= 2.0) {
        Ld = 2.0;
    }

    while(1) {
        std::pair<double, double> p = coordinate_tf(next_point_x, next_point_y);
        purepursuit_current_co_tf_x = p.first;
        purepursuit_current_co_tf_y = p.second;
        purepursuit_current_co_tf_x += 1.206373665536404;                                                 // from rear wheel
        purepursuit_d = sqrt( pow(purepursuit_current_co_tf_x, 2) + pow(purepursuit_current_co_tf_y, 2) );  // target distance from rear wheel
        if (purepursuit_d <= Ld) {
            next_point();
        }
        else {
            break;
        }
    }
}

void PointControl::speed_Callback(const std_msgs::Float32::ConstPtr& speed_msg) {
    PID(speed_msg);
}

void PointControl::PID(const std_msgs::Float32::ConstPtr& speed_msg){
    double p, i, d, Kp, Ki, Kd, target_speed, error, previous_error, delta_error, error_integral, error_derivative;

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
    if (global_planning) {
        control_pub.publish(move);
    }
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
void PointControl::purepursuit_point_marker() {
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
    line_mid_point.id = 0;
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

void PointControl::calc_sliding_window_near_point() {
    sliding_window_near_point_x = next_point_x;
    sliding_window_near_point_y = next_point_y;
}

void PointControl::calc_sliding_window_error() {
    calc_sliding_window_near_point();
    double x, y, d;
    x = mid_point_marker_x - sliding_window_near_point_x;
    y = mid_point_marker_y - sliding_window_near_point_y;
    d = sqrt( pow(x, 2) + pow(y, 2) );
    sliding_window_error = d;
    sliding_window_error_msg.data = sliding_window_error;
    if(sliding_window_error_msg.data >= 0.4){
        following_state = 20;
        angle = waypoint_angle;
    }
   
        
    sliding_window_error_pub.publish(sliding_window_error_msg);
}

void PointControl::Print() {
    ROS_INFO("next_point = %f, %f", next_point_x, next_point_y);
}

void PointControl::publish() {
    visualize_waypoint();
    mid_point_marker();
    purepursuit_point_marker();
    calc_sliding_window_error();
}

void PointControl::Run() {
    if (!waypoint_stop) {
        ReadCenterLine();
    }
    if (global_planning) {
        // pure_pursuit();

        purepursuit_next_point();
        publish();
    }
    Print();
}

int main( int argc, char** argv ) {
    ros::init(argc, argv, "point_control");
    PointControl point_control;
    // point_control.waypoint();
    // point_control.ReadCenterLine();
    ros::Rate loop_rate(30);  //1초에 30번

    while(ros::ok()) {
        point_control.Run();
        point_control.control();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};