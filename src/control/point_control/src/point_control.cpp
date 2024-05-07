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

    object_sub = nh.subscribe("/fusion_info", 100, &PointControl::ObjectCallback, this);
}

void PointControl::ObjectCallback(const detection_msgs::SensorFusion::ConstPtr& obj_msg) {
    fusion_msg = *obj_msg;
    bool object_callback = fusion_msg.toggle;
    if (object_callback) {
        obeject_detection = true;
    }
    else {
        obeject_detection = false;
    }
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
            SetVelocityProfile(waypoint_vec);
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
    // std::ifstream in("/home/baek/git/Ku_ik/resources/waypoint_center.txt");
    // std::ifstream in("/home/baek/git/Ku_ik/resources/dynamic_vehicle_waypoint.txt");
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
    SetVelocityProfile(waypoint_vec);
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
    // ROS_INFO("next_point start");
    int count = 0;
    
    std::vector<double> waypoint;

    for(std::vector<std::vector<double>>::iterator itr = waypoint_vec.begin(); itr != waypoint_vec.end(); ++itr) {
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
}

void PointControl::next_speed() {
    // ROS_INFO("next_speed start");
    int count = 0;

    for(std::vector<double>::iterator itr = velocity_vec.begin(); itr != velocity_vec.end(); ++itr) {
        if(count == velocity_container_count){
            break;
        }
        // ROS_INFO("velocity_container for");

        target_speed_ms = *itr;

        count++;
    }

    velocity_container_count++;
    if (velocity_container_count >= velocity_vec.size()) {
        target_speed_ms = 0;
    }
}

bool PointControl::DeliveryZone() {
    bool delivery_zone = false;
    double distance = DELIVERY_STOP_DISTANCE;
    double a_x = A_ZONE_X;
    double a_y = A_ZONE_Y;
    double b_x = B_ZONE_X;
    double b_y = B_ZONE_Y;
    double c_x = C_ZONE_X;
    double c_y = C_ZONE_Y;
    double d_x = D_ZONE_X;
    double d_y = D_ZONE_Y;
    double x_a = ego_vehicle_x - a_x;
    double y_a = ego_vehicle_y - a_y;
    double x_b = ego_vehicle_x - b_x;
    double y_b = ego_vehicle_y - b_y;
    double x_c = ego_vehicle_x - c_x;
    double y_c = ego_vehicle_y - c_y;
    double x_d = ego_vehicle_x - d_x;
    double y_d = ego_vehicle_y - d_y;
    distance_a = sqrt(pow(x_a, 2) + pow(y_a, 2));
    distance_b = sqrt(pow(x_b, 2) + pow(y_b, 2));
    distance_c = sqrt(pow(x_c, 2) + pow(y_c, 2));
    distance_d = sqrt(pow(x_d, 2) + pow(y_d, 2));
    if (distance_a < distance || distance_b < distance || distance_c < distance || distance_d < distance) {
        delivery_zone = true; 
    }
    else {
        delivery_zone = false;
        delivery_end = false;
    }

    return delivery_zone;
}

void PointControl::DeliveryStop() {
    if (delivery_zone) {
        target_speed_ms = 0.0;
        if(current_speed == 0) {
            delivery_time = true;
        }
    }
}

void PointControl::pure_pursuit() {
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

void PointControl::purepursuit_next_point() {
    wheelbase = 2.66503413435;     // wheelbase
    double velocity = current_speed * 3.6;
    Ld = 0 + (0.25 * velocity);  // TODO:tunnings
    // Ld = 5.0;
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
            next_speed();
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
    double p, i, d, Kp, Ki, Kd, error, previous_error, delta_error, error_integral, error_derivative;
    
    Kp = 0.7;
    Ki = 0.15;
    Kd = 0.001;
    
    double delta_time = 0.01;   
    double high = 0.1;
    // target_speed_ms = SPEED_KPH / 3.6;

    current_speed = speed_msg->data;
    error = target_speed_ms - current_speed;
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

void PointControl::control() {
    carla_msgs::CarlaEgoVehicleControl move;

    move.header.stamp = ros::Time::now();
    move.header.frame_id = "map";
    move.throttle = accelerator;      //0.0 ~ 1.0
    move.steer = angle;               //-1.0 ~ 1.0
    move.brake = stop;                 //0.0 ~ 1.0
    move.hand_brake = false;
    move.reverse = false;
    move.gear = 1;                    // D = 1,  N = 0,  R = -1
    move.manual_gear_shift = false;
    if (global_planning) {
        control_pub.publish(move);
        if (delivery_zone && delivery_time) {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            delivery_time = false;
            delivery_end = true;
            next_speed();
        }
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

void PointControl::SetVelocityProfile(std::vector<std::vector<double>>& container) {
    std::vector<double> velocity_container;
    // ROS_INFO("size = %i", container.size());

    for (int idx = IDX_DIFF; idx < container.size()-IDX_DIFF; idx++) {
        arma::vec prev_now(3, arma::fill::zeros);
        arma::vec now_prev(3, arma::fill::zeros);
        arma::vec next_now(3, arma::fill::zeros);
        arma::vec next_prev(3, arma::fill::zeros);
        // ROS_INFO("idx = %i", idx);
        

        prev_now = {
            container[idx-IDX_DIFF][LOCAL_X] - container[idx][LOCAL_X],
            container[idx-IDX_DIFF][LOCAL_Y] - container[idx][LOCAL_Y],
            0.0     
        };
        now_prev = {
            container[idx][LOCAL_X] - container[idx-IDX_DIFF][LOCAL_X],
            container[idx][LOCAL_Y] - container[idx-IDX_DIFF][LOCAL_Y],
            0.0
        };        
        next_now = {
            container[idx+IDX_DIFF-1][LOCAL_X] - container[idx][LOCAL_X],
            container[idx+IDX_DIFF-1][LOCAL_Y] - container[idx][LOCAL_Y],
            0.0
        };
        next_prev = {
            container[idx+IDX_DIFF-1][LOCAL_X] - container[idx-IDX_DIFF][LOCAL_X],
            container[idx+IDX_DIFF-1][LOCAL_Y] - container[idx-IDX_DIFF][LOCAL_Y],
            0.0
        };        
        // ROS_INFO("01 = %f", container[idx-IDX_DIFF][LOCAL_X]);
        // ROS_INFO("02 = %f", container[idx][LOCAL_X]);
        // ROS_INFO("11 = %f", container[idx-IDX_DIFF][LOCAL_Y]);
        // ROS_INFO("12 = %f", container[idx][LOCAL_Y]);
        // ROS_INFO("x = %f", prev_now[0]);
        // ROS_INFO("y = %f", prev_now[1]);
        // ROS_INFO("size = %i", prev_now.size());

        double boonja = 2.0 * arma::norm(arma::cross(next_now, prev_now), 2);
        double boonmo = arma::norm(next_now, 2) * arma::norm(now_prev, 2) * arma::norm(next_prev, 2);
        double kappa = boonja / boonmo;
        double max_vel_ms = sqrt(MAX_LATERAL_ACCEL_MS2 / (kappa));
        // double max_vel_ms = 0.5 * sqrt(MAX_LATERAL_ACCEL_MS2 / (kappa));
        double speed_max = MAX_SPEED_KPH/3.6;
        if(max_vel_ms > speed_max) {
            max_vel_ms = speed_max;
        }
        velocity_container.push_back(max_vel_ms);
        // ROS_INFO("velocity_container pushback");
    }
    start_end_speed_ms = START_END_SPEED_KPH/3.6;
    for (int i = 0; i < IDX_DIFF; i++) {
        velocity_container.insert(velocity_container.begin(), start_end_speed_ms);
    }
    for (int i = 0; i < IDX_DIFF / 2; i++) {
        velocity_container.push_back(start_end_speed_ms);
    }
    for (int i = 0; i < IDX_DIFF / 2; i++) {
        double end_speed = 0.0;
        velocity_container.push_back(end_speed);
    }
    // ROS_INFO("velocity_container size = %i", velocity_container.size());
    // for (int i = 0; i < velocity_container.size(); i++) {
    //     double vel = velocity_container[i] * 3.6;
    //     ROS_INFO("speed = %f", vel);
    // }
    filter_vec = MovingAveFilter(velocity_container);
    // ROS_INFO("filter_vec size = %i", filter_vec.size());
    velocity_vec = reconstructionFilter(filter_vec);
    // ROS_INFO("velocity_vec size = %i", velocity_vec.size());
    // for (int i = 0; i < velocity_vec.size(); i++) {
    //     double vel = velocity_vec[i] * 3.6;
    //     ROS_INFO("speed = %f", vel);
    // }
}

std::vector<double> PointControl::MovingAveFilter(const std::vector<double>& data) {
    std::vector<double> result;
    int dataSize = data.size();
    int windowSize = WINDOW_SIZE;
    
    // 데이터가 windowSize보다 작으면 그대로 반환
    if (dataSize < windowSize) {
        return result;
    }

    for (int i = 0; i <= dataSize - windowSize; ++i) {
        double sum = 0.0;
        for (int j = i; j < i + windowSize; ++j) {
            sum += data[j];
        }
        result.push_back(sum / windowSize);
    }
    result.erase(result.end() - 1);

    for (int i = 0; i < windowSize; i++) {
        result.insert(result.begin(), start_end_speed_ms);
    }

    return result;
}

std::vector<double> PointControl::reconstructionFilter(const std::vector<double>& data) {
    std::vector<double> result;
    double threshold = THRESHOLD_SIZE;

    // 첫 번째 데이터는 그대로 저장
    result.push_back(data[0]);

    // 데이터 변화를 추적하고, 변화가 큰 경우 smoothing
    for (int i = 1; i < data.size() - 1; ++i) {
        double diff1 = std::abs(data[i] - data[i - 1]);
        double diff2 = std::abs(data[i] - data[i + 1]);

        // 변화가 큰 경우 smoothing
        if (diff1 > threshold && diff2 > threshold) {
            result.push_back((data[i - 1] + data[i] + data[i + 1]) / 3.0);
        } else {
            result.push_back(data[i]);
        }
    }

    // 마지막 데이터는 그대로 저장
    result.push_back(data[data.size() - 1]);

    return result;
}

void PointControl::ObjectDetection() {
    std::string object_type = fusion_msg.Class;
    double object_distance = fusion_msg.distance;
    if (object_type == "pedesatrian") {
        pedestrian_distance = object_distance;
        PedestrianStop();
    }
    else if(object_type == "vehicle") {
        dynamic_vehicle_distance = object_distance;
        DynamicVehicleVelocity();
    }
}

void PointControl::PedestrianStop() {
    if(pedestrian_distance <= 7) {
        accelerator = 0.0;
        stop = 1.0;
    }
}

void PointControl::DynamicVehicleVelocity() {
    if(dynamic_vehicle_distance < 20) {
        accelerator = 0.0;
        if(dynamic_vehicle_distance < 10) {
            stop = 0.5;
        }
        else if(dynamic_vehicle_distance < 5) {
            stop = 1.0;
        }
        else {
            stop = 0.0;
        }
    }
    else if(dynamic_vehicle_distance > 20) {
        accelerator += 0.1;
    }
}

void PointControl::Print() {
    // ROS_INFO("next_point = %f, %f", next_point_x, next_point_y);
    double target_speed_kph = target_speed_ms * 3.6;
    ROS_INFO("target_speed_kph = %f", target_speed_kph);
    ROS_INFO("distance_a = %f", distance_a);
    ROS_INFO("distance_b = %f", distance_b);
    ROS_INFO("delivery_end = %i", delivery_end);
    ROS_INFO("delivery_zone = %i", delivery_zone);
    ROS_INFO("delivery_time = %i", delivery_time);
    ROS_INFO("========================================");
}

void PointControl::publish() {
    visualize_waypoint();
    mid_point_marker();
    purepursuit_point_marker();
    calc_sliding_window_error();
}

void PointControl::Run() {
    if (!waypoint_stop) {
        // ReadCenterLine();  // not_waypoint_test
    }
    if (global_planning) {
        purepursuit_next_point();
        delivery_zone = DeliveryZone();
        if (!delivery_end) {
            DeliveryStop();
        }
        if (obeject_detection) {
            ObjectDetection();
        }
        publish();
    }
    Print();
}

int main( int argc, char** argv ) {
    ros::init(argc, argv, "point_control");
    PointControl point_control;
    point_control.waypoint();  // waypoint_test
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