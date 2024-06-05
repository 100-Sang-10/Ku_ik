#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <carla_msgs/CarlaWalkerControl.h>
#include <nav_msgs/Odometry.h>

#define PEDESTRIAN_SPEED  4
#define PEDESTRIAN1_X1    35.0
#define PEDESTRIAN1_X2    22.5
#define PEDESTRIAN2_X1    27.5
#define PEDESTRIAN2_X2    40.0

class PedestrianControl {
  private:
    ros::NodeHandle nh;

    ros::Publisher pedestrain_1_control_pub;
    ros::Subscriber pedestrain_1_speed_sub;
    ros::Subscriber pedestrain_1_odom_sub;
    ros::Publisher pedestrain_2_control_pub;
    ros::Subscriber pedestrain_2_speed_sub;
    ros::Subscriber pedestrain_2_odom_sub;

    carla_msgs::CarlaWalkerControl pedestrain_1_cmd_msg;
    geometry_msgs::Pose pedestrain_1_curr_pose;
    float pedestrain_1_curr_speed;
    carla_msgs::CarlaWalkerControl pedestrain_2_cmd_msg;
    geometry_msgs::Pose pedestrain_2_curr_pose;
    float pedestrain_2_curr_speed;

    bool pedestrain_1_go = true;
    bool pedestrain_1_return = false;
    bool pedestrain_2_go = true;
    bool pedestrain_2_return = false;
    
  public:
    PedestrianControl();
    ~PedestrianControl();

    void Pedestrian1SpeedCallback(const std_msgs::Float32ConstPtr speed_msg);
    void Pedestrian1PoseCallback(const nav_msgs::OdometryConstPtr odom_msg);
    void Pedestrian1Control1();
    void Pedestrian1Control2();
    void Pedestrian1Control();

    void Pedestrian2SpeedCallback(const std_msgs::Float32ConstPtr speed_msg);
    void Pedestrian2PoseCallback(const nav_msgs::OdometryConstPtr odom_msg);
    void Pedestrian2Control1();
    void Pedestrian2Control2();
    void Pedestrian2Control();

    void Print();
    void Publish();
    void Run();
};

PedestrianControl::PedestrianControl() {
    pedestrain_1_control_pub = nh.advertise<carla_msgs::CarlaWalkerControl>("/carla/pedestrian_1/walker_control_cmd", 10);
    pedestrain_1_speed_sub = nh.subscribe("/carla/pedestrian_1/speedometer", 10, &PedestrianControl::Pedestrian1SpeedCallback, this);
    pedestrain_1_odom_sub = nh.subscribe("/carla/pedestrian_1/odometry", 10, &PedestrianControl::Pedestrian1PoseCallback, this);

    pedestrain_2_control_pub = nh.advertise<carla_msgs::CarlaWalkerControl>("/carla/pedestrian_2/walker_control_cmd", 10);
    pedestrain_2_speed_sub = nh.subscribe("/carla/pedestrian_2/speedometer", 10, &PedestrianControl::Pedestrian2SpeedCallback, this);
    pedestrain_2_odom_sub = nh.subscribe("/carla/pedestrian_2/odometry", 10, &PedestrianControl::Pedestrian2PoseCallback, this);
}

PedestrianControl::~PedestrianControl() {}

void PedestrianControl::Pedestrian1SpeedCallback(const std_msgs::Float32ConstPtr speed_msg) {
    pedestrain_1_curr_speed = speed_msg->data;
}

void PedestrianControl::Pedestrian1PoseCallback(const nav_msgs::OdometryConstPtr odom_msg) {
    pedestrain_1_curr_pose = odom_msg->pose.pose;
}

void PedestrianControl::Pedestrian2SpeedCallback(const std_msgs::Float32ConstPtr speed_msg) {
    pedestrain_2_curr_speed = speed_msg->data;
}

void PedestrianControl::Pedestrian2PoseCallback(const nav_msgs::OdometryConstPtr odom_msg) {
    pedestrain_2_curr_pose = odom_msg->pose.pose;
}

void PedestrianControl::Pedestrian1Control1() {
    geometry_msgs::PoseStamped pedestrain_1_next_pose;
    pedestrain_1_next_pose.pose.position.x = PEDESTRIAN1_X1;
    pedestrain_1_next_pose.pose.position.y = 124.0;

    double direction_x = pedestrain_1_next_pose.pose.position.x - pedestrain_1_curr_pose.position.x;
    double direction_y = pedestrain_1_next_pose.pose.position.y - pedestrain_1_curr_pose.position.y;
    double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

    pedestrain_1_cmd_msg.speed = PEDESTRIAN_SPEED;
    pedestrain_1_cmd_msg.direction.x = direction_x / direction_norm;
    pedestrain_1_cmd_msg.direction.y = direction_y / direction_norm;

    double distance = abs(pedestrain_1_curr_pose.position.x - PEDESTRIAN1_X1);
    if (distance < 2) {
        pedestrain_1_go = false;
        pedestrain_1_return = true;
    }
    else {
        pedestrain_1_go = true;
        pedestrain_1_return = false;
    }
}

void PedestrianControl::Pedestrian1Control2() {
    geometry_msgs::PoseStamped pedestrain_1_next_pose;
    pedestrain_1_next_pose.pose.position.x = PEDESTRIAN1_X2;
    pedestrain_1_next_pose.pose.position.y = 124.0;

    double direction_x = pedestrain_1_next_pose.pose.position.x - pedestrain_1_curr_pose.position.x;
    double direction_y = pedestrain_1_next_pose.pose.position.y - pedestrain_1_curr_pose.position.y;
    double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

    pedestrain_1_cmd_msg.speed = PEDESTRIAN_SPEED;
    pedestrain_1_cmd_msg.direction.x = direction_x / direction_norm;
    pedestrain_1_cmd_msg.direction.y = direction_y / direction_norm;

    double distance = abs(pedestrain_1_curr_pose.position.x - PEDESTRIAN1_X2);
    if (distance < 2) {
        pedestrain_1_go = true;
        pedestrain_1_return = false;
    }
    else {
        pedestrain_1_go = false;
        pedestrain_1_return = true;
    }
}

void PedestrianControl::Pedestrian1Control() {
    if(pedestrain_1_go) {
        Pedestrian1Control1();
    }
    else if(pedestrain_1_return) {
        Pedestrian1Control2();
    }
}

void PedestrianControl::Pedestrian2Control1() {
    geometry_msgs::PoseStamped pedestrain_2_next_pose;
    pedestrain_2_next_pose.pose.position.x = PEDESTRIAN2_X1;
    pedestrain_2_next_pose.pose.position.y = -124.0;

    double direction_x = pedestrain_2_next_pose.pose.position.x - pedestrain_2_curr_pose.position.x;
    double direction_y = pedestrain_2_next_pose.pose.position.y - pedestrain_2_curr_pose.position.y;
    double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

    pedestrain_2_cmd_msg.speed = PEDESTRIAN_SPEED;
    pedestrain_2_cmd_msg.direction.x = direction_x / direction_norm;
    pedestrain_2_cmd_msg.direction.y = direction_y / direction_norm;

    double distance = abs(pedestrain_2_curr_pose.position.x - PEDESTRIAN2_X1);
    if (distance < 2) {
        pedestrain_2_go = false;
        pedestrain_2_return = true;
    }
    else {
        pedestrain_2_go = true;
        pedestrain_2_return = false;
    }
}

void PedestrianControl::Pedestrian2Control2() {
    geometry_msgs::PoseStamped pedestrain_2_next_pose;
    pedestrain_2_next_pose.pose.position.x = PEDESTRIAN2_X2;
    pedestrain_2_next_pose.pose.position.y = -124.0;

    double direction_x = pedestrain_2_next_pose.pose.position.x - pedestrain_2_curr_pose.position.x;
    double direction_y = pedestrain_2_next_pose.pose.position.y - pedestrain_2_curr_pose.position.y;
    double direction_norm = sqrt( pow(direction_x, 2) + pow(direction_y, 2) );

    pedestrain_2_cmd_msg.speed = PEDESTRIAN_SPEED;
    pedestrain_2_cmd_msg.direction.x = direction_x / direction_norm;
    pedestrain_2_cmd_msg.direction.y = direction_y / direction_norm;

    double distance = abs(pedestrain_2_curr_pose.position.x - PEDESTRIAN2_X2);
    if (distance < 2) {
        pedestrain_2_go = true;
        pedestrain_2_return = false;
    }
    else {
        pedestrain_2_go = false;
        pedestrain_2_return = true;
    }
}

void PedestrianControl::Pedestrian2Control() {
    if(pedestrain_2_go) {
        Pedestrian2Control1();
    }
    else if(pedestrain_2_return) {
        Pedestrian2Control2();
    }
}

void PedestrianControl::Print() {
    // ROS_INFO("pedestrain_1_go = %f", pedestrain_1_go);
    // ROS_INFO("pedestrain_1_return = %f", pedestrain_1_return);
}

void PedestrianControl::Publish() {
    pedestrain_1_control_pub.publish(pedestrain_1_cmd_msg);
    pedestrain_2_control_pub.publish(pedestrain_2_cmd_msg);
}

void PedestrianControl::Run() {
    Pedestrian1Control();
    Pedestrian2Control();
    Publish();
    Print();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pedestrian_control");
    PedestrianControl pedestrian_control;
    ros::Rate loop_rate(30);

    while(ros::ok()) {   
        pedestrian_control.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}