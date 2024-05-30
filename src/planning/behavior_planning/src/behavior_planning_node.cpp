#include "ros/ros.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"

#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "detection_msgs/SensorFusion.h"

#include "pcl/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <string>

#define GLOBAL_PATH_FOLLOW    70
#define WAIT_MOVING_LEFT      71
#define MOVING_LEFT_LANE      72
#define OVERTAKE              73
#define WAIT_MOVING_RIGHT     74
#define MOVING_RIGHT_LANE     75


#define LANE_CHANGE_RIGHT_OFFSET    -2.5
#define LANE_CHANGE_LEFT_OFFSET     2.5
#define LANE_ERROR_DISTANCE_MAX     18
#define LANE_ERROR_DISTANCE_MIN     10

using namespace std;

class SetStateAndPub{

    private:
        ros::NodeHandle nh;
        ros::Subscriber vehicle_stat_sub;
        ros::Subscriber vehicle_odom_sub;
        ros::Subscriber front_obstacle_pos;
        ros::Subscriber right_obstacle_pos;

        carla_msgs::CarlaEgoVehicleStatus ego_vehicle_status;
        nav_msgs::Odometry ego_vehicle_odom;
        nav_msgs::Odometry prev_vehicle_odom;
        pcl::PointCloud<pcl::PointXYZ> right_obstacle;
        
        int state;
        string state_name;
        int ego_vehicle_lane_before;
        int ego_vehicle_lane_now = 3;
        int temp_state;
        
        double right_obstacle_x = -1;
        double right_obstacle_y = -1;
        double right_obstacle_z;
        double lane_change_thre ;

        double distance_with_front_obstacle = 9999.;
        double distance_with_right_obstacle = 9999.;

        string vehicle_class;
        
        bool local_planning;
        bool lane_change;
        bool vehicle_in_front;
        bool vehicle_in_left;
        bool vehicle_in_right;
        bool fast_vehicle_in_right;

    public:
        SetStateAndPub(){
            vehicle_stat_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 100, &SetStateAndPub::VehicleStatusCallback, this);
            vehicle_odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &SetStateAndPub::GNSSCallback, this);
            front_obstacle_pos = nh.subscribe("/fusion_info", 100, &SetStateAndPub::FObstaclePosCallback, this);
            right_obstacle_pos = nh.subscribe("/right_output", 100, &SetStateAndPub::RObstaclePosCallback, this);
            state = GLOBAL_PATH_FOLLOW;
            local_planning = false;
            vehicle_in_front = false;
            
            fast_vehicle_in_right = false;
            vehicle_in_left = false;
            vehicle_in_right = false;
            lane_change = false;
        };
        ~SetStateAndPub(){};

        void GNSSCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        void VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg);
        void FObstaclePosCallback(const detection_msgs::SensorFusion::ConstPtr& pos_msg);
        void RObstaclePosCallback(const sensor_msgs::PointCloud2::ConstPtr& pos_msg);
        void SetState();
        bool LaneChanged();
        void Print();
};

void SetStateAndPub::FObstaclePosCallback(const detection_msgs::SensorFusion::ConstPtr& pos_msg){

    vehicle_class = pos_msg->Class;
    distance_with_front_obstacle = pos_msg->distance;
    
 
    if(distance_with_front_obstacle == -1){
        vehicle_in_front = false;
    }                                 
    else{
        vehicle_in_front = true;
    }   
}

void SetStateAndPub::RObstaclePosCallback(const sensor_msgs::PointCloud2::ConstPtr& pos_msg){
    pcl::fromROSMsg(*pos_msg, right_obstacle);

    for(int index = 0 ; index < right_obstacle.size() ; index++){

        if(!right_obstacle.empty()){
            if(right_obstacle[index].y < -1.3){

                distance_with_right_obstacle = sqrt(pow(right_obstacle[index].x, 2)
                                                + pow(right_obstacle[index].y, 2));
                right_obstacle_x = right_obstacle[index].x;
                right_obstacle_y = right_obstacle[index].y;
                right_obstacle_z = right_obstacle[index].z;
                
                vehicle_in_right = true;
                if(distance_with_right_obstacle > 5.0){
                       vehicle_in_right = false;
                }
            }
            else{
                vehicle_in_right = false;
            }
        }
        else{
            vehicle_in_right = false;
        }

    }
                   

}


void SetStateAndPub::GNSSCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){

    ego_vehicle_odom.pose.pose.position.x = odom_msg->pose.pose.position.x;
    ego_vehicle_odom.pose.pose.position.y = odom_msg->pose.pose.position.y;
    ego_vehicle_odom.pose.pose.position.z = odom_msg->pose.pose.position.z;

    if(state_name == "WAIT_MOVING_LEFT"){
        prev_vehicle_odom = ego_vehicle_odom;
    }
    if(state_name == "MOVING_LEFT_LANE"){
        lane_change_thre = (prev_vehicle_odom.pose.pose.position.y - ego_vehicle_odom.pose.pose.position.y);
        if(lane_change_thre > LANE_CHANGE_LEFT_OFFSET){
            lane_change = true;
        }
        
    }
    if(state_name == "WAIT_MOVING_RIGHT"){
        prev_vehicle_odom = ego_vehicle_odom;
    }
    if(state_name == "MOVING_RIGHT_LANE"){
        lane_change_thre = (prev_vehicle_odom.pose.pose.position.y - ego_vehicle_odom.pose.pose.position.y);
        if(lane_change_thre < LANE_CHANGE_RIGHT_OFFSET){
            lane_change = true;
        }
    }
         
                               
                               

}

void SetStateAndPub::VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg){

    ego_vehicle_status.velocity = msg->velocity;
    ego_vehicle_status.acceleration = msg->acceleration;
    ego_vehicle_status.orientation = msg->orientation;
    ego_vehicle_status.control = msg->control;

}

void SetStateAndPub::SetState() {
    

    switch (state) {
        case GLOBAL_PATH_FOLLOW : 
            local_planning = false;
            state_name = "GLOBAL_PATH_FOLLOW";

            if (vehicle_class == "vehicle" && (distance_with_front_obstacle <= LANE_ERROR_DISTANCE_MAX && distance_with_front_obstacle >= LANE_ERROR_DISTANCE_MIN)){
                state = WAIT_MOVING_LEFT;
                temp_state = state;
            }
            break;
    
        case WAIT_MOVING_LEFT :
            local_planning = true;
            state_name = "WAIT_MOVING_LEFT";
            
            if ((vehicle_in_left == false) && (ego_vehicle_lane_now != 1)) {
                state = MOVING_LEFT_LANE; 
                temp_state = state;
            }
            break;


        case MOVING_LEFT_LANE :
            local_planning = true;
            state_name = "MOVING_LEFT_LANE";
            // if (lane_change == true) {
            //     state = WAIT_OVERTAKE;
            //     temp_state = state;
            // }
            if (vehicle_in_front == false && lane_change == true) {
                state = OVERTAKE;
                temp_state = state;
                lane_change = false;
                lane_change_thre = 0;
            }
            break;

        case OVERTAKE :
            local_planning = true;
            state_name = "OVERTAKE";
            if ((vehicle_in_right == false)) {
                state = WAIT_MOVING_RIGHT;
                temp_state = state; 
            }
            break;
            
          case WAIT_MOVING_RIGHT :
            local_planning = true;
            state_name = "WAIT_MOVING_RIGHT";
            
            if ((vehicle_in_right == false)) {
                state = MOVING_RIGHT_LANE; 
                temp_state = state;
            }
            break;



        case MOVING_RIGHT_LANE :
            local_planning = true;
            state_name = "MOVING_RIGHT_LANE";
            if (lane_change == true) {
                    state = GLOBAL_PATH_FOLLOW;
                    temp_state = state;
                    lane_change = false;
                }
            else {
                state = OVERTAKE;
                temp_state = state;
            }
            
            break;
    }
    // }

}

bool SetStateAndPub::LaneChanged() {
    if (ego_vehicle_lane_before != ego_vehicle_lane_now) {
        ego_vehicle_lane_before = ego_vehicle_lane_now; // update lane
        std::cout << "\nlane changed" << std::endl;
        return true;
    }
    else {
        ego_vehicle_lane_before = ego_vehicle_lane_now; // update lane
        return false;
    }
}

void SetStateAndPub::Print(){
    
    cout << "---------------" << endl;
    cout << "Vehicle X : " << ego_vehicle_odom.pose.pose.position.x << endl;
    cout << "Vehicle Y : " << ego_vehicle_odom.pose.pose.position.y << endl;
    cout << "---------------" << endl;
    cout << "Class :" << vehicle_class << endl;
    cout << "Distance :" << distance_with_front_obstacle << endl;
    cout << "State : " << state_name << endl;
    cout << "obstacle x : " << right_obstacle_x << endl;
    cout << "obstacle y : " << right_obstacle_y << endl;
    cout << "Distance with front Obstacle : " << distance_with_right_obstacle << "(m)" <<endl;
    cout << "lane_change : " << lane_change_thre << endl;
                
}

int main(int argc, char ** argv){
  
  ros::init(argc,argv,"behavior_planning_node");

  SetStateAndPub set;

  ros::Rate loop_rate(40);

  while(ros::ok()){
    
    set.SetState();
    set.Print();
    ros::spinOnce();
    loop_rate.sleep();
    
  }

}
