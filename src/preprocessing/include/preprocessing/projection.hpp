#ifndef PROJECTION_HPP
#define PROJECTION_HPP


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cmath>

//pcl include
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//image include
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

//yolo include
#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>
#include <detection_msgs/SensorFusion.h>

// typedef pcl::PointCloud<pcl::PointXYZI> point;
typedef pcl::PointCloud<pcl::PointXYZ> point;

class Projection{
    private:
        ros::NodeHandle nh;

        ros::Subscriber clustering_sub;
        ros::Subscriber image_sub;
        ros::Subscriber boundingbox_sub;

        ros::Publisher projection_pub;
        ros::Publisher fusion_pub;
    
    public:
        Projection();
        ~Projection();

        void clustering_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
        void image_cb(const sensor_msgs::ImageConstPtr& msg);
        void boundingbox_cb(const detection_msgs::BoundingBoxesConstPtr& msg);

        void LidarProjection(point cluster_data);
        void ObjectFitting();

        void Publish();
        void Clear();
        void Run();

        Eigen::Matrix3f K; //intrinsic matrix
        int u, v;
        // std::vector<std::pair<cv::Point2i,int>> projection_point;
        std::vector<cv::Point2i> projection_point;
        // int cluster_total_point;
        // int prev_intensity;
        std::vector<std::pair<cv::Point2i,pcl::PointXYZ>> vec_uv;

        cv::Mat origin_img;
        std::vector<detection_msgs::BoundingBox> boundingbox_datas;

        detection_msgs::SensorFusion fusion_msg;

};


#endif