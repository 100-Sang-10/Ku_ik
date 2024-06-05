#ifndef RIGHT_PREPROCESS_H
#define RIGHT_PREPROCESS_H

#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <pcl/common/centroid.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/common/transforms.h>
#include <chrono>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class RightPreprocess{
    ros::NodeHandle nh;
    ros::Subscriber right_lidar_sub;
    ros::Publisher right_output_pub;

    PointCloud raw_cloud;
    PointCloud object_cloud;

public:
    RightPreprocess();
    ~RightPreprocess();

    void right_cb(const sensor_msgs::PointCloud2ConstPtr msg);
    void Run();
    PointCloud::Ptr Voxel(PointCloud cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

};



#endif