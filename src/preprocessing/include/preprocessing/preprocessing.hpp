#ifndef PREPROCESSING_H
#define PREPROCESSING_H

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


class Preprocessing{
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Publisher output_pub;
    ros::Publisher centroid_pub;
    ros::Publisher right_pub;

    pcl::PointCloud<pcl::PointXYZ> cloud;

public:
    Preprocessing();

    void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Voxel(pcl::PointCloud<pcl::PointXYZ> cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Ransac(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& filter_cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering( pcl::PointCloud<pcl::PointXYZ>::Ptr& inlierPoint_neg);
    void Object_detection();

}; 

#endif

