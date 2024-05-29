#include "preprocessing/right_preprocess.hpp"

RightPreprocess::RightPreprocess(){
    right_lidar_sub = nh.subscribe("/carla/ego_vehicle/lidar_right",10, &RightPreprocess::right_cb, this);
    right_output_pub = nh.advertise<sensor_msgs::PointCloud2>("right_output",1);
}

RightPreprocess::~RightPreprocess(){};

void RightPreprocess::right_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    pcl::fromROSMsg(*msg,raw_cloud);
}


PointCloud::Ptr RightPreprocess::Voxel(PointCloud raw_cloud){

    for(unsigned int i = 0; i < raw_cloud.points.size();i++){
    if(raw_cloud.points[i].y > 0 || raw_cloud.points[i].y < -3.75 || fabs(raw_cloud.points[i].x) > 20 || raw_cloud.points[i].z < -1){
      raw_cloud.points[i].x = 0;
      raw_cloud.points[i].y = 0;
      raw_cloud.points[i].z = 0;
    }
    
  }
  
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  PointCloud::Ptr filter_cloud(new PointCloud);
  vg.setInputCloud(raw_cloud.makeShared()); //point cloud 객체에 shared_pointer 생성
  vg.setLeafSize(0.2,0.2,0.2);
  vg.filter(*filter_cloud);

//   sensor_msgs::PointCloud2 test_output;
//   pcl::toROSMsg(*filter_cloud,test_output);
//   right_output_pub.publish(test_output);
  
  return filter_cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> RightPreprocess::Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  
  int size = cloud ->size();
  std::vector<double> z(size);
  for(int i = 0;i < size; i++){
    z[i] = cloud ->points[i].z;
    cloud ->points[i].z = 0.0;
  }
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud); //kdtree 생성
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.6);
  ec.setMinClusterSize (15);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
  
  for(int i = 0; i < size ; i++){
    cloud -> points[i].z = z[i];
  }

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;

  int j = 0;
  pcl::PointCloud<pcl::PointXYZ> centroid_cloud;
  pcl::PointCloud<pcl::PointXYZ> right_cloud;

  for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    xyzi_cloud -> points.reserve(it->indices.size());
    double temp_x = 0;
    double temp_y = 0;
    double temp_z = 0;
    pcl::PointXYZ avg_point;

    for(auto pit = it -> indices.begin(); pit != it -> indices.end(); ++pit){
      pcl::PointXYZ pt = cloud->points[*pit];
      pcl::PointXYZI pt2;
      pt2.x = pt.x;
      pt2.y = pt.y;
      pt2.z = pt.z;

      temp_x += pt.x;
      temp_y += pt.y;
      temp_z += pt.z;

      pt2.intensity = (float)(j+1);
      TotalCloud.push_back(pt2);
      xyzi_cloud -> points.push_back(pt2);
    }
    
    j++;
    
    //클러스터링된 모든 물체의 평균점 1개
    avg_point.x = temp_x / it->indices.size();
    avg_point.y = temp_y / it->indices.size();
    avg_point.z = temp_z / it->indices.size();

    centroid_cloud.push_back(avg_point);
    clusters.push_back(xyzi_cloud);
  }
  sensor_msgs::PointCloud2 test_output;
  pcl::toROSMsg(centroid_cloud,test_output);
  test_output.header.frame_id = "ego_vehicle/lidar_right";
  right_output_pub.publish(test_output);

  return clusters;
}

void RightPreprocess::Run(){
  raw_cloud.header.frame_id = "ego_vehicle/lidar_right";

  std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
  // std::cout << "start_time: " << start_time << std::endl;

  PointCloud::Ptr voxel_cloud(new PointCloud);
  voxel_cloud = Voxel(raw_cloud);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  clusters = Clustering(voxel_cloud);

}


int main(int argc, char ** argv){
  
  ros::init(argc,argv,"rightpreprocess");
  RightPreprocess rightpreprocess;
  ros::Rate loop_rate(40);


  while(ros::ok()){

    rightpreprocess.Run();
    ros::spinOnce();
    loop_rate.sleep();
    
  }

}