#include "preprocessing/preprocessing.hpp"

#define LOOP_RATE_HZ    40

Preprocessing::Preprocessing(){

    lidar_sub = nh.subscribe("/carla/ego_vehicle/lidar",1, &Preprocessing::pointcloud_cb, this);
    output_pub = nh.advertise<sensor_msgs::PointCloud2>("clustering",1);
    centroid_pub = nh.advertise<sensor_msgs::PointCloud2>("centroid",1);
    right_pub = nh.advertise<sensor_msgs::PointCloud2>("right_centroid",1);
}

//--------------------------------Downsampling(Voxelization)-----------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocessing::Voxel(pcl::PointCloud<pcl::PointXYZ> cloud){
  for(unsigned int i = 0; i < cloud.points.size();i++){
    if(cloud.points[i].z > -0.38 || cloud.points[i].y > 5 || cloud.points[i].y < -5){
      cloud.points[i].x = 0;
      cloud.points[i].y = 0;
      cloud.points[i].z = 0;
    }
    
    if(cloud.points[i].z < -2){
      // cloud.points[i].x = 0;
      // cloud.points[i].y = 0;
      cloud.points[i].z = 0;
    }
    
  }
  
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud.makeShared()); //point cloud 객체에 shared_pointer 생성
  vg.setLeafSize(0.2,0.2,0.2);
  vg.filter(*filter_cloud);
  // std::cout << "downsampled cloud size: " << filter_cloud ->points.size() << std::endl;
  return filter_cloud;
}
// ------------------------------------------------Ransac---------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocessing::Ransac(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& filter_cloud){

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZ>());
  
  // SACSegmentation 을 위해서 seg 를 만들고 방법과 모델과 기준을 정함
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE); // 적용 모델
  seg.setMethodType(pcl::SAC_RANSAC); // 적용 방법
  seg.setMaxIterations(1000); //최대 실행 수

  seg.setDistanceThreshold(0.1); //inlier로 처리할 거리 정보
  seg.setInputCloud(filter_cloud); //setinputcloud에는 포인터 넣는 거인듯?? [setInputCloud (const PointCloudConstPtr &cloud)]
  seg.segment(*inliers,*coefficients);

  // input cloud에서 평면 inlier를 추출함.
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(filter_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);           //바닥제거하기 위해 inlier를 없앤다
  extract.filter(*inlierPoints_neg);
  return inlierPoints_neg;
}
// -----------------------------------------------------Clustering---------------------------------------------------------
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Preprocessing::Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& inlierPoints_neg){
  
  int size = inlierPoints_neg ->size();
  std::vector<double> z(size);
  for(int i = 0;i < size; i++){
    z[i] = inlierPoints_neg ->points[i].z;
    inlierPoints_neg ->points[i].z = 0.0;
  }
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (inlierPoints_neg); //kdtree 생성
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.6);
  ec.setMinClusterSize (15);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (inlierPoints_neg);
  ec.extract (cluster_indices);
  
  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
  
  for(int i = 0; i < size ; i++){
    inlierPoints_neg -> points[i].z = z[i];
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
    double right_x,right_y,right_z = 0;
    int right_count = 0;
    pcl::PointXYZ avg_point;
    pcl::PointXYZ right_avg_point;

    for(auto pit = it -> indices.begin(); pit != it -> indices.end(); ++pit){
      pcl::PointXYZ pt = inlierPoints_neg->points[*pit];
      pcl::PointXYZI pt2;
      pt2.x = pt.x;
      pt2.y = pt.y;
      pt2.z = pt.z;

      if(pt.y < 0){
        right_x += pt.x;
        right_y += pt.y;
        right_z += pt.z;
        right_count++;
      }

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

    //자차 기준 오른쪽에 있는 물체들의 평균점 1개
    right_avg_point.x = right_x / right_count;
    right_avg_point.y = right_y / right_count;
    right_avg_point.z = right_z / right_count;

    centroid_cloud.push_back(avg_point);
    clusters.push_back(xyzi_cloud);
    right_cloud.push_back(right_avg_point);
  }

  //클러스터링된 물체의 중점만 보내기
  sensor_msgs::PointCloud2 centroid_output;
  pcl::toROSMsg(centroid_cloud,centroid_output);
  centroid_output.header.frame_id = "ego_vehicle/lidar";
  centroid_pub.publish(centroid_output);

  //클러스터링된 거 다 보내기
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "ego_vehicle/lidar";
  output_pub.publish(output);

  //오른쪽 물체의 중점 보내기
  sensor_msgs::PointCloud2 right_output;
  pcl::toROSMsg(right_cloud, right_output);
  right_output.header.frame_id = "ego_vehicle/lidar";
  right_pub.publish(right_output);

  return clusters;
}


void Preprocessing::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){  
  pcl::fromROSMsg(*pcl_msg, cloud);
}

void Preprocessing::Object_detection(){
  cloud.header.frame_id = "ego_vehicle/lidar";

  pcl::PointCloud<pcl::PointXYZ>::Ptr center_voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  center_voxel_cloud = Voxel(cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr center_ransac_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  center_ransac_cloud = Ransac(center_voxel_cloud);
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  // filter_cloud = Voxel(total_cloud);
  // inlierPoints_neg = Ransac(filter_cloud);
  clusters = Clustering(center_ransac_cloud);
}

int main(int argc, char ** argv){
  
  ros::init(argc,argv,"preprocessing");
  Preprocessing project;
  ros::Rate loop_rate(LOOP_RATE_HZ);

  while(ros::ok()){
    
    project.Object_detection();

    ros::spinOnce();
    loop_rate.sleep();
    
  }

}


