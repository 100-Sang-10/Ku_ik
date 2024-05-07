#include "preprocessing/projection.hpp"


Projection::Projection(){
    // clustering_sub = nh.subscribe("/clustering",10, &Projection::clustering_cb, this);
    clustering_sub = nh.subscribe("centroid",10, &Projection::clustering_cb, this);
    // projection_pub = nh.advertise<sensor_msgs::Image>("projection", 10);
    // image_sub = nh.subscribe("/carla/ego_vehicle/rgb_front/image", 10, &Projection::image_cb, this);
    image_sub = nh.subscribe("/yolov5/image_out", 10, &Projection::image_cb, this);
    boundingbox_sub = nh.subscribe("/yolov5/detections", 10 , &Projection::boundingbox_cb, this);
    fusion_pub = nh.advertise<detection_msgs::SensorFusion>("/fusion_info", 10);
    
    // carla/ego_vehicle/rgb_front/cameraInfo에서 intrinsic 얻어냄
    K << 400.00000000000006, 0.0, 400.0,
         0.0, 400.00000000000006, 300.0,
         0.0, 0.0, 1.0;

    u = 0;
    v = 0;
}

Projection::~Projection(){}

void Projection::image_cb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      origin_img = cv_ptr->image; // 800 * 600
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

void Projection::boundingbox_cb(const detection_msgs::BoundingBoxesConstPtr& msg){
    boundingbox_datas = msg->bounding_boxes;
}

void Projection::clustering_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    point cluster_data;
    pcl::fromROSMsg(*msg, cluster_data);
    LidarProjection(cluster_data);
}

void Projection::ObjectFitting(){
    std::vector<detection_msgs::BoundingBox>::iterator itr;
    for(itr = boundingbox_datas.begin(); itr != boundingbox_datas.end(); ++itr){
        detection_msgs::BoundingBox temp_boundingbox;
        temp_boundingbox = *itr;
        std::string temp_name = temp_boundingbox.Class;

        int temp_xmin = temp_boundingbox.xmin;
        int temp_xmax = temp_boundingbox.xmax;
        int temp_ymin = temp_boundingbox.ymin;
        int temp_ymax = temp_boundingbox.ymax;

        for(auto itr = vec_uv.begin(); itr != vec_uv.end(); ++itr){
            std::pair<cv::Point2i,pcl::PointXYZ> temp_point;
            temp_point = *itr;

            cv::Point2i uv_point = temp_point.first;
            pcl::PointXYZ xyz_point = temp_point.second;

            if(temp_xmin <= uv_point.x && uv_point.x <= temp_xmax && temp_ymin <= uv_point.y && uv_point.y <= temp_ymax){
              double distance = sqrt(pow(xyz_point.x,2) + pow(xyz_point.y,2) + pow(xyz_point.z,2));
              fusion_msg.distance = distance;
              fusion_msg.Class = temp_name;
              std::cout << "fusion_msg.distance: " << fusion_msg.distance << std::endl;
              std::cout << "fusion_msg.Class: " << fusion_msg.Class << std::endl;
            }
        }

        // 클러스터링된 모든 점을 이용해서 하려고 시도해본 거
        /*
        int point_number = 0;
        int cluster_total_point = 0;
        int prev_intensity = 1;

        for(auto itr = projection_point.begin(); itr != projection_point.end(); ++itr){
          std::pair<cv::Point2i, int> temp_projection_point;
          temp_projection_point = *itr;
          cv::Point2i uv_point = temp_projection_point.first;
          int intensity = temp_projection_point.second;

          if((intensity == prev_intensity) && (intensity > 0)){
            cluster_total_point++;
          
            if(temp_xmin <= uv_point.x && uv_point.x <= temp_xmax && temp_ymin <= uv_point.y && uv_point.y <= temp_ymax){
                point_number++; 
            }
            prev_intensity = intensity;
          }
        }

        //threshold 설정
        if(point_number > cluster_total_point * 0.8){
          
        }
        */
    }
    
}

void Projection::LidarProjection(point cluster_data){
    for(auto i = 0; i < cluster_data.size(); i++){

      pcl::PointXYZ raw_point = cluster_data[i]; // 중심점 거리 저장

      if(cluster_data[i].x < 0){
        continue;
      }
      else{
        //Extrinsic calibration
        double extrinsic_x = -cluster_data[i].y;
        double extrinsic_y = -cluster_data[i].z- 0.5;
        double extrinsic_z = cluster_data[i].x;
        // int intensity = cluster_data[i].intensity;

        Eigen::Vector3f world_to_camera;
        world_to_camera(0) = extrinsic_x;
        world_to_camera(1) = extrinsic_y;
        world_to_camera(2) = extrinsic_z;

        //Intrinsic calibration
        Eigen::Vector3f image_coord  = K * world_to_camera;
        //image는 (x,y,1) 형태이기 때문에 z값으로 나눠줌
        u = int(image_coord(0) / image_coord(2));
        v = int(image_coord(1) / image_coord(2));

        cv::Point2i point;
        point.x = u;
        point.y = v;
        // std::cout << "point: " << point << std::endl;
        projection_point.push_back(point);
        // std::cout << "point's intensity: " << intensity << std::endl;

        // std::pair<cv::Point2i, int> label_projection; //pair 처음엔 projection 시킨 점의 좌표, 2번째엔 물체를 구분하기 위한 intensity 저장
        // label_projection.first = point;
        // label_projection.second = intensity;
        
        std::pair<cv::Point2i,pcl::PointXYZ> uv_point;
        uv_point.first = point;
        uv_point.second = raw_point;

        if((u>=0) && (u < origin_img.cols) && (v>=0) && (v < origin_img.rows)) {
            cv::circle(origin_img, cv::Point(u, v), 3, cv::Scalar(255, 0, 0),-1);
        }

        vec_uv.push_back(uv_point);
        // projection_point.push_back(label_projection);
      }
    }

    if(origin_img.cols > 0 && origin_img.rows > 0){
      cv::imshow("projection_img", origin_img);
      cv::waitKey(1);
    }

}

void Projection::Clear(){
  projection_point.clear();
  vec_uv.clear();
}

void Projection::Publish(){
  fusion_pub.publish(fusion_msg);
}

void Projection::Run(){
  ObjectFitting();
  Publish();
  Clear();
}

int main(int argc, char ** argv){
  
  ros::init(argc,argv,"projection");
  Projection projection;
  ros::Rate loop_rate(10);

  while(ros::ok()){

    ros::spinOnce();
    projection.Run();
    loop_rate.sleep();
    
  }

}