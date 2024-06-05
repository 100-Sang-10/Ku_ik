#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/LaneletMap.h>


#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

// we want assert statements to work in release mode
#undef NDEBUG

#define A       71
#define B       72
#define C       73
#define D       74

#define LEFT    11
#define CENTER  12
#define RIGHT   13


using namespace lanelet;
using namespace std;

struct Dst
{
  int number = 0;
  int order = 0;
  double sum_distance = 0;
  Lanelet lanelet;
  vector<BasicPoint2d> points;

};

class GlobalPlanning{

  private: 
    ros::NodeHandle nh;
    ros::Publisher left_marker_pub;
    ros::Publisher center_marker_pub;
    ros::Publisher right_marker_pub;
    ros::Publisher center_line_pub;
    ros::Subscriber vehicle_gnss_sub;

    visualization_msgs::MarkerArray center_array;
    LaneletMapPtr map;
    traffic_rules::TrafficRulesPtr trafficRules;
    routing::RoutingGraphUPtr graph;
    string a = "A";
    string b = "B";

    GPSPoint m_vehicle_gnss;
    BasicPoint2d m_utm_point;
    Lanelet m_spawn_lanelet; 
    Lanelet m_prev_from_lanelet;

    vector<double> m_paths_dist;
    bool m_gnss_bool;
    bool m_init_bool;

    Dst m_address1;
    Dst m_address2;
    Dst m_address3;

  

  public:
    GlobalPlanning(){
      
      center_marker_pub = nh.advertise<visualization_msgs::Marker>("center_marker", 10);
      // center_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("center_marker", 10);
      center_line_pub = nh.advertise<geometry_msgs::PoseArray>("center_line_point", 10);
      vehicle_gnss_sub = nh.subscribe("/carla/ego_vehicle/gnss", 100, &GlobalPlanning::GNSSCallback, this);
      m_gnss_bool = false;
      m_init_bool = false;
    

      map = load("/home/kichang/Ku_ik/src/planning/global_planning/map/Town05_Final.osm",  projection::UtmProjector(Origin({0, 0})));
      // map = load("/home/baek/git/Ku_ik/src/planning/global_planning/map/Town05_modify.osm",  projection::UtmProjector(Origin({0, 0})));
      // map = load("/home/baek/git/Ku_ik/src/planning/global_planning/map/Town05_Final.osm",  projection::UtmProjector(Origin({0, 0})));
      trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
      graph = routing::RoutingGraph::build(*map, *trafficRules);
    };
    ~GlobalPlanning(){};
    void GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);
    void FindSpawnPoint();
    void CalculatePath();
    void GetLanelet(Dst& to_address);
    bool GetDestination(Dst& m_address1, Dst& m_address2, Dst& m_address3);
    void CalculateShortPath(Dst& candidate_path,
                            Lanelet& spawn_lanelet, 
                            Lanelet& lanelet1, 
                            Lanelet& lanelet2, 
                            Lanelet& lanelet3);

    void CreateRoutingGraphs(Dst& candidate_path, Lanelet& from_lanelet, Lanelet& to_lanelet);
    void CompareAddresses(double total_dist);
    
    void VisualizeCenterLine(const vector<BasicPoint2d>& path_points);

    bool getGNSS();

};

void GlobalPlanning::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg){

  m_vehicle_gnss.lat = gnss_msg->latitude;
  m_vehicle_gnss.lon = gnss_msg->longitude;

  if(!m_init_bool){
    FindSpawnPoint();
    m_gnss_bool = true;
  }
  if(m_gnss_bool){
    CalculatePath();
  }
}

bool GlobalPlanning::getGNSS(){
  return m_gnss_bool;
}

void GlobalPlanning::FindSpawnPoint(){

  projection::UtmProjector projection(Origin({0, 0}));
  
  m_utm_point.x() = projection.forward(m_vehicle_gnss).x();
  m_utm_point.y() = projection.forward(m_vehicle_gnss).y();

  cout << "----------Start Point----------- " << endl;
  cout << "Spawn latitude : " << m_vehicle_gnss.lat << "\n" << 
          "Spawn longitude : " << m_vehicle_gnss.lon << "\n" << 
          "Spawn altitude : " << m_vehicle_gnss.ele << endl;

  cout << "X : " << m_utm_point.x() << ", Y : " << m_utm_point.y() << endl;

  //spawn 1(우측 하단).188.886138916, -91.5933456421 0.0 270
  //spawn 2(좌측 상단).-138.765396118 186.384170532 15.0 0
  Point2d point{utils::getId(), m_utm_point.x(), m_utm_point.y()}; 
  auto nearlane_start = geometry::findWithin2d(map->laneletLayer,point, (1.75, 1.75));
  m_spawn_lanelet  = map->laneletLayer.get(nearlane_start.operator[](0).second.id());
  cout << m_spawn_lanelet << endl;

  m_init_bool = true;

}
bool GlobalPlanning::GetDestination(Dst& m_address1, Dst& m_address2, Dst& m_address3){

  /*****************도착 포인트 정보 입력*******************/
  int to_address;
  string input;
  cout << " " << endl;
  cout << "----------Destination----------- \n"
            << " A : 71, B : 72, C : 73, D : 74 \n"
            << "Write Destination address number : ";
  getline(cin, input);
  stringstream ss(input);

  if (!(ss >> m_address1.number >> m_address2.number >> m_address3.number)) {
        cout << "잘못된 입력입니다. 세 개의 정수 값을 다시 입력하세요." << endl;
        return false;
  }
  if ( (m_address1.number < 71 || m_address1.number > 74) ||
      (m_address2.number < 71 || m_address2.number > 74) ||
      (m_address3.number < 71 || m_address3.number > 74)) {
      cout << "잘못된 입력입니다. 71에서 74 사이의 정수 값을 다시 입력하세요." << endl;
      return false;
  }

  return true;
 
}

void GlobalPlanning::CalculatePath(){
 
  while(!GetDestination(m_address1, m_address2, m_address3)){

  }
  cout << "입력한 값들 : " << m_address1.number << ", " << m_address2.number << ", " << m_address3.number << endl;
  GetLanelet(m_address1);
  GetLanelet(m_address2);
  GetLanelet(m_address3);
  // m_candidate_path1.points.clear();
  // m_candidate_path2.points.clear();
  // m_candidate_path3.points.clear();
  // m_candidate_path4.points.clear();
  // m_candidate_path5.points.clear();
  // m_candidate_path6.points.clear();

  Dst m_candidate_path1;
  Dst m_candidate_path2;
  Dst m_candidate_path3;
  Dst m_candidate_path4;
  Dst m_candidate_path5;
  Dst m_candidate_path6;
  m_paths_dist.clear();
  cout << " " << endl;
  cout <<"--------------------------------" << endl;
  CalculateShortPath(m_candidate_path1, m_spawn_lanelet, m_address1.lanelet, m_address2.lanelet, m_address3.lanelet); // 1 - 2 - 3
  cout << "Candidate path 1 check" << endl;
  CalculateShortPath(m_candidate_path2, m_spawn_lanelet, m_address1.lanelet, m_address3.lanelet, m_address2.lanelet); // 1 - 3 - 2
  cout << "Candidate path 2 check" << endl;
  CalculateShortPath(m_candidate_path3, m_spawn_lanelet, m_address2.lanelet, m_address1.lanelet, m_address3.lanelet); // 2 - 1 - 3
  cout << "Candidate path 3 check" << endl;
  CalculateShortPath(m_candidate_path4, m_spawn_lanelet, m_address2.lanelet, m_address3.lanelet, m_address1.lanelet); // 2 - 3 - 1
  cout << "Candidate path 4 check" << endl;
  CalculateShortPath(m_candidate_path5, m_spawn_lanelet, m_address3.lanelet, m_address1.lanelet, m_address2.lanelet); // 3 - 1 - 2
  cout << "Candidate path 5 check" << endl;
  CalculateShortPath(m_candidate_path6, m_spawn_lanelet, m_address3.lanelet, m_address2.lanelet, m_address1.lanelet); // 3 - 2 - 1
  cout << "Candidate path 6 check" << endl;

  double min_value = numeric_limits<double>::max();
  int path_number = 0;
  
  for(int i = 0 ; i < m_paths_dist.size() ; i++){
    cout << "Candidate Path [" << i+1 << "] : "<< m_paths_dist[i] << endl;
    if(m_paths_dist[i] < min_value){
      min_value = m_paths_dist[i];
      path_number = i + 1;
    }
  } 
  m_paths_dist.clear();
  cout <<"  " << endl;
  switch (path_number)
  {
    case 1 :
      cout << "----------Route Direction----------- " << endl;
      cout <<  "1 - 2 - 3" << endl;
      VisualizeCenterLine(m_candidate_path1.points);
      break;
    case 2 :
      cout << "----------Route Direction----------- " << endl;
      cout <<  "1 - 3 - 2" << endl;
      VisualizeCenterLine(m_candidate_path2.points);
      break;
    case 3 :
      cout << "----------Route Direction----------- " << endl;
      cout <<  "2 - 1 - 3" << endl;  
      VisualizeCenterLine(m_candidate_path3.points);
      break;
    case 4 :
      cout << "----------Route Direction----------- " << endl;
      cout <<  "2 - 3 - 1" << endl;  
      VisualizeCenterLine(m_candidate_path4.points);
      break;
    case 5 :
      cout << "----------Route Direction----------- " << endl;
      cout <<  "3 - 1 - 2" << endl;  
      VisualizeCenterLine(m_candidate_path5.points);
      break;
    case 6 :
      cout << "----------Route Direction----------- " << endl;
      cout <<  "3 - 2 - 1" << endl; 
      VisualizeCenterLine(m_candidate_path6.points);
      break;      
    default:
      break;

  }
  
}

void GlobalPlanning::GetLanelet(Dst& to_address){


  /*****************도착 포인트 정보 입력*******************/

  switch (to_address.number)
    {
    case A :
      to_address.lanelet = map->laneletLayer.get(649);  //A
      break;
    case B :
      to_address.lanelet = map->laneletLayer.get(21496); //B
      break;
    case C :
      to_address.lanelet = map->laneletLayer.get(299);  //C //C' = 5797
      break;
    case D :
      to_address.lanelet = map->laneletLayer.get(722);  //D
      break;      
    default:
      break;
  
  }
}

void GlobalPlanning::CalculateShortPath(Dst& candidate_path, 
                                        Lanelet& spawn_lanelet, 
                                        Lanelet& lanelet1, 
                                        Lanelet& lanelet2, 
                                        Lanelet& lanelet3){
  // m_prev_from_lanelet.setId(999999);
  CreateRoutingGraphs(candidate_path, spawn_lanelet, lanelet1);
  CreateRoutingGraphs(candidate_path, lanelet1, lanelet2);
  CreateRoutingGraphs(candidate_path, lanelet2, lanelet3);
  // cout << "Debugging" << endl;
  m_paths_dist.push_back(candidate_path.sum_distance);

}


void GlobalPlanning::CreateRoutingGraphs(Dst& candidate_path, Lanelet& from_lanelet, Lanelet& to_lanelet) {
  
  Optional<routing::LaneletPath> shortestPath = graph->shortestPath(from_lanelet, to_lanelet);
  if (shortestPath) {
    
    // 최단 경로의 각 lanelet을 순회합니다.    
    BasicPoint2d last_point = m_utm_point;
    bool point_check;    
    double sum_dist = 0;
    
    for (const auto& lanelet : *shortestPath) {
      point_check = false;
      // 현재 lanelet의 중앙선을 가져옵니다
      auto centerline = lanelet.centerline();    

      BasicPoint2d prev_point;
      // 중앙선의 각 점을 벡터에 추가합니다
      // cout << "prev lanelet id :" <<m_prev_from_lanelet.id() << endl;
      // cout << "current lanelet id : " << lanelet.id() << endl;
      // if(m_prev_from_lanelet.id() != lanelet.id()){
        for (auto& point : centerline) {
          double distance_x = abs((point.x() - last_point.x()));
          double distance_y = abs((point.y() - last_point.y()));

          if( distance_x < 0.2 || distance_y < 0.2 ){
            point_check = true;
        
          }
          if(point_check){
              candidate_path.points.push_back(point.basicPoint2d());
              double distance = sqrt(pow((point.x()- prev_point.x()), 2) + pow((point.y()- prev_point.y()), 2));
              sum_dist += distance;
              prev_point = point.basicPoint2d();
          }
          
        }
        last_point = centerline[centerline.size()-1].basicPoint2d(); // 각 레인렛의 마지막 점 저장

      // }
      // else{
        // cout << "Same Lanelet : " << m_prev_from_lanelet.id() <<"," << lanelet.id() << endl;
      // }
        
    }
    candidate_path.sum_distance += sum_dist;
  }
  else{
    cout << "No Path" << endl;
  }
 
  m_prev_from_lanelet = to_lanelet;

  // Optional<routing::Route> route = graph->getRoute(m_address2.lanelet, m_address3.lanelet);
  // if (route) {
  //     LaneletSubmapConstPtr routeMap = route->laneletSubmap();
      
  //     write("/home/eonsoo/OSM File/TOWN5_4.osm", *routeMap->laneletMap(), Origin({0, 0}));
  // }`
  // else{
  //   cout << "No Route" << endl;
  // }

}

void GlobalPlanning::VisualizeCenterLine(const vector<BasicPoint2d>& path_points){

     // 메시지 초기화
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // RViz에서 표시할 프레임
    marker.header.stamp = ros::Time::now();
    marker.ns = "centerline";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5; // 선의 두께

    // 선 색상 설정 (R, G, B, A)
    marker.color.r = 1.0;
    marker.color.g = 0.3;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0 ;
    // 중앙선의 점들을 메시지에 추가
    geometry_msgs::PoseArray center_points;
    int count = 0;
    for (const auto& point : path_points) {
        geometry_msgs::Point p;
        geometry_msgs::Pose c;
        
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0; // 2D 중앙선이므로 z 값은 0
        
        c.position = p;
        
        center_points.poses.push_back(c);
        marker.points.push_back(p);
        count ++;
    }
    
    center_line_pub.publish(center_points);

    // 메시지 게시
    center_marker_pub.publish(marker);
}

// void GlobalPlanning::VisualizeCenterLine(const vector<BasicPoint2d>& path_points){

//      // 메시지 초기화
//     center_array.markers.clear();
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "map"; // RViz에서 표시할 프레임
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "centerline";
//     marker.type = visualization_msgs::Marker::CUBE;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.scale.x = 0.2; // 선의 두께
//     marker.scale.y = 0.2; // 선의 두께
//     marker.scale.z = 0.2; // 선의 두께
//     // 선 색상 설정 (R, G, B, A)
//     marker.color.r = 0.5;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.color.a = 1.0;
//     marker.pose.orientation.w = 1.0 ;
//     // 중앙선의 점들을 메시지에 추가
//     int count = 0;
//     geometry_msgs::PoseArray center_points;

//     for (const auto& point : path_points) {
//         geometry_msgs::Point p;
//         geometry_msgs::Pose c;
//         marker.id = count;
//         p.x = point.x();
//         p.y = point.y();
//         p.z = 0.0; // 2D 중앙선이므로 z 값은 0
        
//         c.position = p;
//         marker.pose.position = p;
        
//         center_points.poses.push_back(c);
//         center_array.markers.push_back(marker);
//         count++;
//     }

//     //Control pub
//     center_line_pub.publish(center_points);

//     // 메시지 게시
//     center_marker_pub.publish(center_array);
// }

int main(int argc, char **argv){

  ros::init(argc, argv, "Global_planning_node");

  GlobalPlanning gp;

  ros::spin();

  return 0;
}
