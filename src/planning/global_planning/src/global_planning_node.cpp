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
    bool m_gnss_bool;
    bool m_init_bool;

    Dst m_address1;
    Dst m_address2;
    Dst m_address3;
    int add = 71;

  public:
    GlobalPlanning(){
      left_marker_pub = nh.advertise<visualization_msgs::Marker>("left_marker", 10);
      center_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("center_marker", 10);
      right_marker_pub = nh.advertise<visualization_msgs::Marker>("right_marker", 10);
      center_line_pub = nh.advertise<geometry_msgs::PoseArray>("center_line_point", 10);
      vehicle_gnss_sub = nh.subscribe("/carla/ego_vehicle/gnss", 100, &GlobalPlanning::GNSSCallback, this);
      m_gnss_bool = false;
      m_init_bool = false;

      map = load("/home/eonsoo/OSM File/Town05_modify.osm",  projection::UtmProjector(Origin({0, 0})));
      trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
      graph = routing::RoutingGraph::build(*map, *trafficRules);
    };
    ~GlobalPlanning(){};
    double CreateRoutingGraphs(Lanelet from_lanelet, Dst& to_address);
    bool GetDestination(Dst& m_address1, Dst& m_address2, Dst& m_address3);
    void CalcutatePath();
    void FindSpawnPoint();
    void CompareAddresses(std::map<string, double>& addresses);
    bool getGNSS();
    void GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);
    void VisualizeLeftLine(const vector<BasicPoint2d>& leftlinePoints);
    void VisualizeCenterLine(const vector<BasicPoint2d>& centerlinePoints);
    void VisualizeRightLine(const vector<BasicPoint2d>& rightlinePoints);

};

void GlobalPlanning::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg){

  m_vehicle_gnss.lat = gnss_msg->latitude;
  m_vehicle_gnss.lon = gnss_msg->longitude;

  if(!m_init_bool){
    FindSpawnPoint();
    m_gnss_bool = true;
  }
  if(m_gnss_bool){
    CalcutatePath();
   
  }
}

bool GlobalPlanning::getGNSS(){
  return m_gnss_bool;
}

bool GlobalPlanning::GetDestination(Dst& m_address1, Dst& m_address2, Dst& m_address3){

  /*****************도착 포인트 정보 입력*******************/
  int to_address;
  string input;
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

void GlobalPlanning::CompareAddresses(std::map<string, double>& addresses) {
  string minKey;
  double minValue = numeric_limits<double>::max(); // 최솟값 초기화

  // 딕셔너리 순회
  for (const auto& pair : addresses) {
      if (pair.second < minValue) {
          minValue = pair.second;
          minKey = pair.first;
      }
  }

  // 가장 작은 값을 가진 키를 딕셔너리에서 삭제
  addresses.erase(minKey);
}

void GlobalPlanning::CalcutatePath(){
 
  while(!GetDestination(m_address1, m_address2, m_address3)){

  }

  cout << "입력한 값들 : " << m_address1.number << ", " << m_address2.number << ", " << m_address3.number << endl;

  std::map<string, double> addresses = {{"m_address1", CreateRoutingGraphs(m_spawn_lanelet, m_address1)}, {"m_address2", CreateRoutingGraphs(m_spawn_lanelet, m_address2)}, {"m_address3", CreateRoutingGraphs(m_spawn_lanelet, m_address3)}};
  cout << "address1 : " <<  m_address1.sum_distance << endl; 
  cout << "address2 : " <<  m_address2.sum_distance << endl;
  cout << "address3 : " <<  m_address3.sum_distance << endl;

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

  //spawn 1(우측 하단).188.886138916, -91.5933456421 270
  //spawn 2(좌측 상단).-138.765396118 186.384170532 0
  Point2d point{utils::getId(), m_utm_point.x(), m_utm_point.y()}; 
  auto nearlane_start = geometry::findWithin2d(map->laneletLayer,point, (1.75, 1.75));
  m_spawn_lanelet  = map->laneletLayer.get(nearlane_start.operator[](0).second.id());
  cout << m_spawn_lanelet << endl;

  m_init_bool = true;

}

double GlobalPlanning::CreateRoutingGraphs(Lanelet from_lanelet, Dst& to_address) {
 
  /*****************도착 포인트 정보 입력*******************/

  Lanelet toLanelet;
  switch (to_address.number)
    {
    case A :
      toLanelet = map->laneletLayer.get(21496);    
      break;
    case B :
      toLanelet = map->laneletLayer.get(5797);  
      break;
    case C :
      toLanelet = map->laneletLayer.get(649);    
      break;
    case D :
      toLanelet = map->laneletLayer.get(722);  
      break;      
    default:
      break;
  
  }

  Optional<routing::LaneletPath> shortestPath = graph->shortestPath(from_lanelet, toLanelet);

  vector<BasicPoint2d> centerlinePoints;
  vector<BasicPoint2d> leftlinePoints;
  vector<BasicPoint2d> rightlinePoints;
  
  LaneletMap laneletMap;

  if (shortestPath) {
    centerlinePoints.clear(); // 새로운 점들을 추가하기 전에 기존의 점들을 초기화합니다.
    // 최단 경로의 각 lanelet을 순회합니다.    
    BasicPoint2d last_point = m_utm_point;
    bool point_check;    
    double sum_dist = 0;

    for (const auto& lanelet : *shortestPath) {
        point_check = false;
        // 현재 lanelet의 중앙선을 가져옵니다
        auto centerline = lanelet.centerline();    
        auto leftline = lanelet.leftBound();
        auto rightline = lanelet.rightBound();
        BasicPoint2d prev_point;
        // 중앙선의 각 점을 벡터에 추가합니다
        
        for (auto& point : centerline) {
          double distance_x = abs((point.x() - last_point.x()));
          double distance_y = abs((point.y() - last_point.y()));

          if( distance_x < 0.2 || distance_y < 0.2 ){
            point_check = true;
        
          }
          if(point_check){
              centerlinePoints.push_back(point.basicPoint2d());
              double distance = sqrt(pow((point.x()- prev_point.x()), 2) + pow((point.y()- prev_point.y()), 2));
              sum_dist += distance;
              prev_point = point.basicPoint2d();
          }
          
        }
        last_point = centerline[centerline.size()-1].basicPoint2d(); // 각 레인렛의 마지막 점 저장
        for (const auto& point : rightline) {
          rightlinePoints.push_back(point.basicPoint2d());
        }
        for (const auto& point : leftline) {
          leftlinePoints.push_back(point.basicPoint2d());
        }
      }
      to_address.sum_distance = sum_dist;
      
  }
  else{
    cout << "No Path" << endl;
  }
  
  to_address.points = centerlinePoints;
  // VisualizeLeftLine(leftlinePoints);
  // VisualizeRightLine(rightlinePoints);
  // VisualizeCenterLine(centerlinePoints);
  
  // Optional<routing::Route> route = graph->getRoute(from_lanelet, toLanelet);
  // if (route) {
  //     LaneletSubmapConstPtr routeMap = route->laneletSubmap();
      
  //     write("/home/eonsoo/OSM File/TOWN5_4.osm", *routeMap->laneletMap(), Origin({0, 0}));
  // }
  // else{
  //   cout << "No Route" << endl;
  // }

}

void GlobalPlanning::VisualizeCenterLine(const vector<BasicPoint2d>& centerlinePoints){

     // 메시지 초기화
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // RViz에서 표시할 프레임
    marker.header.stamp = ros::Time::now();
    marker.ns = "centerline";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // 선의 두께
    marker.scale.y = 0.1; // 선의 두께
    marker.scale.z = 0.1; // 선의 두께
    // 선 색상 설정 (R, G, B, A)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0 ;
    // 중앙선의 점들을 메시지에 추가
    int count = 0;
    geometry_msgs::PoseArray center_points;

    for (const auto& point : centerlinePoints) {
        geometry_msgs::Point p;
        geometry_msgs::Pose c;
        marker.id = count;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0; // 2D 중앙선이므로 z 값은 0
        
        c.position = p;
        marker.pose.position = p;
        
        center_points.poses.push_back(c);
        center_array.markers.push_back(marker);
        count++;
    }
    
    center_line_pub.publish(center_points);

    // 메시지 게시
    center_marker_pub.publish(center_array);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "Global_planning_node");

  GlobalPlanning gp;

  ros::spin();

  return 0;
}

// void GlobalPlanning::VisualizeLeftLine(const vector<BasicPoint2d>& leftlinePoints){

//     // 메시지 초기화
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "map"; // RViz에서 표시할 프레임
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "leftline";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::LINE_STRIP;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.scale.x = 0.1; // 선의 두께
//     marker.pose.orientation.w = 1.0 ;
//     // 선 색상 설정 (R, G, B, A)
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.color.a = 1.0;

//     // 중앙선의 점들을 메시지에 추가
//     for (const auto& point : leftlinePoints) {
//         geometry_msgs::Point p;
//         p.x = point.x();
//         p.y = point.y();
//         p.z = 0.0; // 2D 중앙선이므로 z 값은 0
//         marker.points.push_back(p);
//     }

//     // 메시지 게시
//     left_marker_pub.publish(marker);
// }


// void GlobalPlanning::VisualizeRightLine(const vector<BasicPoint2d>& rightlinePoints){

//     // 메시지 초기화
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "map"; // RViz에서 표시할 프레임
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "rightline";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::LINE_STRIP;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.scale.x = 0.1; // 선의 두께
//     marker.pose.orientation.w = 1.0 ;
//     // 선 색상 설정 (R, G, B, A)
//     marker.color.r = 0.1;
//     marker.color.g = 0.3;
//     marker.color.b = 0.7;
//     marker.color.a = 1.0;

//     // 중앙선의 점들을 메시지에 추가
//     for (const auto& point : rightlinePoints) {
//         geometry_msgs::Point p;
//         p.x = point.x();
//         p.y = point.y();
//         p.z = 0.0; // 2D 중앙선이므로 z 값은 0
//         marker.points.push_back(p);
//     }

//     // 메시지 게시
//     right_marker_pub.publish(marker);
// }


