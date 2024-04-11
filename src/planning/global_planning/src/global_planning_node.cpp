#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <fstream>


// we want assert statements to work in release mode
#undef NDEBUG
#define A       71
#define B       72
#define LEFT    11
#define CENTER  12
#define RIGHT   13


using namespace lanelet;
using namespace std;

class GlobalPlanning{

  private: 
    ros::NodeHandle nh;
    ros::Publisher left_marker_pub;
    ros::Publisher center_marker_pub;
    ros::Publisher right_marker_pub;
    ros::Publisher center_line_pub;

    LaneletMapPtr map;
    traffic_rules::TrafficRulesPtr trafficRules;
    routing::RoutingGraphUPtr graph;
    string a = "A";
    string b = "B";

  public:
    GlobalPlanning(){
      left_marker_pub = nh.advertise<visualization_msgs::Marker>("left_marker", 10);
      center_marker_pub = nh.advertise<visualization_msgs::Marker>("center_marker", 10);
      right_marker_pub = nh.advertise<visualization_msgs::Marker>("right_marker", 10);
      center_line_pub = nh.advertise<geometry_msgs::PoseArray>("center_line_point", 10);

      map = load("/home/eonsoo/Ku_ik/src/planning/global_planning/src/Town05.osm",  projection::UtmProjector(Origin({0, 0})));
      trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
      graph = routing::RoutingGraph::build(*map, *trafficRules);
    };
    ~GlobalPlanning(){};
    std::vector<BasicPoint2d> CreateRoutingGraphs();
    
    void VisualizeLeftLine(const std::vector<BasicPoint2d>& leftlinePoints);
    void VisualizeCenterLine(const std::vector<BasicPoint2d>& centerlinePoints);
    void VisualizeRightLine(const std::vector<BasicPoint2d>& rightlinePoints);

};


std::vector<BasicPoint2d> GlobalPlanning::CreateRoutingGraphs() {
  
  projection::UtmProjector projection(Origin({0, 0}));
  GPSPoint gps;

  /*****************시작 포인트 정보 입력*******************/

  int from_address;
  float from_lat;
  float from_lon;
  float from_utm_x, from_utm_y;
  std::cout << "----------Address----------- \n"
            << " A : 71, B : 72 \n"
            << "Write from address number : ";
  std::cin >> from_address;
  switch (from_address)
  {
  case A :
    gps.lat =-0.00072758317; // 0.00173466305149; //  
    gps.lon = 0.0020453343;//0.000622928226931; // 
    break;
  case B :
    gps.lat = 0.00007413843;//-0.00171244438533; //
    gps.lon = 0.00121679525; //0.000399127055247; //
  default:
    break;
  }
 
  std::cout << "From address : " << from_address;
  std::cout << "From latitude : " << gps.lat << ", From longitude : " << gps.lon << std::endl;
  
  from_utm_x = projection.forward(gps).x();
  from_utm_y = projection.forward(gps).y();
  std::cout << "from_utm_x : " << from_utm_x << ", from_utm_y : " << from_utm_y << std::endl;
  std::cout << "-----------------------" << std::endl;
  Lanelets from_lanelets = map->laneletLayer.nearest(BasicPoint2d(from_utm_x, from_utm_y), 1);
  
  /*****************도착 포인트 정보 입력*******************/
  int to_address;
  float to_lat;
  float to_lon; 
  double to_utm_x, to_utm_y;
  std::cout << "----------Address----------- \n"
            << " A : 71, B : 72 \n"
            << "Write from address number : ";
  std::cin >> to_address;

  switch (to_address)
  {
    case A :
      gps.lat =-0.00072758317; // 0.00173466305149; //  
      gps.lon = 0.0020453343;//0.000622928226931; // 
      break;
    case B :
      gps.lat = 0.00007413843;//-0.00171244438533; //
      gps.lon = 0.00121679525; //0.000399127055247; //
    default:
      break;
  }
 
  std::cout << "To address : " << to_address;
  std::cout << "To latitude : " << gps.lat << ", To longitude : " << gps.lon << std::endl;
  
  to_utm_x = projection.forward(gps).x();
  to_utm_y = projection.forward(gps).y();
  std::cout << "to_utm_x : " << to_utm_x << ", to_utm_y : " << to_utm_y << std::endl;
  std::cout << "-----------------------" << std::endl;
  Lanelets to_lanelets = map->laneletLayer.nearest(BasicPoint2d(to_utm_x, to_utm_y), 1);
  // ConstLanelet fromLanelet = from_lanelets[0];
  // ConstLanelet toLanelet = to_lanelets[0];
  
  LineString3d middleLs{map->lineStringLayer.get(16628)};
  LineString3d middleNextLs{map->lineStringLayer.get(10894)};
  Lanelet fromLanelet = map->laneletLayer.get(17447);
  Lanelet toLanelet = map->laneletLayer.get(16629);
  Lanelet nextToLanelet = map->laneletLayer.get(10895);
  Lanelet nextFromLanelet= map->laneletLayer.get(10911);
  middleLs.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
  middleLs.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;
  middleNextLs = middleLs;
  toLanelet.attributes()[AttributeName::Type] = AttributeValueString::Lanelet;
  toLanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
  toLanelet.attributes()[AttributeName::Location] = AttributeValueString::Nonurban;
  fromLanelet.attributes() = toLanelet.attributes();
  nextToLanelet.attributes() = toLanelet.attributes();
  toLanelet.attributes()[AttributeName::OneWay] = false;
  nextToLanelet.attributes()[AttributeName::OneWay] = false;
  


  assert(trafficRules->canChangeLane(fromLanelet, toLanelet));
  // cout << fromLanelet.leftBound().id() << endl;
  // cout << toLanelet.leftBound().id() << endl;
  Optional<routing::LaneletPath> shortestPath = graph->shortestPath(toLanelet, nextFromLanelet, 1, true);
  assert(shortestPath.has_value());
  // GlobalPlanning 클래스 정의 내부에
  std::vector<BasicPoint2d> centerlinePoints;
  std::vector<BasicPoint2d> leftlinePoints;
  std::vector<BasicPoint2d> rightlinePoints;
  
  LaneletMap laneletMap;

  // CreateRoutingGraphs 메서드 내부에서, 최단 경로를 얻은 후에
  if (shortestPath) {
      centerlinePoints.clear(); // 새로운 점들을 추가하기 전에 기존의 점들을 초기화합니다.
      // 최단 경로의 각 lanelet을 순회합니다
      for (const auto& lanelet : *shortestPath) {
          // 현재 lanelet의 중앙선을 가져옵니다
          auto centerline = lanelet.centerline();    
          auto leftline = lanelet.leftBound();
          auto rightline = lanelet.rightBound(); 
          // 중앙선의 각 점을 벡터에 추가합니다
          for (const auto& point : centerline) {
            centerlinePoints.push_back(point.basicPoint2d());
          }
          for (const auto& point : leftline) {
            leftlinePoints.push_back(point.basicPoint2d());
          }
          for (const auto& point : rightline) {
            rightlinePoints.push_back(point.basicPoint2d());
          }
      } 
  }

  VisualizeLeftLine(leftlinePoints);
  VisualizeRightLine(rightlinePoints);

  Optional<routing::Route> route = graph->getRoute(toLanelet, nextFromLanelet, 1, true);
  // routing::LaneletPath shortest_path = route->shortestPath();
  // LaneletSequence fullLane = route->fullLane(fromLanelet);
  // assert(!shortest_path.empty());
  if (route) {
      LaneletSubmapConstPtr routeMap = route->laneletSubmap();
      
      write("/home/eonsoo/Ku_ik/src/planning/global_planning/src/TOWN5.osm", *routeMap->laneletMap(), Origin({0, 0}));
  }
  else{
    cout << "No Route" << endl;
  }
  return centerlinePoints;
}

void GlobalPlanning::VisualizeCenterLine(const std::vector<BasicPoint2d>& centerlinePoints){

     // 메시지 초기화
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // RViz에서 표시할 프레임
    marker.header.stamp = ros::Time::now();
    marker.ns = "centerline";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // 선의 두께

    // 선 색상 설정 (R, G, B, A)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0 ;
    // 중앙선의 점들을 메시지에 추가
    geometry_msgs::PoseArray center_points;
    int count = 0;
    for (const auto& point : centerlinePoints) {
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

void GlobalPlanning::VisualizeLeftLine(const std::vector<BasicPoint2d>& leftlinePoints){

    // 메시지 초기화
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // RViz에서 표시할 프레임
    marker.header.stamp = ros::Time::now();
    marker.ns = "leftline";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // 선의 두께
    marker.pose.orientation.w = 1.0 ;
    // 선 색상 설정 (R, G, B, A)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // 중앙선의 점들을 메시지에 추가
    for (const auto& point : leftlinePoints) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0; // 2D 중앙선이므로 z 값은 0
        marker.points.push_back(p);
    }

    // 메시지 게시
    left_marker_pub.publish(marker);
}


void GlobalPlanning::VisualizeRightLine(const std::vector<BasicPoint2d>& rightlinePoints){

    // 메시지 초기화
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // RViz에서 표시할 프레임
    marker.header.stamp = ros::Time::now();
    marker.ns = "rightline";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // 선의 두께
    marker.pose.orientation.w = 1.0 ;
    // 선 색상 설정 (R, G, B, A)
    marker.color.r = 0.1;
    marker.color.g = 0.3;
    marker.color.b = 0.7;
    marker.color.a = 1.0;

    // 중앙선의 점들을 메시지에 추가
    for (const auto& point : rightlinePoints) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0.0; // 2D 중앙선이므로 z 값은 0
        marker.points.push_back(p);
    }

    // 메시지 게시
    right_marker_pub.publish(marker);
}



int main(int argc, char **argv){

  ros::init(argc, argv, "Global_planning_node");

  GlobalPlanning gp;
  std::vector<BasicPoint2d> centerline_point;

  centerline_point = gp.CreateRoutingGraphs();

  // while(ros::ok()){
  gp.VisualizeCenterLine(centerline_point);
  // }
  ros::spin();

  return 0;
}
