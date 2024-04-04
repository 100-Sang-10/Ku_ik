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

#include <iostream>
// we want assert statements to work in release mode
#undef NDEBUG
#define A    71
#define B    72



using namespace lanelet;
using namespace std;

class GlobalPlanning{

  private: 
    LaneletMapPtr map;
    traffic_rules::TrafficRulesPtr trafficRules;
    routing::RoutingGraphUPtr graph;
    string a = "A";
    string b = "B";

  public:
    GlobalPlanning(){
        map = load("/home/eonsoo/Ku_ik/src/planning/global_planning/src/Town03.osm",  projection::UtmProjector(Origin({0, 0})));
        trafficRules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
        graph = routing::RoutingGraph::build(*map, *trafficRules);
    };
    ~GlobalPlanning(){};
    void CreateRoutingGraphs();

};


void GlobalPlanning::CreateRoutingGraphs() {
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
    gps.lat = -0.00072758317;
    gps.lon = 0.0020453343;
    break;
  case B :
    gps.lat = 0.00007413843;
    gps.lon = 0.00121679525;
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
  float to_utm_x, to_utm_y;
  std::cout << "----------Address----------- \n"
            << " A : 71, B : 72 \n"
            << "Write from address number : ";
  std::cin >> to_address;

  switch (to_address)
  {
  case A :
    gps.lat = -0.00072758317;
    gps.lon = 0.0020453343;
    break;
  case B :
    gps.lat = 0.00007413843;
    gps.lon = 0.00121679525;
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
 
  
  ConstLanelet fromLanelet = from_lanelets[0];
  ConstLanelet toLanelet = to_lanelets[0];
  Optional<routing::LaneletPath> shortestPath = graph->shortestPath(fromLanelet, toLanelet, 1);
  
  std::cout << shortestPath.has_value() << std::endl;
  

  Optional<routing::Route> route = graph->getRoute(fromLanelet, toLanelet, 1);
    if (route) {
        LaneletSubmapConstPtr routeMap = route->laneletSubmap();
        write("/home/eonsoo/Ku_ik/src/planning/global_planning/src/Town03_short.osm", *routeMap->laneletMap(), Origin({0, 0}));
    }
}

int main() {

  GlobalPlanning gp;
  gp.CreateRoutingGraphs();

  return 0;
}
