#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// we want assert statements to work in release mode
#undef NDEBUG


void part1CreatingAndUsingRoutingGraphs();
void part2UsingRoutes();
void part3UsingRoutingGraphContainers();

int main() {

  part1CreatingAndUsingRoutingGraphs();

  return 0;
}

void part1CreatingAndUsingRoutingGraphs() {
  using namespace lanelet;

  LaneletMapPtr map = load("/home/eonsoo/Ku_ik/src/planning/global_planning/src/Town05.osm", Origin({0, 0}));
  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphUPtr graph = routing::RoutingGraph::build(*map, *trafficRules);
  ConstLanelet fromLanelet = map->laneletLayer.get(17447);
  ConstLanelet toLanelet = map->laneletLayer.get(5797);
  Optional<routing::LaneletPath> shortestPath = graph->shortestPath(fromLanelet, toLanelet, 1);
  std::cout << shortestPath.has_value() << std::endl;
  

  Optional<routing::Route> route = graph->getRoute(fromLanelet, toLanelet, 1);
    if (route) {
        LaneletSubmapConstPtr routeMap = route->laneletSubmap();
        write("/home/eonsoo/Ku_ik/src/planning/global_planning/src/Town05.osm", *routeMap->laneletMap(), Origin({0, 0}));
    }
}
