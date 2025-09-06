#ifndef __NOXIMROUTING_DISTANCE_AWARE_H__
#define __NOXIMROUTING_DISTANCE_AWARE_H__

#include "RoutingAlgorithm.h"
#include "RoutingAlgorithms.h"
#include "../Router.h"

using namespace std;

class Routing_DISTANCE_AWARE : public RoutingAlgorithm {
public:
    // Main routing function - implements distance-aware routing
    vector<int> route(Router * router, const RouteData& route_data);
    
    static Routing_DISTANCE_AWARE * getInstance();
    
private:
    Routing_DISTANCE_AWARE(){};
    ~Routing_DISTANCE_AWARE(){};
    
    // Helper functions
    vector<int> routeXY(const RouteData& route_data);
    int getWirelessDirection(int current_id, int destination_id);
    int getHubId(int node_id);
    bool canHubsCommunicate(int hub1_id, int hub2_id);
    
    // Utility functions
    Coord id2Coord(int id);
    int coord2Id(const Coord& coord);
    
    static Routing_DISTANCE_AWARE * routing_distance_aware;
    static RoutingAlgorithmsRegister routingAlgorithmsRegister;
    static RoutingAlgorithm * xy;
};

#endif