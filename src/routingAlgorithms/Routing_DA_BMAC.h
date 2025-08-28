#ifndef __NOXIMROUTING_DA_BMAC_H__
#define __NOXIMROUTING_DA_BMAC_H__

#include "RoutingAlgorithm.h"
#include "RoutingAlgorithms.h"
#include "../Router.h"

using namespace std;

class Routing_DA_BMAC : public RoutingAlgorithm {
public:
    // Main routing function - implements distance-aware routing
    vector<int> route(Router * router, const RouteData& route_data);
    
    static Routing_DA_BMAC * getInstance();
    
private:
    Routing_DA_BMAC(){};
    ~Routing_DA_BMAC(){};
    
    // Helper functions
    vector<int> routeXY(const RouteData& route_data);
    int getWirelessDirection(int current_id, int destination_id);
    int getHubId(int node_id);
    bool canHubsCommunicate(int hub1_id, int hub2_id);
    
    // Utility functions
    Coord id2Coord(int id);
    int coord2Id(const Coord& coord);
    
    static Routing_DA_BMAC * routing_DA_BMAC;
    static RoutingAlgorithmsRegister routingAlgorithmsRegister;
    static RoutingAlgorithm * xy;
};

#endif