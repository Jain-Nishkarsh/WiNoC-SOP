#ifndef __NOXIMROUTING_DISTANCE_CONGESTION_AWARE_H__
#define __NOXIMROUTING_DISTANCE_CONGESTION_AWARE_H__

#include "RoutingAlgorithm.h"
#include "RoutingAlgorithms.h"
#include "../Router.h"

using namespace std;

// Distance-aware routing with simple congestion-aware fallback for wireless
class Routing_DISTANCE_CONGESTION_AWARE : public RoutingAlgorithm {
public:
    // Main routing function - distance-aware with wireless buffer check
    vector<int> route(Router * router, const RouteData& route_data);
    static Routing_DISTANCE_CONGESTION_AWARE * getInstance();

private:
    Routing_DISTANCE_CONGESTION_AWARE(){};
    ~Routing_DISTANCE_CONGESTION_AWARE(){};

    // Helper functions
    vector<int> routeXY(Router * router, const RouteData& route_data);
    int getWirelessDirection(int current_id, int destination_id);
    int getHubId(int node_id);
    bool canHubsCommunicate(int hub1_id, int hub2_id);

    // Utility functions
    Coord id2Coord(int id);
    int coord2Id(const Coord& coord);

    static Routing_DISTANCE_CONGESTION_AWARE * instance;
    static RoutingAlgorithmsRegister routingAlgorithmsRegister;
    static RoutingAlgorithm * xy;
};

#endif
