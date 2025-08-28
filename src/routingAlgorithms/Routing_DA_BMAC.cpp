#include "Routing_DA_BMAC.h"

RoutingAlgorithmsRegister Routing_DA_BMAC::routingAlgorithmsRegister("DA_BMAC", getInstance());

Routing_DA_BMAC * Routing_DA_BMAC::routing_DA_BMAC = 0;
RoutingAlgorithm * Routing_DA_BMAC::xy = 0;

Routing_DA_BMAC * Routing_DA_BMAC::getInstance() {
    if (routing_DA_BMAC == 0)
        routing_DA_BMAC = new Routing_DA_BMAC();
    return routing_DA_BMAC;
}

vector<int> Routing_DA_BMAC::route(Router * router, const RouteData& route_data)
{
    vector<int> directions;
    int current_id = route_data.current_id;
    int destination_id = route_data.dst_id;
    
    // Get coordinates from node IDs
    Coord current_coord = id2Coord(current_id);
    Coord destination_coord = id2Coord(destination_id);
    
    // Calculate Manhattan distance
    int manhattan_distance = abs(current_coord.x - destination_coord.x) + 
                           abs(current_coord.y - destination_coord.y);
    
    // Distance threshold from configuration (default 5 for 8x8 mesh)
    int distance_threshold = GlobalParams::da_threshold;
    
    // DA routing decision: wireless vs wired
    if (manhattan_distance > distance_threshold && GlobalParams::use_winoc) {
        // Long distance: use wireless if available
        int wireless_direction = getWirelessDirection(current_id, destination_id);
        if (wireless_direction != NOT_VALID) {
            directions.push_back(wireless_direction);
            return directions;
        }
    }
    
    // Short distance OR wireless not available: use wired XY routing
    return routeXY(route_data);
}

vector<int> Routing_DA_BMAC::routeXY(const RouteData& route_data)
{
    // Use existing XY routing algorithm for wired routing
    if (!xy) {
        xy = RoutingAlgorithms::get("XY");
        if (!xy)
            assert(false);
    }
    
    // Delegate to XY routing for wired paths
    return xy->route(NULL, route_data);
}

int Routing_DA_BMAC::getWirelessDirection(int current_id, int destination_id)
{
    // Check if current node has access to wireless hub
    int current_hub_id = getHubId(current_id);
    int destination_hub_id = getHubId(destination_id);
    
    if (current_hub_id == NOT_VALID || destination_hub_id == NOT_VALID)
        return NOT_VALID;
        
    // Check if hubs can communicate (same radio channel)
    if (canHubsCommunicate(current_hub_id, destination_hub_id))
        return DIRECTION_HUB;
        
    return NOT_VALID;
}

int Routing_DA_BMAC::getHubId(int node_id)
{
    // Based on the 8x8 mesh with 16 hubs configuration
    // Each hub covers a 2x2 area of tiles
    Coord coord = id2Coord(node_id);
    
    int hub_x = coord.x / 2;
    int hub_y = coord.y / 2;
    
    if (hub_x >= 0 && hub_x < 4 && hub_y >= 0 && hub_y < 4)
        return hub_y * 4 + hub_x;
        
    return NOT_VALID;
}

bool Routing_DA_BMAC::canHubsCommunicate(int hub1_id, int hub2_id)
{
    // In single channel configuration, all hubs can communicate
    // For multi-channel, check channel assignments
    if (hub1_id == hub2_id)
        return false; // Same hub
        
    return true; // All hubs share radio channel 0 in this configuration
}

// Utility functions
Coord Routing_DA_BMAC::id2Coord(int id)
{
    Coord coord;
    coord.x = id % GlobalParams::mesh_dim_x;
    coord.y = id / GlobalParams::mesh_dim_x;
    return coord;
}

int Routing_DA_BMAC::coord2Id(const Coord& coord)
{
    return coord.y * GlobalParams::mesh_dim_x + coord.x;
}