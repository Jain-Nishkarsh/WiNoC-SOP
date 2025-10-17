#include "Routing_DISTANCE_AWARE.h"

RoutingAlgorithmsRegister Routing_DISTANCE_AWARE::routingAlgorithmsRegister("DISTANCE_AWARE", getInstance());

Routing_DISTANCE_AWARE * Routing_DISTANCE_AWARE::routing_distance_aware = 0;
RoutingAlgorithm * Routing_DISTANCE_AWARE::xy = 0;

Routing_DISTANCE_AWARE * Routing_DISTANCE_AWARE::getInstance() {
    if (routing_distance_aware == 0)
        routing_distance_aware = new Routing_DISTANCE_AWARE();
    return routing_distance_aware;
}

vector<int> Routing_DISTANCE_AWARE::route(Router * router, const RouteData& route_data)
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
    static int wired_count = 0, wireless_count = 0, debug_counter = 0;
    
    if (manhattan_distance > distance_threshold && GlobalParams::use_winoc) {
        // Long distance: use wireless if available
        int wireless_direction = getWirelessDirection(current_id, destination_id);
        if (wireless_direction != NOT_VALID) {
            wireless_count++;
            if (wireless_count <= 3 || wireless_count % 100 == 0) {
                cerr << "WIRELESS #" << wireless_count << ": " << current_id << "->" << destination_id 
                     << " dist=" << manhattan_distance << endl;
            }
            directions.push_back(wireless_direction);
            return directions;
        }
    }
    
    // Short distance OR wireless not available: use wired XY routing
    wired_count++;
    if (debug_counter < 5) {
        cerr << "WIRED: " << current_id << "->" << destination_id << " dist=" << manhattan_distance 
             << " (thr=" << distance_threshold << ")" << endl;
        debug_counter++;
    }
    return routeXY(route_data);
}

vector<int> Routing_DISTANCE_AWARE::routeXY(const RouteData& route_data)
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

int Routing_DISTANCE_AWARE::getWirelessDirection(int current_id, int destination_id)
{
    // Check if current node has access to wireless hub
    int current_hub_id = getHubId(current_id);
    int destination_hub_id = getHubId(destination_id);
    
    static int debug_counter2 = 0;
    if (debug_counter2 < 10) {
        cerr << "DA getWireless: " << current_id << "->" << destination_id 
             << " cur_hub=" << current_hub_id << " dst_hub=" << destination_hub_id << endl;
        debug_counter2++;
    }
    
    if (current_hub_id == NOT_VALID || destination_hub_id == NOT_VALID) {
        return NOT_VALID;
    }
        
    // Check if hubs can communicate (same radio channel)
    if (canHubsCommunicate(current_hub_id, destination_hub_id)) {
        static int wireless_used = 0;
        if (wireless_used < 5) {
            cerr << "DA: WIRELESS! " << current_id << "->" << destination_id 
                 << " hub " << current_hub_id << "->" << destination_hub_id << endl;
            wireless_used++;
        }
        return DIRECTION_HUB;
    }
        
    return NOT_VALID;
}

int Routing_DISTANCE_AWARE::getHubId(int node_id)
{
    // Use the actual hub configuration from GlobalParams
    map<int, int>::iterator it = GlobalParams::hub_for_tile.find(node_id);
    
    if (it != GlobalParams::hub_for_tile.end())
        return it->second;  // Return the hub ID this node is connected to
        
    return NOT_VALID;  // Node not connected to any hub
}

bool Routing_DISTANCE_AWARE::canHubsCommunicate(int hub1_id, int hub2_id)
{
    // In single channel configuration, all hubs can communicate
    // For multi-channel, check channel assignments
    if (hub1_id == hub2_id)
        return false; // Same hub
        
    return true; // All hubs share radio channel 0 in this configuration
}

// Utility functions
Coord Routing_DISTANCE_AWARE::id2Coord(int id)
{
    Coord coord;
    coord.x = id % GlobalParams::mesh_dim_x;
    coord.y = id / GlobalParams::mesh_dim_x;
    return coord;
}

int Routing_DISTANCE_AWARE::coord2Id(const Coord& coord)
{
    return coord.y * GlobalParams::mesh_dim_x + coord.x;
}