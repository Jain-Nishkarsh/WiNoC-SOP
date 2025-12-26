#include "Routing_DISTANCE_CONGESTION_AWARE.h"
#include "../Hub.h"

RoutingAlgorithmsRegister Routing_DISTANCE_CONGESTION_AWARE::routingAlgorithmsRegister("DISTANCE_CONGESTION_AWARE", getInstance());

Routing_DISTANCE_CONGESTION_AWARE * Routing_DISTANCE_CONGESTION_AWARE::instance = 0;
RoutingAlgorithm * Routing_DISTANCE_CONGESTION_AWARE::xy = 0;

Routing_DISTANCE_CONGESTION_AWARE * Routing_DISTANCE_CONGESTION_AWARE::getInstance() {
    if (instance == 0)
        instance = new Routing_DISTANCE_CONGESTION_AWARE();
    return instance;
}

vector<int> Routing_DISTANCE_CONGESTION_AWARE::route(Router * router, const RouteData& route_data)
{
    vector<int> directions;
    int current_id = route_data.current_id;
    int destination_id = route_data.dst_id;

    // Coordinates and Manhattan distance
    Coord current_coord = id2Coord(current_id);
    Coord destination_coord = id2Coord(destination_id);
    int manhattan_distance = abs(current_coord.x - destination_coord.x) + 
                             abs(current_coord.y - destination_coord.y);

    int distance_threshold = GlobalParams::da_threshold; // from config

    if (GlobalParams::use_winoc && manhattan_distance > distance_threshold) {
        // Try wireless (same heuristic as distance-aware)
        int wireless_direction = getWirelessDirection(current_id, destination_id);
        if (wireless_direction != NOT_VALID) {
            // Additional guard: if the source hub's TX buffers are congested, avoid wireless
            int current_hub_id = getHubId(current_id);
            if (current_hub_id != NOT_VALID && Hub::isTxCongestedForHub(current_hub_id)) {
                if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
                    cerr << "CA: Fallback to WIRED (hub TX congested) for "
                         << current_id << "->" << destination_id
                         << " dist=" << manhattan_distance << endl;
                }
                return routeXY(router, route_data);
            }
            // Simple congestion-aware guard: if hub input buffer for this VC is full, fallback to wired
            // Matches the condition checked in Router::txProcess when forwarding
            bool hub_buffer_full = false;
            if (router != NULL) {
                int vc = route_data.vc_id;
                if (vc < 0 || vc >= GlobalParams::n_virtual_channels) vc = 0; // safety
                TBufferFullStatus bfs = router->buffer_full_status_tx[DIRECTION_HUB].read();
                hub_buffer_full = bfs.mask[vc];
            }

            if (!hub_buffer_full) {
                directions.push_back(DIRECTION_HUB);
                return directions;
            } else {
                if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
                    cerr << "CA: Fallback to WIRED (hub buffer full) for "
                         << current_id << "->" << destination_id
                         << " dist=" << manhattan_distance << endl;
                }
                return routeXY(router, route_data);
            }
        }
    }

    // Wired XY otherwise
    return routeXY(router, route_data);
}

vector<int> Routing_DISTANCE_CONGESTION_AWARE::routeXY(Router * router, const RouteData& route_data)
{
    if (!xy) {
        xy = RoutingAlgorithms::get("XY");
        if (!xy) assert(false);
    }
    // Delegate to XY routing for wired paths
    return xy->route(router, route_data);
}

int Routing_DISTANCE_CONGESTION_AWARE::getWirelessDirection(int current_id, int destination_id)
{
    int current_hub_id = getHubId(current_id);
    int destination_hub_id = getHubId(destination_id);

    if (current_hub_id == NOT_VALID || destination_hub_id == NOT_VALID)
        return NOT_VALID;

    if (canHubsCommunicate(current_hub_id, destination_hub_id))
        return DIRECTION_HUB;

    return NOT_VALID;
}

int Routing_DISTANCE_CONGESTION_AWARE::getHubId(int node_id)
{
    map<int, int>::iterator it = GlobalParams::hub_for_tile.find(node_id);
    if (it != GlobalParams::hub_for_tile.end())
        return it->second;
    return NOT_VALID;
}

bool Routing_DISTANCE_CONGESTION_AWARE::canHubsCommunicate(int hub1_id, int hub2_id)
{
    if (hub1_id == hub2_id) return false; // same hub is not a wireless hop
    // Single-channel common case: assume connectivity if both have the channel
    return true;
}

// Utilities
Coord Routing_DISTANCE_CONGESTION_AWARE::id2Coord(int id)
{
    Coord coord;
    coord.x = id % GlobalParams::mesh_dim_x;
    coord.y = id / GlobalParams::mesh_dim_x;
    return coord;
}

int Routing_DISTANCE_CONGESTION_AWARE::coord2Id(const Coord& coord)
{
    return coord.y * GlobalParams::mesh_dim_x + coord.x;
}
