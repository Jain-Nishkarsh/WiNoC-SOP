/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the global params needed by Noxim
 * to forward configuration to every sub-block
 */

#ifndef __NOXIMGLOBALPARAMS_H__
#define __NOXIMGLOBALPARAMS_H__ 

#include <map>
#include <utility>
#include <vector>
#include <string>
#include <fstream>

using namespace std;

#define CONFIG_FILENAME        "config.yaml"
#define POWER_CONFIG_FILENAME  "power.yaml"

// Define the directions as numbers
#define DIRECTIONS              4
#define DIRECTION_NORTH         0
#define DIRECTION_EAST          1
#define DIRECTION_SOUTH         2
#define DIRECTION_WEST          3
#define DIRECTION_LOCAL         4
#define DIRECTION_HUB           5
#define DIRECTION_HUB_RELAY     5000
#define DIRECTION_WIRELESS    747

#define MAX_VIRTUAL_CHANNELS	8
#define DEFAULT_VC 		0

#define RT_AVAILABLE 1
#define RT_ALREADY_SAME -1
#define RT_ALREADY_OTHER_OUT -2
#define RT_OUTVC_BUSY -3

// Generic not reserved resource
#define NOT_RESERVED          -2

// To mark invalid or non exhistent values
#define NOT_VALID             -1

//Topologies
#define TOPOLOGY_MESH          "MESH"
//Delta Networks Topologies
#define TOPOLOGY_BASELINE      "BASELINE"
#define TOPOLOGY_BUTTERFLY     "BUTTERFLY"
#define TOPOLOGY_OMEGA         "OMEGA"

// Routing algorithms
#define ROUTING_DYAD           "DYAD"
#define ROUTING_TABLE_BASED    "TABLE_BASED"


// Channel selection 
#define CHSEL_RANDOM 0
#define CHSEL_FIRST_FREE 1

// Traffic distribution
#define TRAFFIC_RANDOM         "TRAFFIC_RANDOM"
#define TRAFFIC_TRANSPOSE1     "TRAFFIC_TRANSPOSE1"
#define TRAFFIC_TRANSPOSE2     "TRAFFIC_TRANSPOSE2"
#define TRAFFIC_HOTSPOT        "TRAFFIC_HOTSPOT"
#define TRAFFIC_TABLE_BASED    "TRAFFIC_TABLE_BASED"
#define TRAFFIC_BIT_REVERSAL   "TRAFFIC_BIT_REVERSAL"
#define TRAFFIC_SHUFFLE        "TRAFFIC_SHUFFLE"
#define TRAFFIC_BUTTERFLY      "TRAFFIC_BUTTERFLY"
#define TRAFFIC_LOCAL	       "TRAFFIC_LOCAL"
#define TRAFFIC_ULOCAL	       "TRAFFIC_ULOCAL"

// Verbosity levels
#define VERBOSE_OFF            "VERBOSE_OFF"
#define VERBOSE_LOW            "VERBOSE_LOW"
#define VERBOSE_MEDIUM         "VERBOSE_MEDIUM"
#define VERBOSE_HIGH           "VERBOSE_HIGH"


// Wireless MAC constants
#define RELEASE_CHANNEL 1
#define HOLD_CHANNEL 	2

#define TOKEN_HOLD             "TOKEN_HOLD"
#define TOKEN_MAX_HOLD         "TOKEN_MAX_HOLD"
#define TOKEN_PACKET           "TOKEN_PACKET"
#define FUZZY_TOKEN            "FUZZY_TOKEN"
#define FUZZY_RCT              "FUZZY_RCT"
#define FUZZY_RAJ              "FUZZY_RAJ"
#define FUZZY_SWJ              "FUZZY_SWJ"

enum ProbabilityDistribution {
    PI_EQUAL,
    PI_GAUSSIAN,
    PI_OTHER
};

// Fuzzy Token configuration
typedef struct {
    int preamble_cycles;        // Cycles for preamble phase (default: 1)
    double thr1;                // Lower threshold for mode switch (fraction of N or absolute)
    double thr2;                // Upper threshold for mode switch (fraction of N or absolute)
    int initial_FA;             // Initial fuzzy area size
    int FA_increment;           // Additive increase on silence (default: 1)
    double FA_decrement_factor; // Multiplicative decrease on collision (default: 0.5)
    string pi_type_str;         // Probability distribution type: "equal" or "gaussian" (string for config)
    ProbabilityDistribution pi_type; // Enum for runtime efficiency
    double min_transmission_prob; // Minimum transmission probability for nodes with data
    int initial_token_holder;   // Initial token holder node ID
    string token_order;         // Token order: "static" or "pseudo_random"
    bool thr_is_percentage;     // True if thr1/thr2 are percentages of N
    
    // Ready-count trigger parameters (FUZZY_RCT)
    int ready_history_window;        // W: Number of cycles to track ready counts (default: 6)
    int fuzzy_ready_threshold;       // K: Max ready hubs for FUZZY mode (≤K ready) (default: 3)
    int focused_ready_threshold;     // M: Min ready hubs for FOCUSED mode (>M ready) (default: 8)
    int fuzzy_consecutive_windows;   // Consecutive windows ≤K to trigger FUZZY (default: 4)
    int focused_consecutive_windows; // Consecutive windows >M to trigger FOCUSED (default: 4)
} FuzzyTokenConfig;

typedef struct {
    pair<double, double> ber;
    int dataRate;
    vector<string> macPolicy;
    vector<int> tokenRingOrder;
    FuzzyTokenConfig fuzzyTokenConfig;
} ChannelConfig;

typedef struct {
    vector<int> attachedNodes;
    vector<int> rxChannels;
    vector<int> txChannels;
    int toTileBufferSize;
    int fromTileBufferSize;
    int txBufferSize;
    int rxBufferSize;
} HubConfig;

typedef struct {
    map<pair <int, int>, double> front;
    map<pair <int, int>, double> pop;
    map<pair <int, int>, double> push;
    map<pair <int, int>, double> leakage;
} BufferPowerConfig;

typedef map<double, pair <double, double> > LinkBitLinePowerConfig;

typedef struct {
    map<pair<double, double>, pair<double, double> > crossbar_pm;
    map<int, pair<double, double> > network_interface;
    map<string, pair<double, double> > routing_algorithm_pm;
    map<string, pair<double, double> > selection_strategy_pm;
} RouterPowerConfig;

typedef struct {
    pair<double, double> transceiver_leakage;
    pair<double, double> transceiver_biasing;
    double rx_dynamic;
    double rx_snooping;
    double default_tx_energy;
    map<pair <int, int>, double> transmitter_attenuation_map;
} HubPowerConfig;

typedef struct {
    BufferPowerConfig bufferPowerConfig;
    LinkBitLinePowerConfig linkBitLinePowerConfig;
    RouterPowerConfig routerPowerConfig;
    HubPowerConfig hubPowerConfig;
} PowerConfig;

struct GlobalParams {
    static string verbose_mode;
    static int trace_mode;
    static string trace_filename;
    static string topology;
    static int mesh_dim_x;
    static int mesh_dim_y;
    static int n_delta_tiles;
    static double r2r_link_length;
    static double r2h_link_length;
    static int buffer_depth;
    static int flit_size;
    static int min_packet_size;
    static int max_packet_size;
    static string routing_algorithm;
    static string routing_table_filename;
    static string selection_strategy;
    static double packet_injection_rate;
    static double probability_of_retransmission;
    static double locality;
    static string traffic_distribution;
    static string traffic_table_filename;
    static string config_filename;
    static string power_config_filename;
    static int clock_period_ps;
    static int simulation_time;
    static int n_virtual_channels;
    static int reset_time;
    static int stats_warm_up_time;
    static int rnd_generator_seed;
    static bool detailed;
    static vector <pair <int, double> > hotspots;
    static double dyad_threshold;
    static int da_threshold;
    static unsigned int max_volume_to_be_drained;
    static bool show_buffer_stats;
    static bool use_winoc;
    static int winoc_dst_hops;
    static bool use_powermanager;

    // CSV Logging
    static bool csv_log_enabled;
    static string csv_routing_log_filename;
    static string csv_mac_log_filename;
    static ofstream csv_routing_log_stream;
    static ofstream csv_mac_log_stream;

    static ChannelConfig default_channel_configuration;
    static map<int, ChannelConfig> channel_configuration;
    static HubConfig default_hub_configuration;
    static map<int, HubConfig> hub_configuration;
    static map<int, int> hub_for_tile;
    static PowerConfig power_configuration;
    // out of yaml configuration
    static bool ascii_monitor;
    static int channel_selection;
};

#endif
