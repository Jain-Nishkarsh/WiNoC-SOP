/*
 * Noxim - the NoC Simulator
 *
 * (C) 2025
 *
 * This file contains the declaration of the BMAC Controller
 * for bidirectional token ring management
 */

#ifndef __BMACCONTROLLER_H__
#define __BMACCONTROLLER_H__

#include <systemc.h>
#include "Utils.h"
#include "DataStructs.h"
#include <map>
#include <vector>

using namespace std;

// BMAC-specific states
#define BMAC_IDLE          0
#define BMAC_TOKEN_HOLD    1
#define BMAC_TRANSMITTING  2
#define BMAC_RECEIVING     3

// BMAC direction modes for bidirectional token ring
#define BMAC_CLOCKWISE      0
#define BMAC_COUNTERCLOCKWISE 1

struct BMACState {
    int hub_id;
    int current_state;
    int token_direction;
    int token_hold_cycles;
    bool bidirectional_mode;
    bool has_token;
    int next_hub_cw;    // next hub in clockwise direction
    int next_hub_ccw;   // next hub in counter-clockwise direction
};

SC_MODULE(BMACController)
{
    SC_HAS_PROCESS(BMACController);

    // I/O Ports
    sc_in_clk clock;
    sc_in<bool> reset;

    // BMAC state management
    map<int, BMACState> hub_states;
    map<int, vector<int>> token_ring_order;
    map<int, int> current_token_holder;
    map<int, int> token_direction;

    // Constructor
    BMACController(sc_module_name nm);

    // Main process
    void bmacProcess();
    
    // Token management functions
    void initializeBMAC();
    void updateTokenRing(int channel);
    void passBidirectionalToken(int channel, int current_holder);
    bool shouldReverseDirection(int channel, int hub_id);
    
    // Utility functions
    int getNextHub(int channel, int current_hub, int direction);
    void setTokenDirection(int channel, int direction);
    bool isTransmissionComplete(int hub_id, int channel);
    
    // BMAC-specific functions
    void handleTokenExpiration(int channel, int hub_id);
    void optimizeTokenPath(int channel);
    
private:
    // Configuration
    void configureBMACRing(int channel, const vector<int>& hub_order);
    void enableBidirectionalMode(int channel, bool enable);
    
    // State management
    void updateHubState(int hub_id, int new_state);
    BMACState getHubState(int hub_id);
};

#endif
