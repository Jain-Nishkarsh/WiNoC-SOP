/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the Fuzzy Token MAC Controller
 */

#ifndef __NOXIMFUZZYTOKENCONTROLLER_H__
#define __NOXIMFUZZYTOKENCONTROLLER_H__

#include <systemc.h>
#include <vector>
#include <map>
#include <set>
#include <bitset>
#include <deque>
#include "GlobalParams.h"
#include "DataStructs.h"

using namespace std;

// Maximum number of nodes supported
#define MAX_FUZZY_TOKEN_NODES 256

// Period modes
enum FuzzyTokenMode {
    FUZZY_MODE,
    FOCUSED_MODE
};

// Step outcome types
enum StepOutcome {
    OUTCOME_COLLISION,
    OUTCOME_SUCCESS,
    OUTCOME_SILENCE,
    OUTCOME_CONGESTION // New: TX-side congestion (treat like collision for AIMD)
};

// Per-channel Fuzzy Token state
class FuzzyTokenChannelState {
public:
    FuzzyTokenMode periodMode;
    int tokenID;
    int FA_size;
    bitset<MAX_FUZZY_TOKEN_NODES> fuzzyArea;
    vector<double> transmissionProb; // p[i] for each node
    int currentStepCycles;
    
    int numNodes;
    FuzzyTokenConfig config;
    vector<int> tokenRingOrder; // Static or pseudo-random order
    int tokenRingPosition; // Current position in the ring
    
    // Statistics
    int totalCollisions;
    int totalCongestions; // New: count TX-side congestion events
    int totalSuccesses;
    int totalSilences;
    int totalFuzzySteps;
    int totalFocusedSteps;
    
    // Histogram of FA_size (key: FA_size, value: count of steps)
    map<int, int> FA_size_histogram;
    
    // Global transmission tracking (for broadcast collision detection)
    int activeTransmittersThisStep;
    set<int> transmittingHubsThisStep;
    
    // PHASE 1: Ready bit piggybacking - tracks which hubs are ready to send
    vector<bool> ready_bitmap;
    
    // PHASE 2: Ready-count trigger - tracks ready count history for proactive mode switching
    // CORRECTED LOGIC: High traffic → FOCUSED (deterministic), Low traffic → FUZZY (probabilistic)
    deque<int> ready_history;
    int ready_history_window;       // W: sliding window size (from config)
    int fuzzy_trigger_count;        // K: consecutive windows with ≤1 ready for FUZZY (from config)
    int focused_trigger_count;      // M: consecutive windows with >1 ready for FOCUSED (from config)
    int cycles_in_current_mode;     // Hysteresis: prevent rapid switching
    int min_mode_hold_cycles;       // Minimum cycles to stay in a mode (from config)
    
    FuzzyTokenChannelState() : 
        periodMode(FUZZY_MODE),
        tokenID(0),
        FA_size(0),
        currentStepCycles(0),
        numNodes(0),
        tokenRingPosition(0),
        totalCollisions(0),
        totalCongestions(0),
        totalSuccesses(0),
        totalSilences(0),
        totalFuzzySteps(0),
        totalFocusedSteps(0),
        activeTransmittersThisStep(0),
        cycles_in_current_mode(0)
    {
        fuzzyArea.reset();
    }
    
    void initialize(const FuzzyTokenConfig& cfg, int num_nodes, const vector<int>& nodeIds);
    void updateFuzzyArea(StepOutcome outcome);
    void updateTransmissionProbabilities();
    void advanceToken();
    void switchMode();
    void rebuildFuzzyArea();
    double getTransmissionProbability(int nodeId) const;
    bool isInFuzzyArea(int nodeId) const;
    int getTokenHolder() const { return tokenID; }
    FuzzyTokenMode getMode() const { return periodMode; }
    
    // Global transmission tracking methods
    void registerTransmission(int hubId);
    void resetStepState();
    bool hasCollision() const { return activeTransmittersThisStep > 1; }
    bool hasSilence() const { return activeTransmittersThisStep == 0; }
    int getActiveTransmitters() const { return activeTransmittersThisStep; }
    
    // PHASE 1: Ready bitmap management methods
    void setHubReady(int hubId, bool isReady);
    bool isHubReady(int hubId) const;
    void resetReadyBitmap();
    int countReadyHubs() const;
    void logReadyBitmap(int currentCycle) const; // PHASE 1: For testing
    
    // PHASE 2: Ready-count trigger methods
    void updateReadyHistory();
    bool shouldSwitchToFuzzy() const;
    bool shouldSwitchToFocused() const;
    void checkAndSwitchModeProactive();
};

// Global Fuzzy Token Controller (manages all channels)
class FuzzyTokenController {
private:
    map<int, FuzzyTokenChannelState*> channelStates;
    
    // Singleton
    static FuzzyTokenController* instance;
    FuzzyTokenController() {}
    
public:
    static FuzzyTokenController& getInstance() {
        if (!instance) {
            instance = new FuzzyTokenController();
        }
        return *instance;
    }
    
    void registerChannel(int channelId, const FuzzyTokenConfig& config, 
                        int numNodes, const vector<int>& nodeIds);
    
    FuzzyTokenChannelState* getChannelState(int channelId);
    
    // Called by hub at end of each step to update state
    void endStep(int channelId, StepOutcome outcome, int stepCycles);
    
    void reset();
    
    // Statistics
    void printStats(int channelId);
    void printAllStats();
    bool hasFuzzyTokenChannels() const { return !channelStates.empty(); }
};

#endif
