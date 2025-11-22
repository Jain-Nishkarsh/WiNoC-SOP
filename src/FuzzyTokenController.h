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

enum StepOutcome {
    OUTCOME_COLLISION,
    OUTCOME_SUCCESS,
    OUTCOME_SILENCE,
    OUTCOME_CONGESTION
};

class FuzzyTokenChannelState {
public:
    FuzzyTokenMode periodMode;
    int tokenID;
    int FA_size;
    bitset<MAX_FUZZY_TOKEN_NODES> fuzzyArea;
    vector<double> transmissionProb;
    int currentStepCycles;
    
    int numNodes;
    FuzzyTokenConfig config;
    vector<int> tokenRingOrder;
    int tokenRingPosition;
    string macPolicy;
    
    // Statistics
    int totalCollisions;
    int totalCongestions;
    int totalSuccesses;
    int totalSilences;
    int totalFuzzySteps;
    int totalFocusedSteps;
    map<int, int> FA_size_histogram;
    
    // Cache for Gaussian weights
    map<int, vector<double>> gaussian_weights_cache;

    // Transmission tracking
    int activeTransmittersThisStep;
    set<int> transmittingHubsThisStep;
    
    // Ready bitmap (Phase 1)
    vector<bool> ready_bitmap;
    
    // Ready-count trigger (Phase 2)
    deque<int> ready_history;
    int ready_history_window;
    int fuzzy_ready_threshold;
    int focused_ready_threshold;
    int fuzzy_consecutive_windows;
    int focused_consecutive_windows;
    
    // Ready-aware jump
    int cycles_since_last_jump;
    int jump_cooldown;
    
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
        cycles_since_last_jump(0)
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
    
    // Ready bitmap management
    void setHubReady(int hubId, bool isReady);
    bool isHubReady(int hubId) const;
    void resetReadyBitmap();
    int countReadyHubs() const;
    void logReadyBitmap(int currentCycle) const;
    
    // Ready-count trigger
    void updateReadyHistory();
    bool shouldSwitchToFuzzy() const;
    bool shouldSwitchToFocused() const;
    void checkAndSwitchModeProactive();
    
    // Ready-aware jump
    int findNextReadyToken(int startPos) const;
    void advanceTokenSmart();
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
                        int numNodes, const vector<int>& nodeIds, const string& policy);
    FuzzyTokenChannelState* getChannelState(int channelId);
    void endStep(int channelId, StepOutcome outcome, int stepCycles);
    void reset();
    void printStats(int channelId);
    void printAllStats();
    bool hasFuzzyTokenChannels() const { return !channelStates.empty(); }
};

#endif
