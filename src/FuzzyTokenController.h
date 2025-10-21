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
    OUTCOME_SILENCE
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
    int totalSuccesses;
    int totalSilences;
    int totalFuzzySteps;
    int totalFocusedSteps;
    
    // Histogram of FA_size (key: FA_size, value: count of steps)
    map<int, int> FA_size_histogram;
    
    // Global transmission tracking (for broadcast collision detection)
    int activeTransmittersThisStep;
    set<int> transmittingHubsThisStep;
    
    FuzzyTokenChannelState() : 
        periodMode(FUZZY_MODE),
        tokenID(0),
        FA_size(0),
        currentStepCycles(0),
        numNodes(0),
        tokenRingPosition(0),
        totalCollisions(0),
        totalSuccesses(0),
        totalSilences(0),
        totalFuzzySteps(0),
        totalFocusedSteps(0),
        activeTransmittersThisStep(0)
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
