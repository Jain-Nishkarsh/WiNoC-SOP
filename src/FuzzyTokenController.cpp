/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the Fuzzy Token MAC Controller implementation
 */

#include "FuzzyTokenController.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

// Static instance
FuzzyTokenController* FuzzyTokenController::instance = nullptr;

void FuzzyTokenChannelState::initialize(const FuzzyTokenConfig& cfg, int num_nodes, const vector<int>& nodeIds) {
    config = cfg;
    numNodes = num_nodes;
    transmissionProb.resize(numNodes, 0.0);
    
    // Initialize token ring order
    tokenRingOrder = nodeIds;
    if (config.token_order == "pseudo_random") {
        // Shuffle the order (using a deterministic seed for reproducibility)
        random_shuffle(tokenRingOrder.begin(), tokenRingOrder.end());
    }
    // else: static order (already in nodeIds order)
    
    // Set initial token holder
    tokenID = config.initial_token_holder;
    tokenRingPosition = 0;
    for (int i = 0; i < tokenRingOrder.size(); i++) {
        if (tokenRingOrder[i] == tokenID) {
            tokenRingPosition = i;
            break;
        }
    }
    
    // Initialize fuzzy area
    FA_size = config.initial_FA;
    if (FA_size > numNodes) FA_size = numNodes;
    if (FA_size < 1) FA_size = 1;
    
    fuzzyArea.reset();
    for (int i = 0; i < FA_size && i < numNodes; i++) {
        fuzzyArea.set(tokenRingOrder[i]);
    }
    
    // Initialize transmission probabilities
    updateTransmissionProbabilities();
    
    // Start in fuzzy mode
    periodMode = FUZZY_MODE;
    currentStepCycles = config.silence_cycles; // Initial assumption
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
        cout << "FuzzyTokenChannelState initialized: numNodes=" << numNodes 
             << ", FA_size=" << FA_size 
             << ", tokenID=" << tokenID 
             << ", mode=" << (periodMode == FUZZY_MODE ? "FUZZY" : "FOCUSED") << endl;
    }
}

void FuzzyTokenChannelState::updateFuzzyArea(StepOutcome outcome) {
    int oldFA = FA_size;
    
    switch (outcome) {
        case OUTCOME_COLLISION:
            // Multiplicative decrease
            FA_size = (int)ceil(FA_size * config.FA_decrement_factor);
            if (FA_size < 1) FA_size = 1;
            totalCollisions++;
            break;
            
        case OUTCOME_SUCCESS:
            // No change on success
            totalSuccesses++;
            break;
            
        case OUTCOME_SILENCE:
            // Additive increase
            FA_size += config.FA_increment;
            if (FA_size > numNodes) FA_size = numNodes;
            totalSilences++;
            break;
    }
    
    // Rebuild fuzzy area bitset based on new FA_size
    fuzzyArea.reset();
    for (int i = 0; i < FA_size && i < numNodes; i++) {
        fuzzyArea.set(tokenRingOrder[i]);
    }
    
    // Update transmission probabilities
    updateTransmissionProbabilities();
    
    if (oldFA != FA_size) {
        static int fa_change_count = 0;
        fa_change_count++;
        const char* outcome_str = (outcome == OUTCOME_COLLISION) ? "COLLISION" : 
                                  (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
        cerr << "[AIMD-FA-UPDATE #" << fa_change_count << "] FA_size: " << oldFA << " -> " << FA_size 
             << " (outcome=" << outcome_str << ", collisions=" << totalCollisions 
             << ", successes=" << totalSuccesses << ", silences=" << totalSilences << ")" << endl;
    }
}

void FuzzyTokenChannelState::updateTransmissionProbabilities() {
    // Reset all probabilities
    for (int i = 0; i < numNodes; i++) {
        transmissionProb[i] = 0.0;
    }
    
    if (FA_size == 0) return;
    
    if (config.pi_type == "equal") {
        // Equal probability for all nodes in fuzzy area
        double p = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (fuzzyArea.test(i)) {
                transmissionProb[i] = p;
            }
        }
    } else if (config.pi_type == "gaussian") {
        // Gaussian distribution centered around token holder
        // For simplicity, use equal for now (can be extended)
        double p = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (fuzzyArea.test(i)) {
                transmissionProb[i] = p;
            }
        }
    }
}

void FuzzyTokenChannelState::advanceToken() {
    // Move to next node in ring order
    tokenRingPosition = (tokenRingPosition + 1) % tokenRingOrder.size();
    tokenID = tokenRingOrder[tokenRingPosition];
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
        cout << "Token advanced to node " << tokenID << endl;
    }
}

void FuzzyTokenChannelState::switchMode() {
    // Check thresholds
    double thr1_value = config.thr1;
    double thr2_value = config.thr2;
    
    if (config.thr_is_percentage) {
        thr1_value = config.thr1 * numNodes;
        thr2_value = config.thr2 * numNodes;
    }
    
    FuzzyTokenMode oldMode = periodMode;
    
    if (periodMode == FUZZY_MODE) {
        // Switch to focused if FA_size < thr1
        if (FA_size < thr1_value) {
            periodMode = FOCUSED_MODE;
            totalFocusedSteps++;
        } else {
            totalFuzzySteps++;
        }
    } else { // FOCUSED_MODE
        // Switch to fuzzy if FA_size > thr2
        if (FA_size > thr2_value) {
            periodMode = FUZZY_MODE;
            totalFuzzySteps++;
        } else {
            totalFocusedSteps++;
        }
    }
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH && oldMode != periodMode) {
        cout << "Mode switched: " << (oldMode == FUZZY_MODE ? "FUZZY" : "FOCUSED")
             << " -> " << (periodMode == FUZZY_MODE ? "FUZZY" : "FOCUSED")
             << " (FA=" << FA_size << ")" << endl;
    }
}

double FuzzyTokenChannelState::getTransmissionProbability(int nodeId) const {
    if (nodeId < 0 || nodeId >= numNodes) return 0.0;
    return transmissionProb[nodeId];
}

bool FuzzyTokenChannelState::isInFuzzyArea(int nodeId) const {
    if (nodeId < 0 || nodeId >= MAX_FUZZY_TOKEN_NODES) return false;
    bool result = fuzzyArea.test(nodeId);
    
    static int check_count = 0;
    if (++check_count % 1000 == 0 && GlobalParams::verbose_mode >= VERBOSE_LOW) {
        cerr << "[FUZZY-AREA-CHECK] nodeId=" << nodeId << " in FA? " << result 
             << " (FA_size=" << FA_size << ", tokenID=" << tokenID << ")" << endl;
    }
    
    return result;
}

// FuzzyTokenController implementation

void FuzzyTokenController::registerChannel(int channelId, const FuzzyTokenConfig& config, 
                                           int numNodes, const vector<int>& nodeIds) {
    if (channelStates.find(channelId) != channelStates.end()) {
        delete channelStates[channelId];
    }
    
    FuzzyTokenChannelState* state = new FuzzyTokenChannelState();
    state->initialize(config, numNodes, nodeIds);
    channelStates[channelId] = state;
    
    if (GlobalParams::verbose_mode >= VERBOSE_LOW) {
        cout << "FuzzyTokenController: Registered channel " << channelId 
             << " with " << numNodes << " nodes" << endl;
    }
}

FuzzyTokenChannelState* FuzzyTokenController::getChannelState(int channelId) {
    if (channelStates.find(channelId) == channelStates.end()) {
        return nullptr;
    }
    return channelStates[channelId];
}

void FuzzyTokenController::endStep(int channelId, StepOutcome outcome, int stepCycles) {
    FuzzyTokenChannelState* state = getChannelState(channelId);
    if (!state) return;
    
    // Log step end with transmission info
    static int step_count = 0;
    step_count++;
    const char* outcome_str = (outcome == OUTCOME_COLLISION) ? "COLLISION" : 
                              (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
    cerr << "[STEP-END #" << step_count << "] Channel " << channelId 
         << " outcome=" << outcome_str
         << ", active_transmitters=" << state->activeTransmittersThisStep
         << ", FA_size=" << state->FA_size << endl;
    
    // Update fuzzy area based on outcome
    state->updateFuzzyArea(outcome);
    
    // Advance token (happens at end of every step)
    state->advanceToken();
    
    // Check for mode switch
    state->switchMode();
    
    // Reset transmission counters for next step
    state->resetStepState();
    
    // Update step cycle count
    state->currentStepCycles = stepCycles;
}

void FuzzyTokenController::reset() {
    for (auto& pair : channelStates) {
        delete pair.second;
    }
    channelStates.clear();
}

void FuzzyTokenController::printStats(int channelId) {
    FuzzyTokenChannelState* state = getChannelState(channelId);
    if (!state) return;
    
    cout << "\n=== Fuzzy Token Statistics for Channel " << channelId << " ===" << endl;
    cout << "Total Collisions: " << state->totalCollisions << endl;
    cout << "Total Successes: " << state->totalSuccesses << endl;
    cout << "Total Silences: " << state->totalSilences << endl;
    cout << "Total Fuzzy Steps: " << state->totalFuzzySteps << endl;
    cout << "Total Focused Steps: " << state->totalFocusedSteps << endl;
    cout << "Final FA Size: " << state->FA_size << endl;
    cout << "Final Mode: " << (state->periodMode == FUZZY_MODE ? "FUZZY" : "FOCUSED") << endl;
    cout << "=================================================\n" << endl;
}

// Global transmission tracking methods
void FuzzyTokenChannelState::registerTransmission(int hubId) {
    transmittingHubsThisStep.insert(hubId);
    activeTransmittersThisStep = transmittingHubsThisStep.size();
    
    static int register_count = 0;
    register_count++;
    cerr << "[PREAMBLE-REGISTERED #" << register_count << "] Hub " << hubId 
         << " registered transmission (total active: " << activeTransmittersThisStep << ")" << endl;
}

void FuzzyTokenChannelState::resetStepState() {
    transmittingHubsThisStep.clear();
    activeTransmittersThisStep = 0;
}
