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
#include "Target.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

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
    
    // Initialize fuzzy area (centered later around current token holder)
    FA_size = config.initial_FA;
    if (FA_size > numNodes) FA_size = numNodes;
    if (FA_size < 1) FA_size = 1;
    rebuildFuzzyArea();
    
    // Initialize transmission probabilities
    updateTransmissionProbabilities();
    
    // Start in fuzzy mode
    periodMode = FUZZY_MODE;
    currentStepCycles = 0; // Will be set based on actual step outcomes
    
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
        case OUTCOME_CONGESTION:
            // Treat TX-side congestion like a collision for AIMD (multiplicative decrease)
            FA_size = (int)ceil(FA_size * config.FA_decrement_factor);
            if (FA_size < 1) FA_size = 1;
            totalCongestions++;
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
    
    // Rebuild fuzzy area centered around token holder
    rebuildFuzzyArea();
    
    // Update transmission probabilities
    updateTransmissionProbabilities();
    
    if (oldFA != FA_size) {
        static int fa_change_count = 0;
        fa_change_count++;
        const char* outcome_str = (outcome == OUTCOME_COLLISION) ? "COLLISION" : 
                                  (outcome == OUTCOME_CONGESTION) ? "CONGESTION" : 
                                  (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
        cerr << "[AIMD-FA-UPDATE #" << fa_change_count << "] FA_size: " << oldFA << " -> " << FA_size 
             << " (outcome=" << outcome_str << ", collisions=" << totalCollisions 
             << ", congestions=" << totalCongestions
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
        double p_uniform = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (fuzzyArea.test(i)) transmissionProb[i] = p_uniform;
        }
    } else if (config.pi_type == "gaussian") {
        // Gaussian distribution centered around token holder position on the ring
        // sigma heuristic: FA_size/2
        const double sigma = std::max(1.0, FA_size / 2.0);
        int centerIdx = tokenRingPosition;
        // Assign weights based on circular distance on ring order
        double sumw = 0.0;
        const int n = (int)tokenRingOrder.size();
        for (int k = 0; k < FA_size && k < n; ++k) {
            int idx = (centerIdx + k - FA_size/2 + n) % n;
            int nodeId = tokenRingOrder[idx];
            const int dist = std::abs(k - FA_size/2);
            const double w = std::exp(-(dist*dist) / (2.0 * sigma * sigma));
            transmissionProb[nodeId] = w;
            sumw += w;
        }
        // Normalize and apply min_transmission_prob if configured
        const double minp = std::max(0.0, config.min_transmission_prob);
        double sumAfter = 0.0;
        if (sumw > 0.0) {
            const double invSum = 1.0 / sumw;
            for (int i = 0; i < numNodes; i++) {
                if (transmissionProb[i] > 0) {
                    transmissionProb[i] *= invSum;
                    if (transmissionProb[i] < minp) transmissionProb[i] = minp;
                    sumAfter += transmissionProb[i];
                }
            }
        }
        // Renormalize to 1 over FA
        if (sumAfter > 0.0) {
            const double invSumAfter = 1.0 / sumAfter;
            for (int i = 0; i < numNodes; i++) {
                if (transmissionProb[i] > 0) transmissionProb[i] *= invSumAfter;
            }
        }
    } else {
        // Fallback to equal if unknown setting
        double p_uniform = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (transmissionProb[i] > 0) {
                transmissionProb[i] = p_uniform;
            }
        }
    }
}

void FuzzyTokenChannelState::advanceToken() {
    // Move to next node in ring order
    tokenRingPosition = (tokenRingPosition + 1) % tokenRingOrder.size();
    tokenID = tokenRingOrder[tokenRingPosition];
    // Recenter FA around new token holder
    rebuildFuzzyArea();
    // Update probabilities with new center
    updateTransmissionProbabilities();

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
        // Switch to FOCUSED when FA_size DROPS BELOW thr1 (low contention narrowed FA)
        if (FA_size < thr1_value) {
            periodMode = FOCUSED_MODE;
            totalFocusedSteps++;
        } else {
            totalFuzzySteps++;
        }
    } else { // FOCUSED_MODE
        // Switch back to FUZZY when FA_size EXCEEDS thr2 (contention reduced, can expand FA)
        if (FA_size > thr2_value) {
            periodMode = FUZZY_MODE;
            totalFuzzySteps++;
        } else {
            totalFocusedSteps++;
        }
    }
    
    // Update FA_size histogram
    FA_size_histogram[FA_size]++;
    
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
                              (outcome == OUTCOME_CONGESTION) ? "CONGESTION" : 
                              (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
    cerr << "[STEP-END #" << step_count << "] Channel " << channelId 
        << " outcome=" << outcome_str
        << ", active_transmitters=" << state->activeTransmittersThisStep
        << ", FA_size=" << state->FA_size
        << ", collisions=" << state->totalCollisions
        << ", congestions=" << state->totalCongestions
        << ", successes=" << state->totalSuccesses
        << ", silences=" << state->totalSilences
        << endl;
    
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

void FuzzyTokenChannelState::rebuildFuzzyArea() {
    fuzzyArea.reset();
    if (numNodes <= 0 || tokenRingOrder.empty()) return;
    int n = (int)tokenRingOrder.size();
    int center = tokenRingPosition;
    int half = FA_size / 2;
    for (int k = -half; k < -half + FA_size; ++k) {
        int idx = (center + k + n) % n;
        int nodeId = tokenRingOrder[idx];
        fuzzyArea.set(nodeId);
    }
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
    cout << "1. Number of collisions detected: " << state->totalCollisions << endl;
    cout << "2. Number of congestion events (TX enqueue-full): " << state->totalCongestions << endl;
    cout << "3. Number of steps for which focused mode is used: " << state->totalFocusedSteps << endl;
    cout << "4. Number of steps for which fuzzy mode is used: " << state->totalFuzzySteps << endl;
    
    // Print FA_size histogram
    cout << "5. Histogram of FA_size:" << endl;
    if (state->FA_size_histogram.empty()) {
        cout << "   (No FA_size data recorded)" << endl;
    } else {
        // Find the max FA_size and max count for formatting
        int maxFA = 0;
        int maxCount = 0;
        for (const auto& entry : state->FA_size_histogram) {
            if (entry.first > maxFA) maxFA = entry.first;
            if (entry.second > maxCount) maxCount = entry.second;
        }
        
        // Print histogram
        cout << "   FA_size | Count" << endl;
        cout << "   --------+------" << endl;
        for (int fa = 1; fa <= maxFA; fa++) {
            if (state->FA_size_histogram.find(fa) != state->FA_size_histogram.end()) {
                int count = state->FA_size_histogram.at(fa);
                cout << "   " << setw(7) << fa << " | " << setw(6) << count << endl;
            }
        }
    }
    
    cout << "\nAdditional Information:" << endl;
    cout << "  Total Successes: " << state->totalSuccesses << endl;
    cout << "  Total Silences: " << state->totalSilences << endl;
    cout << "  Final FA Size: " << state->FA_size << endl;
    cout << "  Final Mode: " << (state->periodMode == FUZZY_MODE ? "FUZZY" : "FOCUSED") << endl;
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

void FuzzyTokenController::printAllStats() {
    if (channelStates.empty()) {
        return; // No fuzzy token channels configured
    }
    
    cout << "\n" << endl;
    cout << "========================================================" << endl;
    cout << "         FUZZY TOKEN PROTOCOL STATISTICS" << endl;
    cout << "========================================================" << endl;
    
    for (const auto& entry : channelStates) {
        int channelId = entry.first;
        printStats(channelId);
    }
    
    // Add wireless buffer statistics
    cout << "\n=== Wireless Buffer Statistics ===" << endl;
    int total_attempts = Target::getTotalWirelessRxAttempts();
    int total_success = Target::getTotalWirelessRxSuccess();
    int total_dropped = Target::getTotalWirelessRxDropped();
    
    cout << "Total wireless RX attempts: " << total_attempts << endl;
    cout << "Total wireless RX success: " << total_success << endl;
    cout << "Total wireless RX dropped (buffer overflow): " << total_dropped << endl;
    
    if (total_attempts > 0) {
        double drop_rate = (100.0 * total_dropped) / total_attempts;
        double success_rate = (100.0 * total_success) / total_attempts;
        cout << "Drop rate: " << drop_rate << "%" << endl;
        cout << "Success rate: " << success_rate << "%" << endl;
    }
    cout << "========================================" << endl;
}
