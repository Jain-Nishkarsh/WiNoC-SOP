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
    EMA_ALPHA = config.ema_alpha;
    numNodes = num_nodes;
    transmissionProb.resize(numNodes, 0.0);
    ready_bitmap.resize(numNodes, false);
    
    // Initialize ready-count trigger parameters
    // ready_history_window = config.ready_history_window;
    // fuzzy_ready_threshold = config.fuzzy_ready_threshold;
    // focused_ready_threshold = config.focused_ready_threshold;
    // fuzzy_consecutive_windows = config.fuzzy_consecutive_windows;
    // focused_consecutive_windows = config.focused_consecutive_windows;
    
    // Initialize ready-aware jump parameters
    jump_cooldown = config.jump_cooldown;
    cycles_since_last_jump = config.jump_cooldown + 1;
    
    // Initialize token ring order
    tokenRingOrder = nodeIds;
    if (config.token_order == "pseudo_random") {
        random_shuffle(tokenRingOrder.begin(), tokenRingOrder.end());
    }
    
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
    rebuildFuzzyArea();
    updateTransmissionProbabilities();
    
    periodMode = FUZZY_MODE;
    currentStepCycles = 0;
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
        cout << "FuzzyTokenChannelState initialized: numNodes=" << numNodes 
             << ", FA_size=" << FA_size << ", tokenID=" << tokenID << endl;
    }
}

void FuzzyTokenChannelState::updateFuzzyArea(StepOutcome outcome) {
    int oldFA = FA_size;
    
    switch (outcome) {
        case OUTCOME_COLLISION:
        case OUTCOME_CONGESTION:
            FA_size = (int)ceil(FA_size * config.FA_decrement_factor);
            if (FA_size < 1) FA_size = 1;
            if (outcome == OUTCOME_COLLISION) totalCollisions++;
            else totalCongestions++;
            break;
            
        case OUTCOME_SUCCESS:
            totalSuccesses++;
            break;
            
        case OUTCOME_SILENCE:
            FA_size += config.FA_increment;
            if (FA_size > numNodes) FA_size = numNodes;
            totalSilences++;
            break;
    }
    
    rebuildFuzzyArea();
    updateTransmissionProbabilities();
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH && oldFA != FA_size) {
        const char* outcome_str = (outcome == OUTCOME_COLLISION) ? "COLLISION" : 
                                  (outcome == OUTCOME_CONGESTION) ? "CONGESTION" : 
                                  (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
        cerr << "[FA-UPDATE] " << oldFA << " -> " << FA_size << " (" << outcome_str << ")" << endl;
    }
}

void FuzzyTokenChannelState::updateTransmissionProbabilities() {
    for (int i = 0; i < numNodes; i++) {
        transmissionProb[i] = 0.0;
    }
    
    if (FA_size == 0) return;
    
    if (config.pi_type == PI_EQUAL) {
        double p_uniform = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (fuzzyArea.test(i)) transmissionProb[i] = p_uniform;
        }
    } else if (config.pi_type == PI_GAUSSIAN) {
        // Check cache first
        if (gaussian_weights_cache.find(FA_size) == gaussian_weights_cache.end()) {
            // Compute and cache weights for this FA_size
            vector<double> weights;
            weights.resize(FA_size);
            const double sigma = std::max(1.0, FA_size / 2.0);
            
            for (int k = 0; k < FA_size; ++k) {
                const int dist = std::abs(k - FA_size/2);
                weights[k] = std::exp(-(dist*dist) / (2.0 * sigma * sigma));
            }
            gaussian_weights_cache[FA_size] = weights;
        }

        const vector<double>& weights = gaussian_weights_cache[FA_size];
        int centerIdx = tokenRingPosition;
        double sumw = 0.0;
        const int n = (int)tokenRingOrder.size();
        
        for (int k = 0; k < FA_size && k < n; ++k) {
            int idx = (centerIdx + k - FA_size/2 + n) % n;
            int nodeId = tokenRingOrder[idx];
            double w = weights[k];
            transmissionProb[nodeId] = w;
            sumw += w;
        }
        
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
        
        if (sumAfter > 0.0) {
            const double invSumAfter = 1.0 / sumAfter;
            for (int i = 0; i < numNodes; i++) {
                if (transmissionProb[i] > 0) transmissionProb[i] *= invSumAfter;
            }
        }
    } else {
        double p_uniform = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (transmissionProb[i] > 0) transmissionProb[i] = p_uniform;
        }
    }
}

void FuzzyTokenChannelState::advanceToken() {
    tokenRingPosition = (tokenRingPosition + 1) % tokenRingOrder.size();
    tokenID = tokenRingOrder[tokenRingPosition];
    rebuildFuzzyArea();
    updateTransmissionProbabilities();
}

int FuzzyTokenChannelState::findNextReadyToken(int currentPos) const {
    for (int offset = 1; offset <= numNodes; offset++) {
        int nextPos = (currentPos + offset) % tokenRingOrder.size();
        int nextHubId = tokenRingOrder[nextPos];
        if (isHubReady(nextHubId)) return nextPos;
    }
    return -1;
}

void FuzzyTokenChannelState::advanceTokenSmart() {
    int oldPosition = tokenRingPosition;
    int nextReadyPos = findNextReadyToken(tokenRingPosition);
    
    cycles_since_last_jump++;
    
    if (nextReadyPos == -1) {
        // No ready hub found, advance sequentially
        tokenRingPosition = (tokenRingPosition + 1) % tokenRingOrder.size();
        tokenID = tokenRingOrder[tokenRingPosition];
    } else {
        int nextSequentialPos = (oldPosition + 1) % tokenRingOrder.size();
        bool wouldJump = (nextReadyPos != nextSequentialPos);
        
        if (wouldJump && cycles_since_last_jump < jump_cooldown) {
            // Jump cooldown active, advance sequentially instead
            tokenRingPosition = nextSequentialPos;
            tokenID = tokenRingOrder[tokenRingPosition];
        } else {
            // Jump to ready hub
            tokenRingPosition = nextReadyPos;
            tokenID = tokenRingOrder[tokenRingPosition];
            
            if (wouldJump) {
                cycles_since_last_jump = 0;
                if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
                    int skipped = (nextReadyPos > oldPosition) ? 
                                 nextReadyPos - oldPosition - 1 : 
                                 tokenRingOrder.size() - oldPosition + nextReadyPos - 1;
                    cerr << "[TOKEN-JUMP] " << tokenRingOrder[oldPosition] << " → " << tokenID 
                         << " (skipped " << skipped << " hubs)" << endl;
                }
            }
        }
    }
    
    rebuildFuzzyArea();
    updateTransmissionProbabilities();
}

void FuzzyTokenChannelState::switchMode() {
    double thr1_value = config.thr_is_percentage ? config.thr1 * numNodes : config.thr1;
    double thr2_value = config.thr_is_percentage ? config.thr2 * numNodes : config.thr2;
    
    if (periodMode == FUZZY_MODE) {
        if (FA_size < thr1_value) {
            periodMode = FOCUSED_MODE;
            totalFocusedSteps++;
        } else {
            totalFuzzySteps++;
        }
    } else {
        if (FA_size > thr2_value) {
            periodMode = FUZZY_MODE;
            totalFuzzySteps++;
        } else {
            totalFocusedSteps++;
        }
    }
    
    FA_size_histogram[FA_size]++;
}

double FuzzyTokenChannelState::getTransmissionProbability(int nodeId) const {
    if (nodeId < 0 || nodeId >= numNodes) return 0.0;
    return transmissionProb[nodeId];
}

bool FuzzyTokenChannelState::isInFuzzyArea(int nodeId) const {
    if (nodeId < 0 || nodeId >= MAX_FUZZY_TOKEN_NODES) return false;
    return fuzzyArea.test(nodeId);
}

// FuzzyTokenController implementation

void FuzzyTokenController::registerChannel(int channelId, const FuzzyTokenConfig& config, 
                                           int numNodes, const vector<int>& nodeIds,
                                           const string& policy) {
    if (channelStates.find(channelId) != channelStates.end()) {
        delete channelStates[channelId];
    }
    
    FuzzyTokenChannelState* state = new FuzzyTokenChannelState();
    state->initialize(config, numNodes, nodeIds);
    state->macPolicy = policy;
    channelStates[channelId] = state;
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
    if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
        static int step_count = 0;
        step_count++;
        const char* outcome_str = (outcome == OUTCOME_COLLISION) ? "COLLISION" : 
                                  (outcome == OUTCOME_CONGESTION) ? "CONGESTION" : 
                                  (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
        cerr << "[STEP-END] Channel " << channelId << " outcome=" << outcome_str << endl;
    }
    
    // Always update FA_size to track contention (needed for baseline FUZZY_TOKEN mode switching)
    // FA_size is used in FUZZY mode for fuzzy area, and checked by switchMode() for reactive switching
    state->updateFuzzyArea(outcome);
    
    bool useJumpFeatures = (state->macPolicy == FUZZY_RAJ);
    bool usePlusFeatures = (state->macPolicy == FUZZY_RCT || state->macPolicy == FUZZY_RAJ);
    bool inFocusedMode = (state->periodMode == FOCUSED_MODE);
    
    if (useJumpFeatures && inFocusedMode) {
        state->advanceTokenSmart();
    } else {
        state->advanceToken();
    }
    
    if (!usePlusFeatures) {
        state->switchMode();
    }
    
    state->resetStepState();
    state->resetReadyBitmap();
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
    
    cout << "5. Histogram of FA_size:" << endl;
    if (state->FA_size_histogram.empty()) {
        cout << "   (No FA_size data recorded)" << endl;
    } else {
        int maxFA = 0;
        for (const auto& entry : state->FA_size_histogram) {
            if (entry.first > maxFA) maxFA = entry.first;
        }
        
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
}

void FuzzyTokenChannelState::resetStepState() {
    transmittingHubsThisStep.clear();
    activeTransmittersThisStep = 0;
}

// Ready bitmap management implementations
void FuzzyTokenChannelState::setHubReady(int hubId, bool isReady) {
    if (hubId >= 0 && hubId < numNodes) {
        ready_bitmap[hubId] = isReady;
    }
}

bool FuzzyTokenChannelState::isHubReady(int hubId) const {
    if (hubId >= 0 && hubId < numNodes) {
        return ready_bitmap[hubId];
    }
    return false;
}

void FuzzyTokenChannelState::resetReadyBitmap() {
    std::fill(ready_bitmap.begin(), ready_bitmap.end(), false);
}

int FuzzyTokenChannelState::countReadyHubs() const {
    int count = 0;
    for (bool ready : ready_bitmap) {
        if (ready) count++;
    }
    return count;
}

void FuzzyTokenChannelState::logReadyBitmap(int currentCycle) const {
    if (GlobalParams::verbose_mode < VERBOSE_HIGH) return;
    
    cerr << "[READY-BITMAP] Cycle " << currentCycle << ": ";
    for (int i = 0; i < numNodes; i++) {
        cerr << (ready_bitmap[i] ? "1" : "0");
    }
    cerr << endl;
}

// Ready-count trigger implementations
void FuzzyTokenChannelState::update_ema(double ready_count, int num_hubs) {
    double ready_frac = double(ready_count) / double(num_hubs);

    if (!ema_initialized) {
        // initialize EMA to the first observation to avoid startup bias
        ema_ready = ready_frac;
        ema_initialized = 1;
    } else {
        ema_ready = (1.0 - EMA_ALPHA) * ema_ready + EMA_ALPHA * ready_frac;
    }
}

void FuzzyTokenChannelState::updateReadyStats() {
    int ready_count = countReadyHubs();
    
    // Update EMA
    update_ema((double)ready_count, numNodes);

    // Increment dwell counter
    dwell_counter++;
}

bool FuzzyTokenChannelState::shouldSwitchToFuzzy() const {
    // Check dwell time
    if (dwell_counter < D_MIN) return false;

    // Check EMA threshold
    if (ema_ready <= ALPHA_DOWN) return true;

    return false;
}

bool FuzzyTokenChannelState::shouldSwitchToFocused() const {
    // Check dwell time
    if (dwell_counter < D_MIN) return false;

    // Check EMA threshold
    if (ema_ready >= ALPHA_UP) return true;

    return false;
}

void FuzzyTokenChannelState::checkAndSwitchModeProactive() {
    if (shouldSwitchToFuzzy() && periodMode != FUZZY_MODE) {
        periodMode = FUZZY_MODE;
        dwell_counter = 0; // Reset dwell counter
        if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
            cerr << "[MODE-SWITCH] FOCUSED -> FUZZY (EMA=" << ema_ready << ")" << endl;
        }
    } else if (shouldSwitchToFocused() && periodMode != FOCUSED_MODE) {
        periodMode = FOCUSED_MODE;
        dwell_counter = 0; // Reset dwell counter
        if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
            cerr << "[MODE-SWITCH] FUZZY -> FOCUSED (EMA=" << ema_ready << ")" << endl;
        }
    }
}

void FuzzyTokenController::printAllStats() {
    if (channelStates.empty()) return;
    
    cout << "\n========================================================" << endl;
    cout << "         FUZZY TOKEN PROTOCOL STATISTICS" << endl;
    cout << "========================================================" << endl;
    
    for (const auto& entry : channelStates) {
        printStats(entry.first);
    }
    
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
