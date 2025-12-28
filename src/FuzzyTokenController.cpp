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
#include "Hub.h"
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
    ready_bitmap.resize(numNodes, false);
    
    // Initialize ready-count trigger parameters
    ready_history_window = config.ready_history_window;
    fuzzy_ready_threshold = config.fuzzy_ready_threshold;
    focused_ready_threshold = config.focused_ready_threshold;
    fuzzy_consecutive_windows = config.fuzzy_consecutive_windows;
    focused_consecutive_windows = config.focused_consecutive_windows;
    
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
    
    periodMode = FOCUSED_MODE;
    
    currentStepCycles = 0;
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
        cout << "FuzzyTokenChannelState initialized: numNodes=" << numNodes 
             << ", FA_size=" << FA_size << ", tokenID=" << tokenID << endl;
    }

    // Initialize SHT
    sht.resize(numNodes);
    nodeToIndex.clear();
    for(int i=0; i<numNodes; i++) {
        nodeToIndex[nodeIds[i]] = i;
        sht[i].success_score = 0.0;
        sht[i].last_visit_cycle = 0;
    }
    last_smoothing_cycle = 0;
    tenure_cycles_remaining = 0;
    
    // Initialize SWJ debug metrics
    last_swj_score = 0.0;
    last_swj_success = 0.0;
    last_swj_age = 0;
    last_swj_target = -1;
}

void FuzzyTokenChannelState::updateFuzzyArea(StepOutcome outcome) {
    int oldFA = FA_size;
    
    switch (outcome) {
        case OUTCOME_COLLISION:
            totalCollisions++;
            FA_size = (int)ceil(FA_size * config.FA_decrement_factor);
            if (FA_size < 1) FA_size = 1;
            break;
            
        case OUTCOME_CONGESTION:
            totalCongestions++;
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
    // Reset probabilities
    std::fill(transmissionProb.begin(), transmissionProb.end(), 0.0);
    
    if (FA_size == 0) return;

    if (config.pi_type == PI_GAUSSIAN) {
        // Ensure cache exists
        if (gaussian_weights_cache.find(FA_size) == gaussian_weights_cache.end()) {
            vector<double> weights(FA_size);
            const double sigma = std::max(1.0, FA_size / 2.0);
            for (int k = 0; k < FA_size; ++k) {
                const int dist = std::abs(k - FA_size/2);
                weights[k] = std::exp(-(dist*dist) / (2.0 * sigma * sigma));
            }
            gaussian_weights_cache[FA_size] = weights;
        }

        // Assign weights to nodes
        const vector<double>& weights = gaussian_weights_cache[FA_size];
        const int n = (int)tokenRingOrder.size();
        double total_weight = 0.0;

        for (int k = 0; k < FA_size && k < n; ++k) {
            int idx = (tokenRingPosition + k - FA_size/2 + n) % n;
            int nodeId = tokenRingOrder[idx];
            
            transmissionProb[nodeId] = weights[k];
            total_weight += weights[k];
        }

        // Normalize Gaussian to sum to 1.0 initially
        if (total_weight > 0.0) {
            double scale = 1.0 / total_weight;
            for (int i = 0; i < numNodes; i++) {
                if (transmissionProb[i] > 0) transmissionProb[i] *= scale;
            }
        }
    } else {
        // PI_EQUAL (default)
        double p_uniform = 1.0 / FA_size;
        for (int i = 0; i < numNodes; i++) {
            if (fuzzyArea.test(i)) transmissionProb[i] = p_uniform;
        }
    }

    // Apply Minimum Transmission Probability
    // This ensures that even if the distribution assigns a low probability,
    // the node transmits with at least this probability.
    if (config.min_transmission_prob > 0.0) {
        for (int i = 0; i < numNodes; i++) {
            if (transmissionProb[i] > 0) {
                transmissionProb[i] = std::max(transmissionProb[i], config.min_transmission_prob);
            }
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
    
    if (nextReadyPos == -1) {
        // No ready hub found, advance sequentially
        tokenRingPosition = (tokenRingPosition + 1) % tokenRingOrder.size();
        tokenID = tokenRingOrder[tokenRingPosition];
    } else {
        // Ready-Aware Jump: Always jump to the next ready node in FOCUSED mode
        // No cooldown logic (C_jump) applied here, as per design requirements.
        tokenRingPosition = nextReadyPos;
        tokenID = tokenRingOrder[tokenRingPosition];
        
        if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
            int skipped = (nextReadyPos > oldPosition) ? 
                            nextReadyPos - oldPosition - 1 : 
                            tokenRingOrder.size() - oldPosition + nextReadyPos - 1;
            if (skipped > 0) {
                cerr << "[TOKEN-JUMP] " << tokenRingOrder[oldPosition] << " → " << tokenID 
                        << " (skipped " << skipped << " hubs)" << endl;
            }
        }
    }
    
    rebuildFuzzyArea();
    updateTransmissionProbabilities();
}

void FuzzyTokenChannelState::updateSHT(StepOutcome outcome, int source_id, int dst_id) {
    double alpha = 0.2;
    int dimX = GlobalParams::mesh_dim_x;

    // 1. Global Decay
    for (int i = 0; i < sht.size(); i++) {
        sht[i].success_score *= (1.0 - alpha);
        if (sht[i].success_score < 0.001) sht[i].success_score = 0;
    }

    // 2. Rewards (Spatial Halo)
    if (outcome == OUTCOME_SUCCESS && source_id != -1) {
        // Set Tenure Lock
        tenure_cycles_remaining = 1;

        // Source Reward
        if (nodeToIndex.count(source_id)) {
            sht[nodeToIndex[source_id]].success_score += alpha * 1.0;
        }

        // Destination Reward
        if (dst_id != -1 && nodeToIndex.count(dst_id)) {
            sht[nodeToIndex[dst_id]].success_score += alpha * 0.5;
        }

        // Neighbor Rewards
        for (int i = 0; i < numNodes; i++) {
            int nodeId = tokenRingOrder[i];
            if (nodeId == source_id || nodeId == dst_id) continue;
            if (!nodeToIndex.count(nodeId)) continue;

            int x_i = nodeId % dimX;
            int y_i = nodeId / dimX;

            bool is_neighbor = false;

            // Check distance to Source
            int x_src = source_id % dimX;
            int y_src = source_id / dimX;
            if (std::abs(x_i - x_src) + std::abs(y_i - y_src) == 1) is_neighbor = true;

            // Check distance to Dest
            if (!is_neighbor && dst_id != -1) {
                int x_dst = dst_id % dimX;
                int y_dst = dst_id / dimX;
                if (std::abs(x_i - x_dst) + std::abs(y_i - y_dst) == 1) is_neighbor = true;
            }

            if (is_neighbor) {
                sht[nodeToIndex[nodeId]].success_score += alpha * 0.2;
            }
        }
    }
}

void FuzzyTokenChannelState::advanceTokenSWJ() {
    // Phase 2: The Heuristic Score & Jump Target
    
    // 0. Tenure Lock Check
    if (tenure_cycles_remaining > 0) {
        tenure_cycles_remaining--;
        // Token stays at current node
        last_swj_target = tokenID;
        // Just for logging, we can keep the last score or set to 0
        return; 
    }

    // Current cycle T
    uint64_t T = step_start_cycle + currentStepCycles;

    // Update last_visit_cycle for the current token holder
    if (nodeToIndex.count(tokenID)) {
        sht[nodeToIndex[tokenID]].last_visit_cycle = T;
    }

    // Weights (Priority Scoring)
    double Ws = 5.0; // Weight for success (exploitation)
    double Wa = 4.0;  // Weight for age (exploration)
    double H_threshold = 1.0; // Stability threshold

    // 1. Calculate Score for Current Holder (Stability Score)
    double current_holder_score = 0.0;
    if (nodeToIndex.count(tokenID)) {
        int idx = nodeToIndex[tokenID];
        // Age is effectively 0 for current holder
        current_holder_score = Ws * sht[idx].success_score; 
    }

    // 2. Find the best node in the rest of the chip (The Challenger)
    int challenger_id = -1;
    double max_challenger_score = -1.0;

    for (int i = 0; i < numNodes; i++) {
        int nodeId = tokenRingOrder[i];
        if (nodeId == tokenID) continue; // Skip current holder
        if (nodeToIndex.find(nodeId) == nodeToIndex.end()) continue;
        
        int idx = nodeToIndex[nodeId];
        
        double exploitation = sht[idx].success_score;
        uint64_t age = (T > sht[idx].last_visit_cycle) ? (T - sht[idx].last_visit_cycle) : 0;
        double exploration = Wa * std::sqrt((double)age);
        
        double score = Ws * exploitation + exploration;
        
        if (score > max_challenger_score) {
            max_challenger_score = score;
            challenger_id = nodeId;
        } else if (score == max_challenger_score) {
             // Tie-Breaking: lowest ID
            if (challenger_id == -1 || nodeId < challenger_id) {
                challenger_id = nodeId;
            }
        }
    }

    // 3. The Hysteresis Decision
    int next_token_holder = tokenID;
    double winning_score = current_holder_score;

    if (challenger_id != -1 && max_challenger_score > (current_holder_score + H_threshold)) {
        next_token_holder = challenger_id;
        winning_score = max_challenger_score;
    }

    // Store debug metrics
    last_swj_score = winning_score;
    last_swj_target = next_token_holder;
    if (next_token_holder != -1 && nodeToIndex.count(next_token_holder)) {
        int idx = nodeToIndex[next_token_holder];
        last_swj_success = sht[idx].success_score;
        last_swj_age = (T > sht[idx].last_visit_cycle) ? (T - sht[idx].last_visit_cycle) : 0;
    } else {
        last_swj_success = 0.0;
        last_swj_age = 0;
    }
    
    int oldTokenID = tokenID;
    
    if (next_token_holder != tokenID) {
        // Find position of next_token_holder in tokenRingOrder
        for(int i=0; i<tokenRingOrder.size(); ++i) {
            if (tokenRingOrder[i] == next_token_holder) {
                tokenRingPosition = i;
                break;
            }
        }
        tokenID = next_token_holder;
    }
    // Else: Stay put, tokenRingPosition remains same
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH && oldTokenID != tokenID) {
        cout << "[SWJ-JUMP] " << oldTokenID << " -> " << tokenID << " (Score: " << winning_score << ")" << endl;
    }
    
    rebuildFuzzyArea();
    updateTransmissionProbabilities();
}

void FuzzyTokenChannelState::switchMode(StepOutcome outcome) {
    double thr1_value = config.thr_is_percentage ? ceil(config.thr1 * numNodes) : config.thr1;
    double thr2_value = config.thr_is_percentage ? floor(config.thr2 * numNodes) : config.thr2;
    
    if (periodMode == FUZZY_MODE) {
        // Transition: Collision AND (fuzzyArea < thr2)
        if (outcome == OUTCOME_COLLISION && FA_size < thr2_value) {
            periodMode = FOCUSED_MODE;
            totalSwitchesToFocused++;
        }
    } else {
        // Transition: Silence AND (fuzzyArea > thr1)
        if (outcome == OUTCOME_SILENCE && FA_size > thr1_value) {
            periodMode = FUZZY_MODE;
            totalSwitchesToFuzzy++;
        }
    }
}

double FuzzyTokenChannelState::getTransmissionProbability(int nodeId) const {
    if (nodeId < 0 || nodeId >= numNodes) return 0.0;
    return transmissionProb[nodeId];
}

bool FuzzyTokenChannelState::isInFuzzyArea(int nodeId) const {
    if (nodeId < 0 || nodeId >= MAX_FUZZY_TOKEN_NODES) return false;
    return fuzzyArea.test(nodeId);
}

void FuzzyTokenChannelState::registerHub(Hub* hub) {
    for (auto h : registeredHubs) {
        if (h == hub) return;
    }
    registeredHubs.push_back(hub);
}

void FuzzyTokenChannelState::perform_control_minislot(int currentCycle, bool enable_polling) {
    if (lastControlStepCycle == currentCycle) return;

    // Reset ready bitmap
    fill(ready_bitmap.begin(), ready_bitmap.end(), false);
    
    int readyCount = 0;
    
    if (enable_polling) {
        // Poll all registered hubs
        for (auto hub : registeredHubs) {
            // Check if hub has data for this channel
            if (hub->init.find(channelID) != hub->init.end()) {
                 if (!hub->init[channelID]->buffer_tx.IsEmpty()) {
                     int hubID = hub->local_id;
                     // Find index in tokenRingOrder
                     for(int i=0; i<tokenRingOrder.size(); ++i) {
                         if (tokenRingOrder[i] == hubID) {
                             ready_bitmap[i] = true;
                             readyCount++;
                             break;
                         }
                     }
                 }
            }
        }
    }
    
    lastControlStepCycle = currentCycle;
    step_start_cycle = currentCycle;
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH && enable_polling) {
        cout << "Control Minislot for Channel " << channelID << " at cycle " << currentCycle 
             << ": " << readyCount << " ready nodes." << endl;
    }
}

// FuzzyTokenController implementation

void FuzzyTokenController::registerChannel(int channelId, const FuzzyTokenConfig& config, 
                                           int numNodes, const vector<int>& nodeIds,
                                           const string& policy) {
    if (channelStates.find(channelId) != channelStates.end()) {
        delete channelStates[channelId];
    }
    
    FuzzyTokenChannelState* state = new FuzzyTokenChannelState();
    state->channelID = channelId;
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

void FuzzyTokenController::endStep(int channelId, StepOutcome outcome, int stepCycles, int dst_id) {
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

    // --- Detailed CSV Logging Logic ---
    
    // 1. Capture Pre-Update State
    int fa_old_size = state->FA_size;
    int current_holder = state->tokenID;
    FuzzyTokenMode mode_before_update = state->periodMode;
    
    // Construct Node Lists (FA and Ready) BEFORE update
    stringstream fa_nodes_ss;
    for(int i=0; i<state->numNodes; ++i) {
        fa_nodes_ss << (state->isInFuzzyArea(i) ? "1" : "0");
    }
    string fa_nodes = fa_nodes_ss.str();
    if (fa_nodes.empty()) fa_nodes = "-";

    stringstream tx_attempt_ss;
    bool first = true;
    int success_node = -1;
    for(int node : state->transmittingHubsThisStep) {
        if(!first) tx_attempt_ss << "|";
        tx_attempt_ss << node;
        first = false;
        if (outcome == OUTCOME_SUCCESS) success_node = node;
    }
    string tx_attempt_nodes = tx_attempt_ss.str();
    if (tx_attempt_nodes.empty()) tx_attempt_nodes = "-";

    stringstream ready_nodes_ss;
    for(int i=0; i<state->numNodes; ++i) {
        ready_nodes_ss << (state->isHubReady(i) ? "1" : "0");
    }
    string ready_nodes = ready_nodes_ss.str();
    if (ready_nodes.empty()) ready_nodes = "-";

    // 2. Update State
    // Always update FA_size to track contention (needed for baseline FUZZY_TOKEN mode switching)
    state->updateFuzzyArea(outcome);
    
    // Increment Step ID
    state->step_id++;
    
    // Record FA_size statistics for all policies
    state->FA_size_histogram[state->FA_size]++;
    
    // Count steps for all policies
    if (state->periodMode == FUZZY_MODE) {
        state->totalFuzzySteps++;
    } else {
        state->totalFocusedSteps++;
    }
    
    // Update currentStepCycles BEFORE SWJ logic to ensure correct age calculation
    state->currentStepCycles = stepCycles;

    bool useJumpFeatures = (state->macPolicy == FUZZY_RAJ);
    bool usePlusFeatures = (state->macPolicy == FUZZY_RCT || state->macPolicy == FUZZY_RAJ);
    bool useSWJ = (state->macPolicy == FUZZY_SWJ);
    bool inFocusedMode = (state->periodMode == FOCUSED_MODE);
    
    if (useSWJ) {
        int success_node_for_sht = -1;
        if (outcome == OUTCOME_SUCCESS) {
             if (state->transmittingHubsThisStep.size() == 1) {
                 success_node_for_sht = *state->transmittingHubsThisStep.begin();
             }
        }
        state->updateSHT(outcome, success_node_for_sht, dst_id);
        state->advanceTokenSWJ();
        state->switchMode(outcome);
    } else if (useJumpFeatures && inFocusedMode) {
        state->advanceTokenSmart();
    } else {
        state->advanceToken();
    }
    
    if (!usePlusFeatures && !useSWJ) {
        state->switchMode(outcome);
    }

    // 3. Log to CSV (Moved to end to capture next token state)
    if (GlobalParams::csv_log_enabled && GlobalParams::csv_mac_log_stream.is_open()) {
        const char* outcome_str = (outcome == OUTCOME_COLLISION) ? "COLLISION" : 
                                  (outcome == OUTCOME_CONGESTION) ? "CONGESTION" : 
                                  (outcome == OUTCOME_SUCCESS) ? "SUCCESS" : "SILENCE";
        const char* mode_str = (mode_before_update == FUZZY_MODE) ? "FUZZY" : "FOCUSED";
        
        GlobalParams::csv_mac_log_stream 
            << state->step_start_cycle << ","
            << state->step_id - 1 << "," // Use previous step ID as we incremented it
            << outcome_str << ","
            << stepCycles << ","
            << current_holder << ","
            << mode_str << ","
            << fa_old_size << ","
            << fa_nodes << ","
            << tx_attempt_nodes << ","
            << success_node << ","
            << ready_nodes << ","
            << state->last_swj_target << ","
            << state->last_swj_score << ","
            << state->last_swj_success << ","
            << state->last_swj_age
            << endl;
    }
    
    state->resetStepState();
    state->resetReadyBitmap();
}

void FuzzyTokenChannelState::rebuildFuzzyArea() {
    fuzzyArea.reset();
    if (numNodes <= 0 || tokenRingOrder.empty()) return;

    if (macPolicy == FUZZY_SWJ) {
        // Blob FA Logic (Manhattan Distance)
        int dimX = GlobalParams::mesh_dim_x;
        // int dimY = GlobalParams::mesh_dim_y; // Unused
        
        int x_token = tokenID % dimX;
        int y_token = tokenID / dimX;
        
        // Calculate radius based on FA_size (Area ~ 2*r^2)
        // r = sqrt(FA_size / 2)
        int radius = (int)std::sqrt(FA_size / 2.0);
        
        for (int i = 0; i < numNodes; i++) {
            int x_i = i % dimX;
            int y_i = i / dimX;
            
            int dist = std::abs(x_token - x_i) + std::abs(y_token - y_i);
            
            if (dist <= radius) {
                fuzzyArea.set(i);
            }
        }
    } else {
        // Original Ring Logic
        int n = (int)tokenRingOrder.size();
        int center = tokenRingPosition;
        int half = FA_size / 2;
        for (int k = -half; k < -half + FA_size; ++k) {
            int idx = (center + k + n) % n;
            int nodeId = tokenRingOrder[idx];
            fuzzyArea.set(nodeId);
        }
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
    
    cout << "5. Number of FOCUSED -> FUZZY transitions: " << state->totalSwitchesToFuzzy << endl;
    cout << "6. Number of FUZZY -> FOCUSED transitions: " << state->totalSwitchesToFocused << endl;
    
    double totalSteps = state->totalFuzzySteps + state->totalFocusedSteps;
    double fuzzyFraction = (totalSteps > 0) ? (double)state->totalFuzzySteps / totalSteps : 0.0;
    double focusedFraction = (totalSteps > 0) ? (double)state->totalFocusedSteps / totalSteps : 0.0;
    
    cout << "7. Fraction of time in FUZZY mode: " << fuzzyFraction << endl;
    cout << "8. Fraction of time in FOCUSED mode: " << focusedFraction << endl;
    
    double avgReadyHubs = (state->totalReadyChecks > 0) ? 
                          (double)state->accumulatedReadyHubs / state->totalReadyChecks : 0.0;
    cout << "9. Average number of ready hubs: " << avgReadyHubs << endl;

    cout << "10. Histogram of FA_size:" << endl;
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
    cerr << "ERROR: setHubReady called outside control minislot! This function is deprecated." << endl;
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
void FuzzyTokenChannelState::updateReadyHistory() {
    int ready_count = countReadyHubs();
    ready_history.push_back(ready_count);
    
    accumulatedReadyHubs += ready_count;
    totalReadyChecks++;
    
    if (ready_history.size() > (size_t)ready_history_window) {
        ready_history.pop_front();
    }
}

bool FuzzyTokenChannelState::shouldSwitchToFuzzy() const {
    if (ready_history.size() < (size_t)fuzzy_consecutive_windows) {
        return false;
    }
    
    for (size_t i = ready_history.size() - fuzzy_consecutive_windows; i < ready_history.size(); i++) {
        if (ready_history[i] > fuzzy_ready_threshold) {
            return false;
        }
    }
    
    return true;
}

bool FuzzyTokenChannelState::shouldSwitchToFocused() const {
    if (ready_history.size() < (size_t)focused_consecutive_windows) {
        return false;
    }
    
    for (size_t i = ready_history.size() - focused_consecutive_windows; i < ready_history.size(); i++) {
        if (ready_history[i] < focused_ready_threshold) {
            return false;
        }
    }
    
    return true;
}

void FuzzyTokenChannelState::checkAndSwitchModeProactive() {
    if (shouldSwitchToFuzzy() && periodMode != FUZZY_MODE) {
        periodMode = FUZZY_MODE;
        totalSwitchesToFuzzy++;
        if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
            cerr << "[MODE-SWITCH] FOCUSED -> FUZZY" << endl;
        }
    } else if (shouldSwitchToFocused() && periodMode != FOCUSED_MODE) {
        periodMode = FOCUSED_MODE;
        totalSwitchesToFocused++;
        if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
            cerr << "[MODE-SWITCH] FUZZY -> FOCUSED" << endl;
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
