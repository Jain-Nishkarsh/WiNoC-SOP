/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the per-node Fuzzy Token helper implementation
 */

#include "FuzzyTokenNode.h"
#include "GlobalParams.h"
#include "Utils.h"
#include <cstdlib>

bool FuzzyTokenNode::shouldAttemptTransmission(bool hasData) {
    static int call_count = 0;
    call_count++;
    
    if (call_count <= 20 || call_count % 1000 == 0) {
        cerr << "[SHOULD-TX-CHECK #" << call_count << "] Node " << nodeId 
             << " hasData=" << hasData << endl;
    }
    
    if (!hasData) return false;
    
    FuzzyTokenChannelState* state = controller.getChannelState(channelId);
    if (!state) return false;
    
    FuzzyTokenMode mode = state->getMode();
    
    if (mode == FOCUSED_MODE) {
        // In focused mode, only token holder can transmit
        return (state->getTokenHolder() == nodeId);
    } else { // FUZZY_MODE
        // In fuzzy mode, nodes in fuzzy area transmit with probability p[i]
        if (!state->isInFuzzyArea(nodeId)) {
            return false;
        }
        
        double p = state->getTransmissionProbability(nodeId);
        
        // OPTION 1 FIX: Significantly increase probability for nodes with data
        // Original p=1/FA_size is too low (6.25% for 16 nodes)
        // Boost to configured minimum to enable actual transmission
        double min_p = state->config.min_transmission_prob;
        if (hasData && p < min_p) {
            p = min_p;  // Use configured minimum probability
        }
        
        double rand_val = (double)rand() / RAND_MAX;
        
        return (rand_val < p);
    }
}

void FuzzyTokenNode::startPreamble() {
    preambleSent = true;
    transmissionPending = true;
    nackReceived = false;
}

void FuzzyTokenNode::receiveNack() {
    nackReceived = true;
    transmissionPending = false;
    
    if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
        cout << "Node " << nodeId << " received NACK, aborting transmission" << endl;
    }
}

bool FuzzyTokenNode::isTokenHolder() {
    FuzzyTokenChannelState* state = controller.getChannelState(channelId);
    if (!state) return false;
    return (state->getTokenHolder() == nodeId);
}

bool FuzzyTokenNode::isInFuzzyArea() {
    FuzzyTokenChannelState* state = controller.getChannelState(channelId);
    if (!state) return false;
    return state->isInFuzzyArea(nodeId);
}

FuzzyTokenMode FuzzyTokenNode::getCurrentMode() {
    FuzzyTokenChannelState* state = controller.getChannelState(channelId);
    if (!state) return FUZZY_MODE; // default
    return state->getMode();
}

void FuzzyTokenNode::resetStepState() {
    transmissionPending = false;
    nackReceived = false;
    preambleSent = false;
    transmissionStartCycle = -1;
}
