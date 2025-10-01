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
