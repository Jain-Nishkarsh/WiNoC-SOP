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
             << " hasData=" << hasData 
             << " decisionMade=" << transmissionDecisionMade 
             << " cachedDecision=" << shouldTransmitThisStep << endl;
    }
    
    // If we already made a decision this step, return the cached result
    if (transmissionDecisionMade) {
        return shouldTransmitThisStep;
    }
    
    // Make the decision once per step
    bool decision = false;
    
    if (!hasData) {
        decision = false;
    } else {
        FuzzyTokenChannelState* state = controller.getChannelState(channelId);
        if (!state) {
            decision = false;
        } else {
            FuzzyTokenMode mode = state->getMode();
            
            if (mode == FOCUSED_MODE) {
                // In focused mode, only token holder can transmit; no probability involved
                decision = (state->getTokenHolder() == nodeId);
            } else { // FUZZY_MODE
                // In fuzzy mode, nodes in fuzzy area transmit with probability p[i]
                if (!state->isInFuzzyArea(nodeId)) {
                    decision = false;
                } else {
                    double p = state->getTransmissionProbability(nodeId);
                    
                    // Apply optional probability floor only for equal PI to prevent stalls.
                    // For gaussian PI we rely on the distribution already applying min floor during normalization.
                    if (state->config.pi_type == string("equal")) {
                        double min_p = state->config.min_transmission_prob;
                        if (hasData && p < min_p) {
                            p = min_p;
                        }
                    }
                    
                    double rand_val = (double)rand() / RAND_MAX;
                    decision = (rand_val < p);
                }
            }
        }
    }
    
    // Cache the decision
    shouldTransmitThisStep = decision;
    transmissionDecisionMade = true;
    
    if (call_count <= 20 || call_count % 1000 == 0) {
        cerr << "[SHOULD-TX-DECISION] Node " << nodeId 
             << " decided: " << decision << endl;
    }
    
    return decision;
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
    transmissionDecisionMade = false;
    shouldTransmitThisStep = false;
}
