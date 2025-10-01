/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the per-node Fuzzy Token helper
 */

#ifndef __NOXIMFUZZYTOKENNODE_H__
#define __NOXIMFUZZYTOKENNODE_H__

#include "FuzzyTokenController.h"
#include "DataStructs.h"
#include <systemc.h>

using namespace std;

// Per-node helper for Fuzzy Token MAC
class FuzzyTokenNode {
private:
    int nodeId;
    int channelId;
    FuzzyTokenController& controller;
    
    bool transmissionPending;
    bool nackReceived;
    bool preambleSent;
    int transmissionStartCycle;
    
public:
    FuzzyTokenNode(int nId, int chId) : 
        nodeId(nId), 
        channelId(chId),
        controller(FuzzyTokenController::getInstance()),
        transmissionPending(false),
        nackReceived(false),
        preambleSent(false),
        transmissionStartCycle(-1)
    {}
    
    // Check if this node should attempt transmission in current step
    bool shouldAttemptTransmission(bool hasData);
    
    // Called when node wants to transmit
    void startPreamble();
    
    // Called when NACK is received
    void receiveNack();
    
    // Check if node is token holder
    bool isTokenHolder();
    
    // Check if node is in fuzzy area
    bool isInFuzzyArea();
    
    // Get current mode
    FuzzyTokenMode getCurrentMode();
    
    // Reset state for new step
    void resetStepState();
    
    // Getters
    bool hasPendingTransmission() const { return transmissionPending; }
    bool hasSentPreamble() const { return preambleSent; }
    bool hasReceivedNack() const { return nackReceived; }
};

#endif
