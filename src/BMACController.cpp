/*
 * Noxim - the NoC Simulator
 *
 * (C) 2025
 *
 * This file contains the implementation of the BMAC Controller
 * for bidirectional token ring management
 */

#include "BMACController.h"

BMACController::BMACController(sc_module_name nm) : sc_module(nm)
{
    SC_METHOD(bmacProcess);
    sensitive << reset;
    sensitive << clock.pos();
    
    // Initialize BMAC configuration from global parameters
    initializeBMAC();
}

void BMACController::initializeBMAC()
{
    // Initialize BMAC for each channel
    for (map<int, ChannelConfig>::iterator it = GlobalParams::channel_configuration.begin();
         it != GlobalParams::channel_configuration.end(); ++it)
    {
        int channel = it->first;
        ChannelConfig config = it->second;
        
        // Check if this channel uses BMAC_BIDIRECTIONAL
        if (config.macPolicy.size() > 0 && config.macPolicy[0] == BMAC_BIDIRECTIONAL)
        {
            // Configure bidirectional token ring
            vector<int> hub_order;
            
            // Get hub order from global configuration
            // Use the token_ring_order from the channel configuration
            if (config.tokenRingOrder.size() > 0)
            {
                hub_order = config.tokenRingOrder;
                LOG << "BMAC Controller: Using token ring order from configuration for channel " 
                    << channel << ": ";
                for (size_t i = 0; i < hub_order.size(); i++)
                {
                    LOG << hub_order[i];
                    if (i < hub_order.size() - 1) LOG << ",";
                }
                LOG << endl;
            }
            else
            {
                // Fallback to default order if not specified in config
                hub_order = {0,1,2,3,7,6,5,9,10,11,15,14,13,12,8,4};
                LOG << "BMAC Controller: Using default token ring order for channel " 
                    << channel << " (no order specified in config)" << endl;
            }
            
            configureBMACRing(channel, hub_order);
            enableBidirectionalMode(channel, true);
            
            // Initialize token holder and direction
            current_token_holder[channel] = hub_order[0];
            token_direction[channel] = BMAC_CLOCKWISE;
            
            LOG << "BMAC Controller: Initialized bidirectional token ring for channel " 
                << channel << " with " << hub_order.size() << " hubs" << endl;
        }
    }
}

void BMACController::bmacProcess()
{
    if (reset.read())
    {
        // Reset all BMAC states
        for (map<int, BMACState>::iterator it = hub_states.begin(); 
             it != hub_states.end(); ++it)
        {
            it->second.current_state = BMAC_IDLE;
            it->second.has_token = false;
            it->second.token_hold_cycles = 0;
        }
        
        // Reset token holders to initial hub (hub 0) for each channel
        for (map<int, int>::iterator it = current_token_holder.begin();
             it != current_token_holder.end(); ++it)
        {
            if (token_ring_order[it->first].size() > 0)
                it->second = token_ring_order[it->first][0];
        }
    }
    else
    {
        // Update token rings for all channels
        for (map<int, vector<int>>::iterator it = token_ring_order.begin();
             it != token_ring_order.end(); ++it)
        {
            int channel = it->first;
            updateTokenRing(channel);
        }
    }
}

void BMACController::updateTokenRing(int channel)
{
    if (token_ring_order.find(channel) == token_ring_order.end())
        return;
        
    int current_holder = current_token_holder[channel];
    
    // Check if current token holder should pass the token
    BMACState& hub_state = hub_states[current_holder];
    
    bool should_pass_token = false;
    
    // BMAC token passing logic
    if (hub_state.current_state == BMAC_TOKEN_HOLD)
    {
        // Check if transmission is complete or token hold time exceeded
        int max_hold_cycles = GlobalParams::hub_configuration[current_holder].toTileBufferSize;
        // Try to get token_hold_cycles from hub configuration if available
        if (GlobalParams::hub_configuration.find(current_holder) != GlobalParams::hub_configuration.end()) {
            // Use configured token hold cycles (default 10 for BMAC)
            max_hold_cycles = 10; // This should be read from configuration
        }
        
        if (isTransmissionComplete(current_holder, channel) ||
            hub_state.token_hold_cycles >= max_hold_cycles)
        {
            should_pass_token = true;
        }
        else
        {
            hub_state.token_hold_cycles++;
        }
    }
    else if (hub_state.current_state == BMAC_IDLE)
    {
        // If hub is idle and has no data to transmit, pass token immediately
        should_pass_token = true;
    }
    
    if (should_pass_token)
    {
        passBidirectionalToken(channel, current_holder);
    }
    
    // Optimize token path if needed
    if ((int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps) % 1000 == 0)
    {
        optimizeTokenPath(channel);
    }
}

void BMACController::passBidirectionalToken(int channel, int current_holder)
{
    int direction = token_direction[channel];
    
    // Check if we should reverse direction for better efficiency
    if (shouldReverseDirection(channel, current_holder))
    {
        direction = (direction == BMAC_CLOCKWISE) ? BMAC_COUNTERCLOCKWISE : BMAC_CLOCKWISE;
        token_direction[channel] = direction;
        
        LOG << "BMAC Controller: Reversing token direction for channel " << channel 
            << " to " << (direction == BMAC_CLOCKWISE ? "clockwise" : "counter-clockwise") << endl;
    }
    
    // Get next hub in the chosen direction
    int next_hub = getNextHub(channel, current_holder, direction);
    
    if (next_hub != NOT_VALID)
    {
        // Update token holder
        current_token_holder[channel] = next_hub;
        
        // Update hub states
        hub_states[current_holder].has_token = false;
        hub_states[current_holder].current_state = BMAC_IDLE;
        hub_states[current_holder].token_hold_cycles = 0;
        
        hub_states[next_hub].has_token = true;
        hub_states[next_hub].current_state = BMAC_TOKEN_HOLD;
        
        LOG << "BMAC Controller: Token passed from hub " << current_holder 
            << " to hub " << next_hub << " on channel " << channel << endl;
    }
}

bool BMACController::shouldReverseDirection(int channel, int hub_id)
{
    // Implement bidirectional optimization logic
    // This is a simplified heuristic - in practice, you might want more sophisticated logic
    
    // Check if there are hubs with pending transmissions in the reverse direction
    vector<int>& ring_order = token_ring_order[channel];
    int current_direction = token_direction[channel];
    
    int hubs_with_data_forward = 0;
    int hubs_with_data_backward = 0;
    
    // Count hubs with pending data in both directions
    for (size_t i = 0; i < ring_order.size(); i++)
    {
        int hub = ring_order[i];
        
        // Check if hub has data to transmit (simplified check)
        // In a real implementation, you'd check the actual buffer status
        if (hub_states[hub].current_state != BMAC_IDLE)
        {
            if (current_direction == BMAC_CLOCKWISE)
            {
                if (i > (size_t)hub_id) hubs_with_data_forward++;
                else hubs_with_data_backward++;
            }
            else
            {
                if (i < (size_t)hub_id) hubs_with_data_forward++;
                else hubs_with_data_backward++;
            }
        }
    }
    
    // Reverse if there are more hubs with data in the opposite direction
    return (hubs_with_data_backward > hubs_with_data_forward);
}

int BMACController::getNextHub(int channel, int current_hub, int direction)
{
    if (token_ring_order.find(channel) == token_ring_order.end())
        return NOT_VALID;
        
    vector<int>& ring_order = token_ring_order[channel];
    
    // Find current hub position in ring
    int current_pos = NOT_VALID;
    for (size_t i = 0; i < ring_order.size(); i++)
    {
        if (ring_order[i] == current_hub)
        {
            current_pos = i;
            break;
        }
    }
    
    if (current_pos == NOT_VALID)
        return NOT_VALID;
    
    // Calculate next position based on direction
    int next_pos;
    if (direction == BMAC_CLOCKWISE)
    {
        next_pos = (current_pos + 1) % ring_order.size();
    }
    else // BMAC_COUNTERCLOCKWISE
    {
        next_pos = (current_pos - 1 + ring_order.size()) % ring_order.size();
    }
    
    return ring_order[next_pos];
}

void BMACController::configureBMACRing(int channel, const vector<int>& hub_order)
{
    token_ring_order[channel] = hub_order;
    
    // Initialize hub states for this channel
    for (size_t i = 0; i < hub_order.size(); i++)
    {
        int hub_id = hub_order[i];
        
        BMACState state;
        state.hub_id = hub_id;
        state.current_state = BMAC_IDLE;
        state.token_direction = BMAC_CLOCKWISE;
        state.token_hold_cycles = 0;
        state.bidirectional_mode = true;
        state.has_token = (i == 0); // First hub starts with token
        
        // Set next hubs in both directions
        state.next_hub_cw = hub_order[(i + 1) % hub_order.size()];
        state.next_hub_ccw = hub_order[(i - 1 + hub_order.size()) % hub_order.size()];
        
        hub_states[hub_id] = state;
    }
}

void BMACController::enableBidirectionalMode(int channel, bool enable)
{
    if (token_ring_order.find(channel) == token_ring_order.end())
        return;
        
    vector<int>& ring_order = token_ring_order[channel];
    
    for (size_t i = 0; i < ring_order.size(); i++)
    {
        int hub_id = ring_order[i];
        hub_states[hub_id].bidirectional_mode = enable;
    }
}

bool BMACController::isTransmissionComplete(int hub_id, int channel)
{
    // Simplified check - in practice, this would interface with the actual hub
    // to check if there are pending transmissions
    
    // For now, assume transmission is complete if hub has been holding token
    // for more than a threshold number of cycles
    BMACState& state = hub_states[hub_id];
    
    return (state.token_hold_cycles > 5); // Simplified threshold
}

void BMACController::optimizeTokenPath(int channel)
{
    // Implement path optimization logic
    // This could include:
    // 1. Analyzing traffic patterns
    // 2. Adjusting token ring order dynamically
    // 3. Implementing adaptive direction changes
    
    // For now, this is a placeholder for future optimization
    LOG << "BMAC Controller: Optimizing token path for channel " << channel << endl;
}

void BMACController::handleTokenExpiration(int channel, int hub_id)
{
    // Handle cases where a hub holds the token too long
    LOG << "BMAC Controller: Token expiration for hub " << hub_id 
        << " on channel " << channel << endl;
        
    // Force token to be passed
    passBidirectionalToken(channel, hub_id);
}

void BMACController::setTokenDirection(int channel, int direction)
{
    token_direction[channel] = direction;
}

void BMACController::updateHubState(int hub_id, int new_state)
{
    if (hub_states.find(hub_id) != hub_states.end())
    {
        hub_states[hub_id].current_state = new_state;
    }
}

BMACState BMACController::getHubState(int hub_id)
{
    if (hub_states.find(hub_id) != hub_states.end())
    {
        return hub_states[hub_id];
    }
    
    // Return default state if hub not found
    BMACState default_state;
    default_state.hub_id = hub_id;
    default_state.current_state = BMAC_IDLE;
    default_state.token_direction = BMAC_CLOCKWISE;
    default_state.token_hold_cycles = 0;
    default_state.bidirectional_mode = false;
    default_state.has_token = false;
    default_state.next_hub_cw = NOT_VALID;
    default_state.next_hub_ccw = NOT_VALID;
    
    return default_state;
}
