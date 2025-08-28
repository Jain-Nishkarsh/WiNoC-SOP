/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the processing element
 */

#include "TokenRing.h"
#include "BMACController.h"

void TokenRing::updateTokenPacket(int channel)
{
    int token_pos = token_position[channel];
    int token_holder = rings_mapping[channel][token_pos];
    // TEST HOLD BUG
	//if (flag[channel][token_pos]->read() == RELEASE_CHANNEL)

    if (flag[channel][token_holder]->read() == RELEASE_CHANNEL)
	{
	    // number of hubs of the ring
	    int num_hubs = rings_mapping[channel].size();

	    token_position[channel] = (token_position[channel]+1)%num_hubs;

	    int new_token_holder = rings_mapping[channel][token_position[channel]];
        LOG << "*** Token of channel " << channel << " has been assigned to Hub_" <<  new_token_holder << endl;
	    current_token_holder[channel]->write(new_token_holder);
	    // TEST HOLD BUG
	    //flag[channel][token_position[channel]]->write(HOLD_CHANNEL);
        flag[channel][new_token_holder]->write(HOLD_CHANNEL);
	}
}

void TokenRing::updateTokenMaxHold(int channel)
{
	if (--token_hold_count[channel] == 0 ||
		flag[channel][token_position[channel]]->read() == RELEASE_CHANNEL)
	{
	    token_hold_count[channel] = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
	    // number of hubs of the ring
	    int num_hubs = rings_mapping[channel].size();

	    token_position[channel] = (token_position[channel]+1)%num_hubs;
	    LOG << "*** Token of channel " << channel << " has been assigned to Hub_" <<  rings_mapping[channel][token_position[channel]] << endl;

	    current_token_holder[channel]->write(rings_mapping[channel][token_position[channel]]);
	}

	current_token_expiration[channel]->write(token_hold_count[channel]);
}

void TokenRing::updateTokenHold(int channel)
{
	if (--token_hold_count[channel] == 0)
	{
	    token_hold_count[channel] = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
	    // number of hubs of the ring
	    int num_hubs = rings_mapping[channel].size();

	    token_position[channel] = (token_position[channel]+1)%num_hubs;
	    LOG << "*** Token of channel " << channel << " has been assigned to Hub_" <<  rings_mapping[channel][token_position[channel]] << endl;

	    current_token_holder[channel]->write(rings_mapping[channel][token_position[channel]]);
	}

	current_token_expiration[channel]->write(token_hold_count[channel]);
}

void TokenRing::updateTokens()
{
    if (reset.read()) {
        for (map<int,ChannelConfig>::iterator i = GlobalParams::channel_configuration.begin();
             i!=GlobalParams::channel_configuration.end();
             i++)
            current_token_holder[i->first]->write(rings_mapping[i->first][0]);
    }
    else
    {

        for (map<int,ChannelConfig>::iterator i = GlobalParams::channel_configuration.begin(); i!=GlobalParams::channel_configuration.end(); i++)
        {
            int channel = i->first;
            //int channel_holder;
            //channel_holder = current_token_holder[channel]->read();

            string macPolicy = getPolicy(channel).first;

            if (macPolicy == TOKEN_PACKET)
                updateTokenPacket(channel);
            else if (macPolicy == TOKEN_HOLD)
                updateTokenHold(channel);
            else if (macPolicy == TOKEN_MAX_HOLD)
                updateTokenMaxHold(channel);
            else if (macPolicy == BMAC_BIDIRECTIONAL)
                updateTokenBMAC(channel);
            else
                assert(false);
        }
    }
}


void TokenRing::attachHub(int channel, int hub, sc_in<int>* hub_token_holder_port, sc_in<int>* hub_token_expiration_port, sc_inout<int>* hub_flag_port)
{
    // If port for requested channel is not present, create the
    // port and connect a signal
    if (!current_token_holder[channel])
    {
        token_position[channel] = 0;
        // TEST HOLDBUG
        //token_position[channel] = hub;
        current_token_holder[channel] = new sc_out<int>();
        current_token_expiration[channel] = new sc_out<int>();

        token_holder_signals[channel] = new sc_signal<int>();
        token_expiration_signals[channel] = new sc_signal<int>();

        current_token_holder[channel]->bind(*(token_holder_signals[channel]));
        current_token_expiration[channel]->bind(*(token_expiration_signals[channel]));

        // initial value that will be overwritten if mac policy != TOKEN_PACKET
        token_hold_count[channel] = 0;


        if (GlobalParams::channel_configuration[channel].macPolicy[0] != TOKEN_PACKET &&
            GlobalParams::channel_configuration[channel].macPolicy[0] != BMAC_BIDIRECTIONAL) {
            // checking max hold cycles vs wireless transmission latency
            // consistency
            //TODO move this check: max_hold_cycles depends on the Channel not on the Hub
            double delay_ps = 1000*GlobalParams::flit_size/GlobalParams::channel_configuration[channel].dataRate;
            int cycles = ceil(delay_ps/GlobalParams::clock_period_ps);
            int max_hold_cycles = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
            assert(cycles< max_hold_cycles);

            token_hold_count[channel] = atoi(GlobalParams::channel_configuration[channel].macPolicy[1].c_str());
        }
        else if (GlobalParams::channel_configuration[channel].macPolicy[0] == BMAC_BIDIRECTIONAL) {
            // BMAC specific initialization - use reasonable token hold time
            token_hold_count[channel] = 10; // Allow sufficient time for packet transmission
        }
    }

    flag[channel][hub] = new sc_inout<int>();
    flag_signals[channel][hub] = new sc_signal<int>();
    flag[channel][hub]->bind(*(flag_signals[channel][hub]));
    hub_flag_port->bind(*(flag_signals[channel][hub]));

    // Connect tokenring to hub
    hub_token_holder_port->bind(*(token_holder_signals[channel]));
    hub_token_expiration_port->bind(*(token_expiration_signals[channel]));

    //LOG << "Attaching Hub " << hub << " to the token ring for channel " << channel << endl;
    rings_mapping[channel].push_back(hub);

    // TEST HOLD BUG
    int starting_hub = rings_mapping[channel][0];
    current_token_holder[channel]->write(starting_hub);
}

void TokenRing::updateTokenBMAC(int channel)
{
    // Use BMACController for bidirectional token management if available
    if (bmac_controller != nullptr)
    {
        // Let BMACController handle token passing decisions
        // The controller will manage the bidirectional logic
        // and communicate back to us via current_token_holder updates
        
        // For now, we just check if the token holder has changed
        // The actual logic is handled in BMACController::bmacProcess()
        return;
    }
    
    // Fallback to simplified BMAC implementation
    int token_pos = token_position[channel];
    int token_holder = rings_mapping[channel][token_pos];
    
    // Check if token should be passed (simplified logic)
    if (flag[channel][token_holder]->read() == RELEASE_CHANNEL)
    {
        // Implement bidirectional token passing
        // For now, use standard clockwise passing
        
        int num_hubs = rings_mapping[channel].size();
        token_position[channel] = (token_position[channel] + 1) % num_hubs;
        
        int new_token_holder = rings_mapping[channel][token_position[channel]];
        
        LOG << "*** BMAC Token for channel " << channel 
            << " passed from Hub " << token_holder 
            << " to Hub " << new_token_holder << endl;
            
        current_token_holder[channel]->write(new_token_holder);
        flag[channel][new_token_holder]->write(HOLD_CHANNEL);
    }
}

void TokenRing::setBMACController(BMACController* controller)
{
    bmac_controller = controller;
}


