/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the global params needed by Noxim
 * to forward configuration to every sub-block
 */
#include "Hub.h"
#include <algorithm>

int Hub::tile2Port(int id)
{
	// TODO: check all [..] map access to replace with at()
	return tile2port_mapping.at(id);
}

int Hub::route(Flit& f)
{
	// check if it is a local delivery
	for (vector<int>::size_type i=0; i< GlobalParams::hub_configuration[local_id].attachedNodes.size();i++)
	{
		// ...to a destination which is connected to the Hub
		if (GlobalParams::hub_configuration[local_id].attachedNodes[i]==f.dst_id)
		{
			return tile2Port(f.dst_id);
		}
		// ...or to a relay which is locally connected to the Hub
		if (GlobalParams::hub_configuration[local_id].attachedNodes[i]==f.hub_relay_node)
		{
			assert(GlobalParams::winoc_dst_hops>0);
			return tile2Port(f.hub_relay_node);
		}

	}
	return DIRECTION_WIRELESS;

}


void Hub::rxPowerManager()
{
	// Check wheter accounting or not buffer to tile leakage
	// For each port, two poweroff condition should be checked:
	// - the buffer to tile is empty
	// - it has not been reserved

	// currently only supported without VC
	assert(GlobalParams::n_virtual_channels==1);

	for (int port=0;port<num_ports;port++)
	{
		if (!buffer_to_tile[port][DEFAULT_VC].IsEmpty() ||
			antenna2tile_reservation_table.isNotReserved(port))
			power.leakageBufferToTile();

		else
			buffer_to_tile_poweroff_cycles[port]++;
	}


	for (unsigned int i=0;i<rxChannels.size();i++)
	{
		int ch_id = rxChannels[i];

		if (!target[ch_id]->buffer_rx.IsEmpty())
		{
			power.leakageAntennaBuffer();
		}
		else
			buffer_rx_sleep_cycles[ch_id]++;
	}

	// Check wheter accounting antenna RX buffer
	// check if there is at least one not empty antenna RX buffer
	// To be only applied if the current hub is in RADIO_EVENT_SLEEP_ON mode

	if (power.isSleeping())
		total_sleep_cycles++;

	else // not sleeping
	{
		power.wirelessSnooping();
		power.leakageTransceiverRx();
		power.biasingRx();
	}
}


void Hub::updateRxPower()
{
	if (GlobalParams::use_powermanager)
		rxPowerManager();
	else
	{
		power.wirelessSnooping();
		power.leakageTransceiverRx();
		power.biasingRx();

		for (unsigned int i=0;i<rxChannels.size();i++)
			for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
				power.leakageAntennaBuffer();

		for (int i = 0; i < num_ports; i++)
			for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
				power.leakageBufferToTile();
	}
}

void Hub::txPowerManager()
{
	for (unsigned int i=0;i<txChannels.size();i++)
	{
		// check if not empty or reserved
		if (!init[i]->buffer_tx.IsEmpty() ||
			tile2antenna_reservation_table.isNotReserved(i) )
		{
			power.leakageAntennaBuffer();
			// check the second condition for turning off analog tx
			if (power.isSleeping())
			{
				analogtxoff_cycles[i]++;
			}
			else
			{
				power.leakageTransceiverTx();
				power.biasingTx();
			}
		}
		else
		{   // abtx is empty and not reserved - turn off
			// note that this also applies to analog tx and serializer
			abtxoff_cycles[i]++;
			analogtxoff_cycles[i]++;
			total_ttxoff_cycles++;
		}
	}
}

void Hub::updateTxPower()
{
	if (GlobalParams::use_powermanager)
		txPowerManager();
	else
	{
		for (unsigned int i=0;i<txChannels.size();i++)
			for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
				power.leakageAntennaBuffer();

		power.leakageTransceiverTx();
		power.biasingTx();
	}

	// mandatory
	power.leakageLinkRouter2Hub();
	for (int i = 0; i < num_ports; i++)
		for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
			power.leakageBufferFromTile();
}


void Hub::txRadioProcessTokenPacket(int channel)
{
	static int token_packet_tx_count = 0;
    int current_holder = current_token_holder[channel]->read();
    int current_channel_flag =flag[channel]->read();

	if ( current_holder == local_id && current_channel_flag !=RELEASE_CHANNEL)
	{
		if (!init[channel]->buffer_tx.IsEmpty())
		{
			Flit flit = init[channel]->buffer_tx.Front();
			
			token_packet_tx_count++;
			if (token_packet_tx_count % 10 == 0) {
				cerr << "[TOKEN-PACKET-TX #" << token_packet_tx_count << "] Hub " << local_id 
				     << " transmitting (src=" << flit.src_id << ", dst=" << flit.dst_id << ")" << endl;
			}

			// TODO: check whether it would make sense to use transmission_in_progress to
			// avoid multiple notify()
			LOG << "*** [Ch"<<channel<<"] Requesting transmission event of flit " << flit << endl;
			init[channel]->start_request_event.notify();
		}
		else
		{
			if (!transmission_in_progress.at(channel))
			{
				LOG << "*** [Ch"<<channel<<"] Buffer_tx empty and no trasmission in progress, releasing token" << endl;
				flag[channel]->write(RELEASE_CHANNEL);
			}
			else
				LOG << "*** [Ch"<<channel<<"] Buffer_tx empty, but trasmission in progress, holding token" << endl;
		}
	}
}

void Hub::txRadioProcessTokenHold(int channel)
{
	if (flag[channel]->read()==RELEASE_CHANNEL)
		flag[channel]->write(HOLD_CHANNEL);

	if (current_token_holder[channel]->read() == local_id)
	{
		if (!init[channel]->buffer_tx.IsEmpty())
		{
			//LOG << "Token holder for channel " << channel << " with not empty buffer_tx" << endl;
			if (current_token_expiration[channel]->read() < flit_transmission_cycles[channel])
			{
				//LOG << "TOKEN_HOLD policy: Not enough token expiration time for sending channel " << channel << endl;
			}
			else
			{
				flag[channel]->write(HOLD_CHANNEL);
				LOG << "*** [Ch" << channel << "] Starting transmission event" << endl;
				init[channel]->start_request_event.notify();
			}
		}
		else
		{
			//LOG << "TOKEN_HOLD policy: nothing to transmit, holding token for channel " << channel << endl;
		}
	}
}

void Hub::txRadioProcessTokenMaxHold(int channel)
{
	if (flag[channel]->read()==RELEASE_CHANNEL)
		flag[channel]->write(HOLD_CHANNEL);

	if (current_token_holder[channel]->read() == local_id)
	{
		if (!init[channel]->buffer_tx.IsEmpty())
		{
			//LOG << "Token holder for channel " << channel << " with not empty buffer_tx" << endl;

			if (current_token_expiration[channel]->read() < flit_transmission_cycles[channel])
			{
				//LOG << "TOKEN_MAX_HOLD: Not enough token expiration time, releasing token for channel " << channel << endl;
				flag[channel]->write(RELEASE_CHANNEL);
			}
			else
			{
				flag[channel]->write(HOLD_CHANNEL);
				LOG << "Starting transmission on channel " << channel << endl;
				init[channel]->start_request_event.notify();
			}
		}
		else
		{
			//LOG << "TOKEN_MAX_HOLD: Buffer_tx empty, releasing token for channel " << channel << endl;
			flag[channel]->write(RELEASE_CHANNEL);
		}
	}
}

void Hub::antennaToTileProcess()
{
	if (reset.read())
	{
		for (int i = 0; i < num_ports; i++)
		{
			req_tx[i]->write(0);
			current_level_tx[i] = 0;
		}
		return;
	}
	// IMPORTANT: do not move from here
	// The rxPowerManager must perform its checks before the flits are removed from buffers
	updateRxPower();


	/***********************************************************************************
      data flow from antenna(s) towards the tiles consist of 3 different steps:

      1) data received from a radio channel stored (if possible) to a specific buffer_rx
      2) data found on a buffer_rx moved to a buffer_to_tile
      3) data found on a buffer_to_tile moved to signal_tx 

      From a implementation perspective, they are performed in 3-2-1 order, to simulate
      a kind of pipelined sequence
     ***********************************************************************************/


	//////////////////////////////////////////////////////////////////////////////////////
	// Moves a flit from buffer_to_tile to the appropriate signal_tx
	// No routing required: each port is associated to a prefixed tile
	for (int i = 0; i < num_ports; i++)
	{
		// TODO: check blocking channel (like the blocking single signal ?)
		for (int k = 0;k < GlobalParams::n_virtual_channels; k++)
		{
			int vc = (start_from_vc[i]+k)%(GlobalParams::n_virtual_channels);

			if (!buffer_to_tile[i][vc].IsEmpty())
			{
				Flit flit = buffer_to_tile[i][vc].Front();

				LOG << "Flit " << flit << " found on buffer_to_tile[" << i <<"][" << vc << "] " << endl;
				if (current_level_tx[i] == ack_tx[i].read() &&
					buffer_full_status_tx[i].read().mask[vc] == false)
				{
					LOG << "Flit " << flit << " moved from buffer_to_tile[" << i <<"][" << vc << "] to signal flit_tx["<<i<<"] " << endl;
					if (flit.flit_type == FLIT_TYPE_HEAD) {
						cerr << "[B2T->ROUTER] Hub " << local_id << " sending HEAD flit (src=" << flit.src_id 
						     << ", dst=" << flit.dst_id << ", seq=" << flit.sequence_no 
						     << ") from buffer_to_tile[" << i << "][" << vc << "] to Router" << endl;
					}

					flit_tx[i].write(flit);
					current_level_tx[i] = 1 - current_level_tx[i];
					req_tx[i].write(current_level_tx[i]);

					buffer_to_tile[i][vc].Pop();
					power.bufferToTilePop();
					power.r2hLink();
					break; // port flit transmitted, skip remaining VCs
				}
				else
				{
					LOG << "Flit " << flit << " cannot move from buffer_to_tile[" << i <<"] [" << vc << "] to signal flit_tx["<<i<<"] " << endl;
					if (flit.flit_type == FLIT_TYPE_HEAD) {
						cerr << "[B2T-STALL] Hub " << local_id << " cannot send HEAD flit (src=" << flit.src_id 
						     << ", dst=" << flit.dst_id << ", seq=" << flit.sequence_no 
						     << ") to Router - backpressure from buffer_to_tile[" << i << "][" << vc << "]" << endl;
					}
				}
			}//if buffer not empty
		}
		start_from_vc[i] = (start_from_vc[i]+1)%GlobalParams::n_virtual_channels;
	}

	/////////////////////////////////////////////////////////////////////////////////
	// Move a flit from antenna buffer_rx to the appropriate buffer_to_tile.
	//
	// Two different phases:
	// 1) stores routing decision about the incoming flit (e.g., to which output port)
	// 2) Moves the flits removing from antenna buffer_rx

	for (unsigned int i = 0; i < rxChannels.size(); i++)
	{
		int channel = rxChannels[i];

		if (!(target[channel]->buffer_rx.IsEmpty()))
		{
			Flit received_flit = target[channel]->buffer_rx.Front();
			power.antennaBufferFront();

			// Check antenna buffer_rx making appropriate reservations
			if (received_flit.flit_type==FLIT_TYPE_HEAD)
			{
				int dst_port;

				if (received_flit.hub_relay_node!=NOT_VALID)
					dst_port = tile2Port(received_flit.hub_relay_node);
				else
                    dst_port = tile2Port(received_flit.dst_id);

				TReservation r;
				r.input = channel;
				r.vc = received_flit.vc_id;

				LOG << " Checking reservation availability of output port " << dst_port << " by channel " << channel << " for flit " << received_flit << endl;

				int rt_status = antenna2tile_reservation_table.checkReservation(r,dst_port);

				if (rt_status == RT_AVAILABLE)
				{
					LOG << "Reserving output port " << dst_port << " by channel " << channel << " for flit " << received_flit << endl;
					antenna2tile_reservation_table.reserve(r, dst_port);

					// The number of commucation using the wireless network, accounting also
					// partial wired path
					wireless_communications_counter++;
				}
				else if (rt_status == RT_ALREADY_SAME)
				{
					LOG << " RT_ALREADY_SAME reserved direction " << dst_port << " for flit " << received_flit << endl;
				}
				else if (rt_status == RT_ALREADY_OTHER_OUT)
				{
					LOG << " RT_ALREADY_OTHER_OUT reservation direction " << dst_port << " for flit " << received_flit << endl;
				}
				else if (rt_status == RT_OUTVC_BUSY)
				{
					LOG << " RT_OUTVC_BUSY reservation direction " << dst_port << " for flit " << received_flit << endl;
				}
				else assert(false); // no meaningful status here


			}
		}
	}
	// forwarding
	for (unsigned int i = 0; i < rxChannels.size(); i++)
	{
		int channel = rxChannels[i];
		vector<pair<int,int> > reservations = antenna2tile_reservation_table.getReservations(channel);

		if (reservations.size()!=0)
		{
			int rnd_idx = rand()%reservations.size();

			int port = reservations[rnd_idx].first;
			int vc = reservations[rnd_idx].second;

			if (!(target[channel]->buffer_rx.IsEmpty()))
			{
				Flit received_flit = target[channel]->buffer_rx.Front();
				power.antennaBufferFront();

				if ( !buffer_to_tile[port][vc].IsFull() )
				{
					target[channel]->buffer_rx.Pop();
					power.antennaBufferPop();
					LOG << "*** [Ch" << channel << "] Moving flit  " << received_flit << " from buffer_rx to buffer_to_tile[" << port <<"][" << vc << "]" << endl;
					if (received_flit.flit_type == FLIT_TYPE_HEAD) {
						cerr << "[RX->B2T] Hub " << local_id << " moving HEAD flit (src=" << received_flit.src_id 
						     << ", dst=" << received_flit.dst_id << ", seq=" << received_flit.sequence_no 
						     << ") from buffer_rx[" << channel << "] to buffer_to_tile[" << port << "][" << vc << "]" << endl;
					}

					buffer_to_tile[port][vc].Push(received_flit);
					power.bufferToTilePush();

					if (received_flit.flit_type == FLIT_TYPE_TAIL)
					{
						LOG << "Releasing reservation for output port " << port << ", flit " << received_flit << endl;
						TReservation r;
						r.input = channel;
						r.vc = vc;
						antenna2tile_reservation_table.release(r,port);
					}
				}
				else
				{
					LOG << "Full buffer_to_tile[" << port <<"][" << vc << "]" << ", cannot store " << received_flit << endl;
					if (received_flit.flit_type == FLIT_TYPE_HEAD) {
						cerr << "[B2T-FULL] Hub " << local_id << " STALLED - buffer_to_tile[" << port << "][" << vc << "] FULL, "
						     << "cannot move HEAD flit (src=" << received_flit.src_id << ", dst=" << received_flit.dst_id 
						     << ", seq=" << received_flit.sequence_no << ")" << endl;
					}
				}
			}
			else
			{
				// should be ok
				/*
                LOG << "WARNING: empty target["<<channel<<"] buffer_rx, but reservation still present, if correct, remove assertion below " << endl;
                assert(false);
                */
			}
		}
	}
}

void Hub::tileToAntennaProcess()
{
	// double cycle = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
	// if (cycle > 0 && cycle < 58428)
	// {
	//     if (local_id == 1)
	//     {
	//         cout << "CYCLES " << cycle << endl;
	//         for (int j = 0; j < num_ports; j++)
	//     	buffer_from_tile[j].Print();;
	//         init[0]->buffer_tx.Print();
	//         cout << endl;
	//     }
	// }

	if (reset.read())
	{
		for (unsigned int i =0 ;i<txChannels.size();i++)
		{
			int channel = txChannels[i];
			flag[channel]->write(HOLD_CHANNEL);
		}

		TBufferFullStatus bfs;
		for (int i = 0; i < num_ports; i++)
		{
			ack_rx[i]->write(0);
			buffer_full_status_rx[i].write(bfs);
			current_level_rx[i] = 0;
		}
		return;
	}

	for (unsigned int i =0 ;i<txChannels.size();i++)
	{
		int channel = txChannels[i];

		string macPolicy = token_ring->getPolicy(channel).first;

		if (macPolicy == TOKEN_PACKET)
			txRadioProcessTokenPacket(channel);
		else if (macPolicy == TOKEN_HOLD)
			txRadioProcessTokenHold(channel);
		else if (macPolicy == TOKEN_MAX_HOLD)
			txRadioProcessTokenMaxHold(channel);
		else if (macPolicy == FUZZY_TOKEN || macPolicy == FUZZY_TOKEN_PLUS || macPolicy == FUZZY_TOKEN_JUMP_PLUS)
			txRadioProcessFuzzyToken(channel);
		else
		{
			cerr << "ERROR: Unknown MAC policy '" << macPolicy << "' for channel " << channel << endl;
			assert(false);
		}
	}

	int last_reserved = NOT_VALID;

	// used to store routing decisions
	int * r_from_tile[num_ports];
	for (int i=0;i<num_ports;i++)
		r_from_tile[i] = new int[GlobalParams::n_virtual_channels];

	// 1st phase: Reservation
	for (int j = 0; j < num_ports; j++)
	{
		int i = (start_from_port + j) % (num_ports);

		for (int k = 0;k < GlobalParams::n_virtual_channels; k++)
		{
			int vc = (start_from_vc[i]+k)%(GlobalParams::n_virtual_channels);

			if (!buffer_from_tile[i][vc].IsEmpty())
			{
				LOG << "Reservation: buffer_from_tile[" << i <<"][" << vc << "] not empty " << endl;

				Flit flit = buffer_from_tile[i][vc].Front();

				assert(flit.vc_id == vc);

				power.bufferFromTileFront();
				r_from_tile[i][vc] = route(flit);

				if (flit.flit_type == FLIT_TYPE_HEAD)
				{
					TReservation r;
					r.input = i;
					r.vc = vc;

					assert(r_from_tile[i][vc]==DIRECTION_WIRELESS);
					int channel;

					if (flit.hub_relay_node==NOT_VALID)
						channel = selectChannel(local_id, tile2Hub(flit.dst_id));
					else
						channel = selectChannel(local_id, tile2Hub(flit.hub_relay_node));


					assert(channel!=NOT_VALID && "hubs are not connected by any channel");

					LOG << "Checking reservation availability of Channel " << channel << " by Hub port[" << i << "][" << vc << "] for flit " << flit << endl;

					int rt_status = tile2antenna_reservation_table.checkReservation(r,channel);

					if (rt_status == RT_AVAILABLE)
					{
						LOG << "Reservation of channel " << channel << " from Hub port["<< i << "]["<<vc<<"] by flit " << flit << endl;
						tile2antenna_reservation_table.reserve(r, channel);
					}
					else if (rt_status == RT_ALREADY_SAME)
					{
						LOG << "RT_ALREADY_SAME reserved channel " << channel << " for flit " << flit << endl;
					}
					else if (rt_status == RT_OUTVC_BUSY)
					{
						LOG << "RT_OUTVC_BUSY reservation for channel " << channel << " for flit " << flit << endl;
					}
					else if (rt_status == RT_ALREADY_OTHER_OUT)
					{
						LOG << "RT_ALREADY_OTHER_OUT a channel different from " << channel << " already reserved by Hub port["<< i << "]["<<vc<<"]" << endl;
					}
					else assert(false); // no meaningful status here
				}
			}
		}
		start_from_vc[i] = (start_from_vc[i]+1)%GlobalParams::n_virtual_channels;
	} // for num_ports

	if (last_reserved!=NOT_VALID)
		start_from_port = (last_reserved+1)%num_ports;

	// 2nd phase: Forwarding
	for (int i = 0; i < num_ports; i++)
	{
		vector<pair<int,int> > reservations = tile2antenna_reservation_table.getReservations(i);

		if (reservations.size()!=0)
		{
			int rnd_idx = rand()%reservations.size();

			int o = reservations[rnd_idx].first;
			int vc = reservations[rnd_idx].second;

			if (!buffer_from_tile[i][vc].IsEmpty())
			{
				Flit flit = buffer_from_tile[i][vc].Front();
				// powerFront already accounted in 1st phase

				assert(r_from_tile[i][vc] == DIRECTION_WIRELESS);

				int channel =  o;

				if (channel != NOT_RESERVED)
				{
					if (!(init[channel]->buffer_tx.IsFull()) )
					{
						buffer_from_tile[i][vc].Pop();
						power.bufferFromTilePop();
						init[channel]->buffer_tx.Push(flit);
						power.antennaBufferPush();
						if (flit.flit_type == FLIT_TYPE_TAIL)
						{
							TReservation r;
							r.input = i;
							r.vc = vc;
							tile2antenna_reservation_table.release(r,channel);
						}

						LOG << "Flit " << flit << " moved from buffer_from_tile["<<i<<"]["<<vc<<"]  to buffer_tx["<<channel<<"] " << endl;
					}
					else
					{
						LOG << "Buffer Full: Cannot move flit " << flit << " from buffer_from_tile["<<i<<"] to buffer_tx["<<channel<<"] " << endl;
						// Trigger congestion handling only on new packet arrival (HEAD flit)
						if (flit.flit_type == FLIT_TYPE_HEAD) {
							// Count enqueue-full event for Token Packet (and generally for visibility)
							tx_enqueue_full_events_total++;
							tx_enqueue_full_events_by_channel[channel]++;
							handleFuzzyTokenCongestion(channel);
						}
						//init[channel]->buffer_tx.Print();
					}
				}
				else
				{
					LOG << "Forwarding: No channel reserved for input port [" << i << "][" << vc << "], flit " << flit << endl;
				}
			}

		}// for all the ports

		// Update global TX congestion flag for this hub (any channel full => congested)
		bool anyTxFull = false;
		for (unsigned int ci = 0; ci < txChannels.size(); ++ci) {
			int ch = txChannels[ci];
			if (init[ch]->buffer_tx.IsFull()) { anyTxFull = true; break; }
		}
		Hub::updateTxCongestionFlag(local_id, anyTxFull);
	}

	for (int i = 0; i < num_ports; i++)
	{

		if (req_rx[i]->read() == 1 - current_level_rx[i])
		{
			Flit received_flit = flit_rx[i]->read();
			int vc = received_flit.vc_id;
			LOG << "Reading " << received_flit << " from signal flit_rx[" << i << "]" << endl;

			/*
            if (!buffer_from_tile[i][vc].deadlockFree())
            {
            LOG << " deadlock on buffer " << i << endl;
            buffer_from_tile[i][vc].Print();
            }
            */

			if (!buffer_from_tile[i][vc].IsFull())
			{
				LOG << "Storing " << received_flit << " on buffer_from_tile[" << i << "][" << vc << "]" << endl;

				buffer_from_tile[i][vc].Push(received_flit);
				power.bufferFromTilePush();

				current_level_rx[i] = 1 - current_level_rx[i];
			}
			else
			{
				LOG << "Buffer Full: Cannot store " << received_flit << " on buffer_from_tile[" << i << "][" << vc << "]" << endl;
				//buffer_from_tile[i][TODO_VC].Print();
			}
		}
		ack_rx[i]->write(current_level_rx[i]);
		// updates the mask of VCs to prevent incoming data on full buffers
		TBufferFullStatus bfs;
		for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
			bfs.mask[vc] = buffer_from_tile[i][vc].IsFull();
		buffer_full_status_rx[i].write(bfs);
	}

	// IMPORTANT: do not move from here
	// The txPowerManager assumes that all flit buffer write have been done
	updateTxPower();
}

// Static map initialization
std::map<int,bool> Hub::s_tx_congested;

bool Hub::isTxCongestedForHub(int hubId) {
	std::map<int,bool>::iterator it = s_tx_congested.find(hubId);
	if (it == s_tx_congested.end()) return false;
	return it->second;
}

void Hub::updateTxCongestionFlag(int hubId, bool congested) {
	s_tx_congested[hubId] = congested;
}

int Hub::selectChannel(int src_hub, int dst_hub) const
{
	vector<int> & first = GlobalParams::hub_configuration[src_hub].txChannels;
	vector<int> & second = GlobalParams::hub_configuration[dst_hub].rxChannels;

	vector<int> intersection;

	for (unsigned int i=0;i<first.size();i++)
	{
		for (unsigned int j=0;j<second.size();j++)
		{
			if (first[i] ==second[j])
				intersection.push_back(first[i]);
		}
	}

	if (intersection.size()==0)
	    return NOT_VALID;

	if (GlobalParams::channel_selection==CHSEL_RANDOM)
		return intersection[rand()%intersection.size()];
	else
	if (GlobalParams::channel_selection==CHSEL_FIRST_FREE)
	{
		int start_channel = rand()%intersection.size();
		int k;

		for (vector<int>::size_type i=0;i<intersection.size();i++)
		{
			k = (start_channel+i)%intersection.size();

			if (!transmission_in_progress.at(intersection[k]))
			{
				cout << "Found free channel " << intersection[k] << " on (src,dest) (" << src_hub << "," << dst_hub << ") " << endl;
				return intersection[k];
			}
		}
		cout << "All channel busy, applying random selection " << endl;
		return intersection[rand()%intersection.size()];
	}

	return NOT_VALID;
}
// Fuzzy Token MAC Implementation

void Hub::initializeFuzzyToken(int channel)
{
	if (fuzzyTokenNodes.find(channel) != fuzzyTokenNodes.end())
		return; // Already initialized
	
	// Create FuzzyTokenNode for this hub on this channel
	fuzzyTokenNodes[channel] = new FuzzyTokenNode(local_id, channel);
	fuzzyTokenStepCycles[channel] = 0;
	fuzzyTokenPreambleDetected[channel] = false;
	fuzzyTokenActiveTransmitters[channel] = 0;
	fuzzyTokenStepStartCycle[channel] = 0;  // Option 2
	fuzzyTokenTransmissionThisStep[channel] = false;  // Option 2
	fuzzyTokenDeferredCongestion[channel] = false; // no deferred congestion initially
	
	// Register this channel with the global controller if not already done
	FuzzyTokenController& controller = FuzzyTokenController::getInstance();
	if (controller.getChannelState(channel) == nullptr)
	{
		// This is the first hub to use this channel, register it
		FuzzyTokenConfig config = GlobalParams::channel_configuration[channel].fuzzyTokenConfig;
		
		// Collect all HUB IDs using this channel
		// NOTE: FuzzyTokenNode uses hub IDs, not tile/node IDs
		vector<int> allHubs;
		for (auto& hubConfig : GlobalParams::hub_configuration)
		{
			int hubId = hubConfig.first;
			vector<int> txCh = hubConfig.second.txChannels;
			
			// Check if this hub transmits on this channel
			if (find(txCh.begin(), txCh.end(), channel) != txCh.end())
			{
				allHubs.push_back(hubId);
			}
		}
		
		// Remove duplicates and sort
		sort(allHubs.begin(), allHubs.end());
		allHubs.erase(unique(allHubs.begin(), allHubs.end()), allHubs.end());
		
		// Get MAC policy for this channel
		string macPolicy = token_ring->getPolicy(channel).first;
		
		controller.registerChannel(channel, config, allHubs.size(), allHubs, macPolicy);
		
		if (GlobalParams::verbose_mode >= VERBOSE_LOW)
		{
			cout << "Hub " << local_id << " initialized Fuzzy Token for channel " << channel
			     << " with " << allHubs.size() << " hubs: ";
			for (int h : allHubs) cout << h << " ";
			cout << endl;
		}
	}
}

bool Hub::detectMultiplePreambles(int channel)
{
	// In a real implementation, this would check if multiple preambles
	// are being transmitted simultaneously
	// For simulation, we track this through fuzzyTokenActiveTransmitters
	return (fuzzyTokenActiveTransmitters[channel] > 1);
}

void Hub::sendNack(int channel)
{
	// Create and broadcast NACK flit
	Flit nack;
	nack.flit_type = FLIT_TYPE_NACK;
	nack.src_id = local_id;
	nack.dst_id = -1; // Broadcast
	nack.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
	
	if (GlobalParams::verbose_mode == VERBOSE_HIGH)
	{
		cout << "Hub " << local_id << " sending NACK on channel " << channel
		     << " at cycle " << (int)nack.timestamp << endl;
	}
	
	// In actual implementation, this would be sent through the wireless channel
	// For now, we just log it and notify nodes through the controller
	// The nodes will abort their transmissions
}

void Hub::txRadioProcessFuzzyToken(int channel)
{
	static int fuzzy_call_count = 0;
	fuzzy_call_count++;
	
	if (fuzzy_call_count % 1000 == 0) {
		cerr << "[FUZZY-PROCESS #" << fuzzy_call_count << "] Hub " << local_id 
		     << " processing channel " << channel << endl;
	}
	
	// Initialize Fuzzy Token for this channel if needed
	if (fuzzyTokenNodes.find(channel) == fuzzyTokenNodes.end())
	{
		cerr << "[FUZZY-INIT] Hub " << local_id << " initializing Fuzzy Token for channel " << channel << endl;
		initializeFuzzyToken(channel);
	}
	
	FuzzyTokenNode* node = fuzzyTokenNodes[channel];
	FuzzyTokenController& controller = FuzzyTokenController::getInstance();
	FuzzyTokenChannelState* state = controller.getChannelState(channel);
	
	// NULL CHECK FIRST - return early if state not initialized yet
	if (!state)
	{
		if (GlobalParams::verbose_mode == VERBOSE_HIGH)
			cout << "Warning: Fuzzy Token state not initialized for channel " << channel << endl;
		return;
	}
	
	// PHASE 1: Update ready bitmap - mark if this hub has packets to send
	bool hasPacketToSend = !init[channel]->buffer_tx.IsEmpty();
	
	// Check which PLUS features to enable based on MAC policy
	string macPolicy = token_ring->getPolicy(channel).first;
	bool usePlusFeatures = (macPolicy == FUZZY_TOKEN_PLUS || macPolicy == FUZZY_TOKEN_JUMP_PLUS);  // Phase 2: Ready-count trigger
	bool useJumpFeatures = (macPolicy == FUZZY_TOKEN_JUMP_PLUS);  // Phase 3: Ready-aware jump
	
	if (usePlusFeatures) {
		state->setHubReady(local_id, hasPacketToSend);
		
		// PHASE 1: Log ready bitmap updates for testing
		static int bitmap_update_count = 0;
		bitmap_update_count++;
		if (bitmap_update_count <= 200) { // Log first 200 updates
			cerr << "[READY-BITMAP-UPDATE #" << bitmap_update_count << "] Hub " << local_id 
			     << " ready=" << (hasPacketToSend ? "YES" : "NO") 
			     << " (buffer_tx.IsEmpty=" << init[channel]->buffer_tx.IsEmpty() << ")" << endl;
		}
	}
	
	// PHASE 2: Token holder updates ready history and performs proactive mode switching
	// This happens once per cycle after all hubs have updated their ready bits
	if (usePlusFeatures && node->isTokenHolder()) {
		static int last_history_update_cycle = -1;
		int current_cycle = (int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
		
		// Update history only once per cycle (token holder coordinates)
		if (current_cycle != last_history_update_cycle) {
			state->updateReadyHistory();
			state->checkAndSwitchModeProactive();
			last_history_update_cycle = current_cycle;
		}
	}
	
	// PAPER-CORRECT: Synchronous step coordination
	// Steps end after: 1 cycle (silence), 2 cycles (collision), C cycles (success)
	// Only token holder coordinates step end to ensure ALL nodes register preambles first
	int current_cycle = (int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
	
	// Track step phases
	if (fuzzyTokenTransmissionThisStep[channel]) {
		fuzzyTokenStepStartCycle[channel] = current_cycle;
		fuzzyTokenTransmissionThisStep[channel] = false;
	}
	
	int step_duration = current_cycle - fuzzyTokenStepStartCycle[channel];
	
	// Determine if we have data early (used by focused-mode fast path)
	bool hasDataEarly = !init[channel]->buffer_tx.IsEmpty();

	// Focused-mode fast path:
	// In focused mode there are no preambles; the token holder should immediately transmit if it has data.
	// Only if the token holder has no data should we end the step as SILENCE and advance the token.
	if (state->getMode() == FOCUSED_MODE) {
		// PHASE 1: Log ready bitmap for first 20 FOCUSED steps (testing)
		static int focused_step_log_count = 0;
		if (node->isTokenHolder() && focused_step_log_count < 20) {
			state->logReadyBitmap(current_cycle);
			focused_step_log_count++;
		}
		
		if (node->isTokenHolder()) {
			if (hasDataEarly) {
				// Start transmission now (no preamble/collision phases)
				if (!init[channel]->buffer_tx.IsEmpty()) {
					Flit flit = init[channel]->buffer_tx.Front();
					static int focused_tx_count = 0;
					focused_tx_count++;
					cerr << "[FOCUSED-TX-START #" << focused_tx_count << "] Hub " << local_id 
					     << " starting transmission in FOCUSED mode:" << endl
					     << "    src=" << flit.src_id << ", dst=" << flit.dst_id 
					     << ", type=" << flit.flit_type << ", seq=" << flit.sequence_no << endl;
					if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
						cout << "Hub " << local_id << " (token holder) transmitting on channel "
						     << channel << " (focused mode)" << endl;
					}

					// Mark that transmission started in this step and notify initiator
					fuzzyTokenTransmissionThisStep[channel] = true;
					init[channel]->start_request_event.notify();
					// NOTE: endStep() will be called in Initiator when TAIL flit is sent
				}
				return; // Do not perform preamble-based handling in focused mode
			} else {
				// No data to send: end step as SILENCE and advance token
				// In FOCUSED mode, no preamble/collision overhead, so advance immediately
				static int focused_silence_count = 0;
				focused_silence_count++;
				if (focused_silence_count <= 10 || focused_silence_count % 1000 == 0) {
					cerr << "[FOCUSED-SILENCE #" << focused_silence_count << "] Token holder (Hub " << local_id
					     << ") has no data, advancing token (SILENCE)" << endl;
				}
				// FOCUSED mode: immediate token advance, no silence_cycles delay
				controller.endStep(channel, OUTCOME_SILENCE, 0);
				// Start new step immediately
				fuzzyTokenStepStartCycle[channel] = current_cycle;
				fuzzyTokenActiveTransmitters[channel] = 0;
				for (auto& pair : fuzzyTokenNodes) {
					if (pair.first == channel) {
						pair.second->resetStepState();
					}
				}
				return; // Focused-mode step concluded
			}
		}
	}

	// CRITICAL FIX: Token holder coordinates step outcome after preamble collection phase (FUZZY mode only)
	// - Handles SILENCE and COLLISION outcomes even if the token holder itself is not transmitting
	// - Prevents multiple initiators from proceeding when the token holder is idle
	// Paper timing: silence=1 cycle (no preambles), collision=2 cycles (preamble+NACK)
	if (state->getMode() == FUZZY_MODE && node->isTokenHolder() && step_duration >= state->config.preamble_cycles) {
		// PHASE 1: Log ready bitmap for first 20 steps (testing)
		static int step_log_count = 0;
		if (step_log_count < 20) {
			state->logReadyBitmap(current_cycle);
			step_log_count++;
		}
		
		// Token holder checks for activity after preamble phase
		int active_count = state->getActiveTransmitters();
		
		if (active_count == 0) {
			// SILENCE: No preambles sent, end step after configured silence_cycles
			static int silence_count = 0;
			silence_count++;
			if (silence_count <= 10 || silence_count % 1000 == 0) {
				cerr << "[FUZZY-SILENCE #" << silence_count << "] Token holder (Hub " << local_id
				     << ") detected silence after " << step_duration << " cycles, ending step" << endl;
			}
			// Report how long this silence step took (just preamble_cycles)
			int step_cycles = state->config.preamble_cycles;
			controller.endStep(channel, OUTCOME_SILENCE, step_cycles);
			
			// Start new step immediately
			fuzzyTokenStepStartCycle[channel] = current_cycle;
			fuzzyTokenActiveTransmitters[channel] = 0;
			// Reset all nodes
			for (auto& pair : fuzzyTokenNodes) {
				if (pair.first == channel) {
					pair.second->resetStepState();
				}
			}
		} else if (active_count > 1) {
			// COLLISION: Multiple preambles detected - ALL hubs must hold
			// Only process collision ONCE per step
			static int last_collision_cycle = -1;
			if (current_cycle != last_collision_cycle) {
				last_collision_cycle = current_cycle;
				
				cerr << "[COLLISION-DETECTED] Hub " << local_id << " (token holder) detected "
					 << active_count << " simultaneous transmitters at cycle " << current_cycle
					 << " (step started at " << fuzzyTokenStepStartCycle[channel] << ")" << endl;

				// COLLISION: Reduce FA_size, advance token, NO transmission this step
				// Report how long this collision step took (just preamble_cycles for detection)
				int step_cycles = state->config.preamble_cycles;
				controller.endStep(channel, OUTCOME_COLLISION, step_cycles);
				
				// Start new step immediately with reduced FA
				fuzzyTokenStepStartCycle[channel] = current_cycle;
				fuzzyTokenActiveTransmitters[channel] = 0;
				for (auto& pair : fuzzyTokenNodes) {
					if (pair.first == channel) {
						pair.second->resetStepState();
					}
				}
				
				cerr << "[COLLISION-STEP-END] Channel " << channel 
				     << " collision detected, FA_size reduced, all hubs hold, new step starting" << endl;
			}
			return; // End this step immediately, no transmission
		}
		// NOTE: Success outcome is handled when TAIL completes; if exactly one preamble was seen,
		// that node will be allowed to transmit after preamble+1 cycles (no NACK broadcast).
	}
	
	bool hasData = !init[channel]->buffer_tx.IsEmpty();
	
	static int has_data_count = 0;
	if (hasData) {
		has_data_count++;
		if (has_data_count % 100 == 0) {
			Flit flit = init[channel]->buffer_tx.Front();
			cerr << "[FUZZY-HAS-DATA #" << has_data_count << "] Hub " << local_id 
			     << " has data: src=" << flit.src_id << ", dst=" << flit.dst_id 
			     << ", type=" << flit.flit_type << ", seq=" << flit.sequence_no 
			     << ", tx_in_progress=" << transmission_in_progress.at(channel) << endl;
		}
	}
	
	// Determine if this node should attempt transmission
	bool shouldTransmit = node->shouldAttemptTransmission(hasData);
	
	static int no_transmit_count = 0;
	static int yes_transmit_count = 0;
	
	if (hasData && !shouldTransmit) {
		no_transmit_count++;
		if (no_transmit_count % 50 == 0) {
			cerr << "[FUZZY-SKIP #" << no_transmit_count << "] Hub " << local_id 
			     << " has data but shouldTransmit=false (token holder=" << node->isTokenHolder() 
			     << ", in FA=" << node->isInFuzzyArea()
			     << ", mode=" << (state->getMode() == FUZZY_MODE ? "FUZZY" : "FOCUSED") 
			     << ", tx_in_progress=" << transmission_in_progress.at(channel) << ")" << endl;
		}
	}
	
	if (hasData && shouldTransmit) {
		yes_transmit_count++;
		cerr << "[FUZZY-SHOULD-TX #" << yes_transmit_count << "] Hub " << local_id 
		     << " should transmit (mode=" << (state->getMode() == FUZZY_MODE ? "FUZZY" : "FOCUSED")
		     << ", token holder=" << node->isTokenHolder()
		     << ", tx_in_progress=" << transmission_in_progress.at(channel) << ")" << endl;
	}
	
	if (shouldTransmit)
	{
		FuzzyTokenMode mode = state->getMode();
		
		if (mode == FUZZY_MODE)
		{
			// PAPER-CORRECT: Fuzzy Token Protocol Phases
			// Phase 1 (Preamble): All nodes in FA send preambles
			// Phase 2 (Collision Detection): Token holder detects collision after preamble_cycles
			// Phase 3 (Transmission): Only if no collision, winner transmits
			
			// Phase 1: Send preamble (happens once per step)
			if (!node->hasSentPreamble())
			{
				node->startPreamble();
				
				// Register transmission with global controller (broadcast model)
				state->registerTransmission(local_id);
				
				cerr << "[PREAMBLE-SENT] Hub " << local_id << " sent preamble on channel " << channel 
				     << " (step start: " << fuzzyTokenStepStartCycle[channel] << ", current: " << current_cycle << ")" << endl;
				
				if (GlobalParams::verbose_mode == VERBOSE_HIGH)
				{
					cout << "Hub " << local_id << " sending preamble on channel " << channel
					     << " (FA mode)" << endl;
				}
				
				// DON'T start transmission yet - wait for collision detection phase!
				return;
			}
			
			// Phase 3: Transmission (only if preamble sent, no collision, and after collision check)
			// Wait for collision detection phase to complete PLUS one cycle for token holder to broadcast result
			if (step_duration < state->config.preamble_cycles + 1)
			{
				// Still in preamble/collision detection phase - don't transmit yet
				// Token holder needs preamble_cycles to collect all preambles, 
				// then +1 cycle to check and broadcast collision status
				return;
			}
			
			// Check if collision was detected this step
			// If activeTransmitters > 1, collision was detected by token holder
			if (state->getActiveTransmitters() > 1) {
				cerr << "[TX-ABORTED] Hub " << local_id << " aborting - collision detected ("
				     << state->getActiveTransmitters() << " transmitters), all hubs hold" << endl;
				return;
			}
			
			// Proceed with transmission - no collision detected
			if (!init[channel]->buffer_tx.IsEmpty())
			{
				Flit flit = init[channel]->buffer_tx.Front();
				
				static int fuzzy_tx_count = 0;
				if (!transmission_in_progress.at(channel)) {
					// First flit of packet
					fuzzy_tx_count++;
					cerr << "[FUZZY-TX-START #" << fuzzy_tx_count << "] Hub " << local_id 
					     << " starting packet transmission in FUZZY mode (after collision check):" << endl
					     << "    src=" << flit.src_id << ", dst=" << flit.dst_id 
					     << ", type=" << flit.flit_type << ", seq=" << flit.sequence_no 
					     << ", active_transmitters=" << state->getActiveTransmitters() << endl;
					
					// Mark that transmission started in this step
					fuzzyTokenTransmissionThisStep[channel] = true;
				}
				
				if (GlobalParams::verbose_mode == VERBOSE_HIGH)
				{
					cout << "Hub " << local_id << " requesting transmission of flit " << flit 
					     << " on channel " << channel << " (FA mode)" << endl;
				}
				
				// Notify Initiator to send next flit (HEAD, BODY, or TAIL)
				init[channel]->start_request_event.notify();
				
				// NOTE: endStep() is called in Initiator when transmission completes (TAIL sent)
			}
		}
		else // FOCUSED_MODE
		{
		// In focused mode, only token holder transmits (no preamble needed)
		// Note: Similar to TOKEN_PACKET, keep calling notify() for each flit until packet completes
		if (node->isTokenHolder() && hasData)
		{
			if (!init[channel]->buffer_tx.IsEmpty())
			{
				Flit flit = init[channel]->buffer_tx.Front();			static int focused_tx_count = 0;
			focused_tx_count++;
			cerr << "[FOCUSED-TX-START #" << focused_tx_count << "] Hub " << local_id 
			     << " starting transmission in FOCUSED mode:" << endl
			     << "    src=" << flit.src_id << ", dst=" << flit.dst_id 
			     << ", type=" << flit.flit_type << ", seq=" << flit.sequence_no << endl;				if (GlobalParams::verbose_mode == VERBOSE_HIGH)
				{
					cout << "Hub " << local_id << " (token holder) transmitting on channel "
					     << channel << " (focused mode)" << endl;
				}
				
				// OPTION 2: Mark that transmission started in this step
				// The step will end when TAIL flit is sent (in Initiator)
				fuzzyTokenTransmissionThisStep[channel] = true;
				
				init[channel]->start_request_event.notify();
				
				// NOTE: endStep() is called in Initiator when transmission completes (TAIL sent)
				// NOT here, because transmission is asynchronous
			}
		}
		}
	}
	else
	{
		// No transmission attempt from this hub
		// NOTE: Do NOT end step here! The synchronous step coordination at the top
		// of this function (lines 825-870) handles step end based on preamble collection.
		// Ending step here would cause premature resetStepState() before all preambles registered.
		
		// PAPER-CORRECT: Only the token holder's synchronous check should end steps,
		// after waiting for preamble_cycles to allow ALL nodes to register transmissions.
	}
}

void Hub::completeFuzzyTokenTransmission(int channel)
{
	cerr << "[DEBUG] completeFuzzyTokenTransmission called for channel " << channel << endl;
	
	// Check if this hub uses FUZZY_TOKEN MAC
	if (fuzzyTokenNodes.find(channel) == fuzzyTokenNodes.end()) {
		cerr << "[DEBUG] No fuzzyTokenNode for channel " << channel << ", returning" << endl;
		return; // Not using Fuzzy Token on this channel
	}
	
	FuzzyTokenController& controller = FuzzyTokenController::getInstance();
	FuzzyTokenChannelState* state = controller.getChannelState(channel);
	if (!state) {
		cerr << "[DEBUG] No channel state for channel " << channel << endl;
		return;
	}
	
	// Calculate transmission duration
	int current_cycle = (int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
	int step_start = fuzzyTokenStepStartCycle[channel];
	int transmission_cycles = current_cycle - step_start;
	
	// Transmission completed; apply deferred congestion if any
	if (fuzzyTokenDeferredCongestion[channel]) {
		controller.endStep(channel, OUTCOME_CONGESTION, transmission_cycles);
		cerr << "[FUZZY-STEP-END] Channel " << channel 
		     << " transmission complete with deferred CONGESTION, FA_size will decrease" << endl;
		fuzzyTokenDeferredCongestion[channel] = false; // consume the deferred flag
	} else {
		controller.endStep(channel, OUTCOME_SUCCESS, transmission_cycles);
		cerr << "[FUZZY-STEP-END] Channel " << channel 
		     << " transmission complete (SUCCESS), step ended, token advanced" << endl;
	}
	
	// Reset step state for next transmission
	if (fuzzyTokenNodes.find(channel) != fuzzyTokenNodes.end()) {
		fuzzyTokenNodes[channel]->resetStepState();
	}
	fuzzyTokenActiveTransmitters[channel] = 0;
	fuzzyTokenStepStartCycle[channel] = current_cycle; // Start new step
	fuzzyTokenTransmissionThisStep[channel] = false;
}

bool Hub::isFuzzyTokenChannel(int channel)
{
	return fuzzyTokenNodes.find(channel) != fuzzyTokenNodes.end();
}

bool Hub::isTokenHolderForChannel(int channel)
{
	if (fuzzyTokenNodes.find(channel) == fuzzyTokenNodes.end()) {
		return false;
	}
	return fuzzyTokenNodes[channel]->isTokenHolder();
}

void Hub::handleFuzzyTokenCollision(int channel)
{
	// Check if this hub uses FUZZY_TOKEN MAC
	if (fuzzyTokenNodes.find(channel) == fuzzyTokenNodes.end()) {
		return; // Not using Fuzzy Token on this channel
	}
	
	FuzzyTokenController& controller = FuzzyTokenController::getInstance();
	FuzzyTokenChannelState* state = controller.getChannelState(channel);
	if (!state) {
		return;
	}
	
	// Calculate collision duration (time since step start)
	int current_cycle = (int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
	int step_start = fuzzyTokenStepStartCycle[channel];
	int collision_cycles = current_cycle - step_start;
	
	// End the step with COLLISION outcome
	// This will trigger MULTIPLICATIVE DECREASE of FA_size (FA × 0.5)
	controller.endStep(channel, OUTCOME_COLLISION, collision_cycles);
	
	// Reset step state for next transmission
	if (fuzzyTokenNodes.find(channel) != fuzzyTokenNodes.end()) {
		fuzzyTokenNodes[channel]->resetStepState();
	}
	fuzzyTokenActiveTransmitters[channel] = 0;
	fuzzyTokenStepStartCycle[channel] = current_cycle; // Start new step
	fuzzyTokenTransmissionThisStep[channel] = false;
	
	cerr << "[FUZZY-COLLISION-END] Channel " << channel 
	     << " TX-ERROR detected (COLLISION), FA_size will decrease multiplicatively" << endl;
}

// Handle buffer overflow (receiver-side congestion, not a true MAC collision)
void Hub::handleBufferOverflow(int channel) {
	// Check if this hub uses FUZZY_TOKEN MAC
	if (fuzzyTokenNodes.find(channel) == fuzzyTokenNodes.end()) {
		return; // Not using Fuzzy Token on this channel
	}
	
	FuzzyTokenController& controller = FuzzyTokenController::getInstance();
	FuzzyTokenChannelState* state = controller.getChannelState(channel);
	if (!state) {
		return;
	}
	
	// Calculate step duration
	int current_cycle = (int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
	int step_start = fuzzyTokenStepStartCycle[channel];
	int step_cycles = current_cycle - step_start;
	
	// Treat RX overflow as CONGESTION for AIMD:
	// - It's not a MAC collision, but it indicates the wireless path is over-feeding the receiver.
	// - Decrease FA_size subtractively to reduce offered load next steps.
	controller.endStep(channel, OUTCOME_CONGESTION, step_cycles);
	
	// Reset step state for next transmission
	if (fuzzyTokenNodes.find(channel) != fuzzyTokenNodes.end()) {
		fuzzyTokenNodes[channel]->resetStepState();
	}
	fuzzyTokenActiveTransmitters[channel] = 0;
	fuzzyTokenStepStartCycle[channel] = current_cycle;
	fuzzyTokenTransmissionThisStep[channel] = false;
	
    cerr << "[BUFFER-OVERFLOW-END] Channel " << channel 
	    << " buffer overflow detected (NOT MAC collision) - treating as CONGESTION (AIMD decrease)" << endl;
}

// Handle TX-side congestion: enqueue-full when a new packet arrives
// Treat like a COLLISION for AIMD (multiplicative decrease of FA)
void Hub::handleFuzzyTokenCongestion(int channel)
{
	// Only applicable for FUZZY_TOKEN channels
	if (fuzzyTokenNodes.find(channel) == fuzzyTokenNodes.end()) {
		return;
	}

	FuzzyTokenController& controller = FuzzyTokenController::getInstance();
	FuzzyTokenChannelState* state = controller.getChannelState(channel);
	if (!state) {
		return;
	}

	// Defer congestion handling to end of current/next transmission step
	if (!fuzzyTokenDeferredCongestion[channel]) {
		fuzzyTokenDeferredCongestion[channel] = true; // coalesce multiple events into one
		cerr << "[FUZZY-CONGESTION-DEFER] Channel " << channel
		     << " HEAD enqueue-full detected; deferring FA decrease until step end" << endl;
	} else {
		// Already deferred for this step; nothing to do
	}
}
