/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the buffer
 */
#include "Hub.h"
#include "Initiator.h"
#include "FuzzyTokenController.h"
#include "GlobalParams.h"

static int total_wireless_tx_attempts = 0;
static int total_wireless_tx_success = 0;
static int total_wireless_tx_errors = 0;

void Initiator::thread_process()
{

	tlm::tlm_generic_payload* trans = new tlm::tlm_generic_payload;
	tlm::tlm_phase phase;
	sc_time delay;

	while (1)
	{
		LOG << " *** waiting for transmissions" << endl;

		wait(start_request_event);

		tlm::tlm_command cmd = tlm::TLM_WRITE_COMMAND;
		flit_payload = buffer_tx.Front();
		hub->power.antennaBufferFront();

		int destHub;

		// hub relay management  ////////////////////////////////////////////////////////////////
		// if explicitly set in the header flit, trasmission target should reach a relay hub
		if (flit_payload.flit_type == FLIT_TYPE_HEAD)
		{
			if (flit_payload.hub_relay_node!=NOT_VALID) {
				current_hub_relay = flit_payload.hub_relay_node;
				LOG << "HUB RELAY: Flit " << flit_payload << " setting transmission hub relay " << current_hub_relay << " to reach destination " << endl;
			}
			else
				current_hub_relay = NOT_VALID;
		}

		if (current_hub_relay!=NOT_VALID)
		{
			flit_payload.hub_relay_node = current_hub_relay;
			destHub = tile2Hub(flit_payload.hub_relay_node);
		}
		else
		{
			destHub = tile2Hub(flit_payload.dst_id);
		}
		////////////////////////////////////////////////////////////////////////////////


	LOG << " *** Starting transmission of " << flit_payload << " to reach HUB_" << destHub <<  endl;
	total_wireless_tx_attempts++;
	// Per-hub aggregated attempts
	hub->tx_attempts_total++;
		
		cerr << "[INITIATOR-TX #" << total_wireless_tx_attempts << "] Hub " << hub->local_id 
		     << " sending flit: type=" << flit_payload.flit_type 
		     << ", src=" << flit_payload.src_id << ", dst=" << flit_payload.dst_id 
		     << ", seq=" << flit_payload.sequence_no << " to Hub " << destHub << endl;

		trans->set_command(cmd);
		trans->set_address(static_cast<const uint64>(destHub));

		trans->set_data_ptr( reinterpret_cast<unsigned char*>(&flit_payload) );
		trans->set_data_length( sizeof(Flit) );
		trans->set_streaming_width( sizeof(Flit) ); // = data_length to indicate no streaming
		trans->set_byte_enable_ptr( 0 ); // 0 indicates unused
		trans->set_dmi_allowed( false ); // Mandatory initial value
		trans->set_response_status( tlm::TLM_INCOMPLETE_RESPONSE ); // Mandatory initial value

		delay = sc_time(0, SC_PS);

		// Call b_transport to demonstrate the b/nb conversion by the simple_target_socket
		socket->b_transport( *trans, delay);

		hub->power.wirelessTx(hub->local_id,destHub,GlobalParams::flit_size);

		// Initiator obliged to check response status and delay
		if (!trans->is_response_error() )
		{
			total_wireless_tx_success++;
			hub->tx_success_total++;
			cerr << "[TX-SUCCESS] Hub " << hub->local_id << " sent flit type=" << flit_payload.flit_type 
			     << " (HEAD=" << FLIT_TYPE_HEAD << ", BODY=" << FLIT_TYPE_BODY << ", TAIL=" << FLIT_TYPE_TAIL << ")" << endl;
			if (flit_payload.flit_type == FLIT_TYPE_HEAD) {
				cerr << "[TX-OK] Hub " << hub->local_id << " successfully sent HEAD flit (src=" << flit_payload.src_id 
				     << ", dst=" << flit_payload.dst_id << ", seq=" << flit_payload.sequence_no 
				     << ") [Success #" << total_wireless_tx_success << "]" << endl;
			}
			buffer_tx.Pop();
			hub->power.antennaBufferPop();

			if (flit_payload.flit_type == FLIT_TYPE_HEAD) {
				hub->transmission_in_progress.at(_channel_id) = true;
				cerr << "[INITIATOR-HEAD] Hub " << hub->local_id << " set transmission_in_progress=true for channel " << _channel_id << endl;
			}

			if (flit_payload.flit_type == FLIT_TYPE_TAIL)
			{
				LOG << "*** [Ch"<< _channel_id <<"] tail flit sent " << flit_payload << ", releasing token" << endl;
				cerr << "[INIT-DEBUG] TAIL flit sent on channel " << _channel_id << endl;
				hub->flag[_channel_id]->write(RELEASE_CHANNEL);
				hub->transmission_in_progress.at(_channel_id) = false;
				
				// AIMD FIX: TAIL sent successfully = SUCCESS outcome
				// For FUZZY_TOKEN, the winning transmitter ends the step regardless of token holder
				if (hub->isFuzzyTokenChannel(_channel_id)) {
					cerr << "[AIMD-SUCCESS] Hub " << hub->local_id << " successfully sent TAIL on channel " 
						 << _channel_id << ", calling endStep(OUTCOME_SUCCESS)" << endl;
					hub->completeFuzzyTokenTransmission(_channel_id);
				}
			}
		}
		else
		{
			total_wireless_tx_errors++;
			hub->tx_errors_total++;
			LOG << " WARNING: incomplete transaction " << endl;
			cerr << "[TX-ERROR] Hub " << hub->local_id << " FAILED to send flit (src=" << flit_payload.src_id 
			     << ", dst=" << flit_payload.dst_id << ", type=" << flit_payload.flit_type 
			     << ", seq=" << flit_payload.sequence_no << ") to Hub " << destHub 
			     << " [Error #" << total_wireless_tx_errors << "]" << endl;
			
			// IMPORTANT: TX-ERROR means receiver buffer overflow, NOT MAC collision!
			// Do NOT treat as AIMD collision - the channel itself is not congested
			// The wireless channel worked fine - just receiver buffer was full
			// Treat as SILENCE so FA doesn't shrink unnecessarily
			if (hub->isFuzzyTokenChannel(_channel_id)) {
				cerr << "[BUFFER-OVERFLOW] Hub " << hub->local_id << " detected buffer overflow (not MAC collision) on channel " 
					 << _channel_id << " - ending step as SILENCE" << endl;
				hub->handleBufferOverflow(_channel_id);
			}
		}

		//check_transaction( *trans );

	}

}



