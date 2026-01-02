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
#include "Target.h"

static int total_wireless_rx_attempts = 0;
static int total_wireless_rx_success = 0;
static int total_wireless_rx_dropped = 0;

void Target::b_transport( tlm::tlm_generic_payload& trans, sc_time& delay )
{
    struct Flit* my_flit = (struct Flit*)trans.get_data_ptr();
    total_wireless_rx_attempts++;

    LOG << "*** [Ch" <<local_id << "] Received: " << *my_flit << endl;
    if (GlobalParams::verbose_mode == VERBOSE_HIGH && my_flit->flit_type == FLIT_TYPE_HEAD) {
        cerr << "[RX] Hub " << hub->local_id << " received HEAD flit on channel " << local_id 
             << " (src=" << my_flit->src_id << ", dst=" << my_flit->dst_id 
             << ", seq=" << my_flit->sequence_no << ") [RX attempt #" << total_wireless_rx_attempts << "]" << endl;
    }

    // only moves received flit to the antenna buffer
    // reservations stuff is done in the hub to avoid 
    // race conditions on shared reservation table
    if (!buffer_rx.IsFull())
    {
        LOG << "*** [Ch" <<local_id << "] Flit " << *my_flit << " moved to buffer_rx " << endl;
        buffer_rx.Push(*my_flit);
        hub->power.antennaBufferPush();
        total_wireless_rx_success++;
        if (GlobalParams::verbose_mode == VERBOSE_HIGH && my_flit->flit_type == FLIT_TYPE_HEAD) {
            cerr << "[RX-OK] Hub " << hub->local_id << " buffered HEAD flit (src=" << my_flit->src_id 
                 << ", dst=" << my_flit->dst_id << ", seq=" << my_flit->sequence_no 
                 << ") [Buffered #" << total_wireless_rx_success << "]" << endl;
        }
        // Obliged to set response status to indicate successful completion
        trans.set_response_status( tlm::TLM_OK_RESPONSE );
        //buffer_rx.Print();
    }
    else
    {
        // the response status will remain ERRROR
        // signaling to the Initiator that something went wrong
        total_wireless_rx_dropped++;
        LOG << "[Ch" <<local_id << "] WARNING: buffer_rx is full cannot store flit " << *my_flit << endl;
        if (GlobalParams::verbose_mode == VERBOSE_HIGH) {
            cerr << "[RX-DROP] Hub " << hub->local_id << " DROPPED flit (src=" << my_flit->src_id 
                 << ", dst=" << my_flit->dst_id << ", type=" << my_flit->flit_type 
                 << ", seq=" << my_flit->sequence_no << ") - buffer_rx FULL [Dropped #" << total_wireless_rx_dropped << "]" << endl;
        }
    }
}

// Static accessor functions for wireless statistics
int Target::getTotalWirelessRxAttempts() {
    return total_wireless_rx_attempts;
}

int Target::getTotalWirelessRxSuccess() {
    return total_wireless_rx_success;
}

int Target::getTotalWirelessRxDropped() {
    return total_wireless_rx_dropped;
}

