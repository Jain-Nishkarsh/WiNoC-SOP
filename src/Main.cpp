/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the top-level of Noxim
 */

#include "ConfigurationManager.h"
#include "NoC.h"
#include "GlobalStats.h"
#include "DataStructs.h"
#include "GlobalParams.h"

#include <csignal>

using namespace std;

// need to be globally visible to allow "-volume" simulation stop
unsigned int drained_volume;
NoC *n;

void signalHandler( int signum )
{
    cout << "\b\b  " << endl;
    cout << endl;
    cout << "Current Statistics:" << endl;
    cout << "(" << sc_time_stamp().to_double() / GlobalParams::clock_period_ps << " sim cycles executed)" << endl;
    GlobalStats gs(n);
    gs.showStats(std::cout, GlobalParams::detailed);
}

int sc_main(int arg_num, char *arg_vet[])
{
    signal(SIGQUIT, signalHandler);  

    // TEMP
    drained_volume = 0;

    // Handle command-line arguments
    cout << "\t--------------------------------------------" << endl; 
    cout << "\t\tNoxim - the NoC Simulator" << endl;
    cout << "\t\t(C) University of Catania" << endl;
    cout << "\t--------------------------------------------" << endl; 

    cout << "Catania V., Mineo A., Monteleone S., Palesi M., and Patti D. (2016) Cycle-Accurate Network on Chip Simulation with Noxim. ACM Trans. Model. Comput. Simul. 27, 1, Article 4 (August 2016), 25 pages. DOI: https://doi.org/10.1145/2953878" << endl;
    cout << endl;
    cout << endl;

    configure(arg_num, arg_vet);

    // Initialize CSV Logs
    if (GlobalParams::csv_log_enabled) {
        // Routing Log (Conditional)
        if (!GlobalParams::csv_routing_log_filename.empty()) {
            GlobalParams::csv_routing_log_stream.open(GlobalParams::csv_routing_log_filename.c_str());
            if (GlobalParams::csv_routing_log_stream.is_open()) {
                GlobalParams::csv_routing_log_stream << "Cycle,NodeID,Src,Dst,FlitType,Seq,InputPort,OutputPort,VC" << endl;
                cout << "CSV Routing Log enabled: " << GlobalParams::csv_routing_log_filename << endl;
            } else {
                cerr << "Error opening CSV routing log file: " << GlobalParams::csv_routing_log_filename << endl;
            }
        }

        // MAC Log
        if (!GlobalParams::csv_mac_log_filename.empty()) {
            GlobalParams::csv_mac_log_stream.open(GlobalParams::csv_mac_log_filename.c_str());
            if (GlobalParams::csv_mac_log_stream.is_open()) {
                GlobalParams::csv_mac_log_stream << "sim_cycle,channel_id,step_id,event_type,event_duration,token_id,period_mode,fa_size,fa_center,fa_nodes,fa_update,fa_old_size,fa_new_size,tx_attempt_nodes,tx_success_node,ready_nodes,ready_in_fa_nodes,nack_sent,nack_sender" << endl;
                cout << "CSV MAC Log enabled: " << GlobalParams::csv_mac_log_filename << endl;
            } else {
                cerr << "Error opening CSV MAC log file: " << GlobalParams::csv_mac_log_filename << endl;
            }
        }
    }

    // Signals
    sc_clock clock("clock", GlobalParams::clock_period_ps, SC_PS);
    sc_signal <bool> reset;

    // NoC instance
    n = new NoC("NoC");

    n->clock(clock);
    n->reset(reset);

    // Trace signals
    sc_trace_file *tf = NULL;
    if (GlobalParams::trace_mode) {
	tf = sc_create_vcd_trace_file(GlobalParams::trace_filename.c_str());
	sc_trace(tf, reset, "reset");
	sc_trace(tf, clock, "clock");

	for (int i = 0; i < GlobalParams::mesh_dim_x; i++) {
	    for (int j = 0; j < GlobalParams::mesh_dim_y; j++) {
		char label[64];

		sprintf(label, "req(%02d)(%02d).east", i, j);
		sc_trace(tf, n->req[i][j].east, label);
		sprintf(label, "req(%02d)(%02d).west", i, j);
		sc_trace(tf, n->req[i][j].west, label);
		sprintf(label, "req(%02d)(%02d).south", i, j);
		sc_trace(tf, n->req[i][j].south, label);
		sprintf(label, "req(%02d)(%02d).north", i, j);
		sc_trace(tf, n->req[i][j].north, label);

		sprintf(label, "ack(%02d)(%02d).east", i, j);
		sc_trace(tf, n->ack[i][j].east, label);
		sprintf(label, "ack(%02d)(%02d).west", i, j);
		sc_trace(tf, n->ack[i][j].west, label);
		sprintf(label, "ack(%02d)(%02d).south", i, j);
		sc_trace(tf, n->ack[i][j].south, label);
		sprintf(label, "ack(%02d)(%02d).north", i, j);
		sc_trace(tf, n->ack[i][j].north, label);
	    }
	}
    }
    // Reset the chip and run the simulation
    reset.write(1);
    cout << "Reset for " << (int)(GlobalParams::reset_time) << " cycles... ";
    srand(GlobalParams::rnd_generator_seed);

    // fix clock periods different from 1ns
    //sc_start(GlobalParams::reset_time, SC_NS);
    sc_start(GlobalParams::reset_time * GlobalParams::clock_period_ps, SC_PS);

    reset.write(0);
    cout << " done! " << endl;
    cout << " Now running for " << GlobalParams:: simulation_time << " cycles..." << endl;
    // fix clock periods different from 1ns
    //sc_start(GlobalParams::simulation_time, SC_NS);
    sc_start(GlobalParams::simulation_time * GlobalParams::clock_period_ps, SC_PS);


    // Close the simulation
    if (GlobalParams::trace_mode) sc_close_vcd_trace_file(tf);
    
    cout << "Noxim simulation completed.";
    cout << " (" << sc_time_stamp().to_double() / GlobalParams::clock_period_ps << " cycles executed)" << endl;
    cout << endl;
//assert(false);
    // Show statistics
    GlobalStats gs(n);
    gs.showStats(std::cout, GlobalParams::detailed);

    // Close CSV Logs
    if (GlobalParams::csv_log_enabled) {
        if (GlobalParams::csv_routing_log_stream.is_open()) {
            GlobalParams::csv_routing_log_stream.close();
        }
        if (GlobalParams::csv_mac_log_stream.is_open()) {
            GlobalParams::csv_mac_log_stream.close();
        }
    }

    if ((GlobalParams::max_volume_to_be_drained > 0) &&
	(sc_time_stamp().to_double() / GlobalParams::clock_period_ps - GlobalParams::reset_time >=
	 GlobalParams::simulation_time)) {
	cout << endl
         << "WARNING! the number of flits specified with -volume option" << endl
	     << "has not been reached. ( " << drained_volume << " instead of " << GlobalParams::max_volume_to_be_drained << " )" << endl
         << "You might want to try an higher value of simulation cycles" << endl
	     << "using -sim option." << endl;

#ifdef TESTING
	cout << endl
         << " Sum of local drained flits: " << gs.drained_total << endl
	     << endl
         << " Effective drained volume: " << drained_volume;
#endif

    }

#ifdef DEADLOCK_AVOIDANCE
	cout << "***** WARNING: DEADLOCK_AVOIDANCE ENABLED!" << endl;
#endif
    return 0;
}
