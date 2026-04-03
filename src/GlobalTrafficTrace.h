/*
 * Noxim - the NoC Simulator
 *
 * This file contains the definition of the global traffic trace controller
 */

#ifndef __NOXIMGLOBALTRAFFIC_TRACE_H__
#define __NOXIMGLOBALTRAFFIC_TRACE_H__

#include <fstream>
#include <string>
#include <vector>

using namespace std;

struct TraceCommunication {
    int dst;
    int message_size;
};

class GlobalTrafficTrace {
  public:
    GlobalTrafficTrace();

    // Load one trace file per source node using the format <id>_<base_filename>
    // Returns true if all files were loaded successfully.
    bool load(const char *base_filename, int num_sources, int flit_headtail_size);

    // Returns true when src has a ready trace entry and it is not waiting for delivery.
    bool canSend(int src);

    // Returns the next communication for src without consuming it.
    bool peekNext(int src, TraceCommunication &comm);

    // Consume the current trace entry and lock src until delivery notification.
    void onPacketQueued(int src);

    // Unlock src when the destination receives the tail flit for a packet from src.
    void onPacketDelivered(int src);

    // True when every source trace has ended and no source is locked.
    bool isSimulationComplete();

    inline int getFlitHeadTailSize() const { return flit_headtail_size; }

  private:
    struct SourceTraceState {
        ifstream fin;
        string filename;
        bool eof_reached;
        bool has_cached;
        bool locked;
        TraceCommunication cached;

        SourceTraceState() : eof_reached(false), has_cached(false), locked(false) {
            cached.dst = -1;
            cached.message_size = 0;
        }
    };

    vector<SourceTraceState> sources;
    int flit_headtail_size;

    bool prefetchNext(int src);
    static bool parseTraceLine(const string &line, TraceCommunication &comm);
    static string buildTraceFilename(const string &base_filename, int src, int width);
};

#endif
