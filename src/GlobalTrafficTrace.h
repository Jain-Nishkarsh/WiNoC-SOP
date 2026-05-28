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

    bool load(const char *base_filename, int num_sources, int flit_headtail_size);

    bool getNext(int src, TraceCommunication &comm);

    inline int getFlitHeadTailSize() const { return flit_headtail_size; }

  private:
    struct SourceTraceState {
        ifstream fin;
        string filename;
        bool eof_reached;
        bool has_cached;
        TraceCommunication cached;

        SourceTraceState() : eof_reached(false), has_cached(false) {
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
