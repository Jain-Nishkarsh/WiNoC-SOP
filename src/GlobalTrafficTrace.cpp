#include "GlobalTrafficTrace.h"
#include "GlobalParams.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

GlobalTrafficTrace::GlobalTrafficTrace()
{
    flit_headtail_size = 0;
}

bool GlobalTrafficTrace::load(const char *base_filename, int num_sources, int _flit_headtail_size)
{
    if (base_filename == NULL || num_sources <= 0)
        return false;

    flit_headtail_size = _flit_headtail_size;
    sources.clear();
    sources.resize(num_sources);

    int width = 1;
    int max_id = std::max(0, num_sources - 1);
    while (max_id >= 10) {
        width++;
        max_id /= 10;
    }
    width = std::max(width, 3);

    for (int src = 0; src < num_sources; src++) {
        string filename = buildTraceFilename(base_filename, src, width);
        sources[src].filename = filename;
        if (GlobalParams::verbose_mode == VERBOSE_HIGH)
            cerr << "[TRACE-LOAD] opening " << filename << endl;
        sources[src].fin.open(filename.c_str(), ios::in);
        if (!sources[src].fin) {
            cerr << "[TRACE-LOAD-ERROR] missing trace file: " << filename << endl;
            return false;
        }
    }

    if (GlobalParams::verbose_mode == VERBOSE_HIGH || GlobalParams::verbose_mode == VERBOSE_MEDIUM)
        cerr << "[TRACE-LOAD] loaded " << num_sources << " trace files" << endl;

    return true;
}

bool GlobalTrafficTrace::getNext(int src, TraceCommunication &comm)
{
    if (src < 0 || src >= (int)sources.size())
        return false;

    if (!prefetchNext(src))
        return false;

    comm = sources[src].cached;
    sources[src].has_cached = false;
    return true;
}

bool GlobalTrafficTrace::prefetchNext(int src)
{
    if (src < 0 || src >= (int)sources.size())
        return false;

    SourceTraceState &state = sources[src];

    if (state.has_cached)
        return true;

    if (state.eof_reached)
        return false;

    string line;
    while (std::getline(state.fin, line)) {
        TraceCommunication comm;
        if (parseTraceLine(line, comm)) {
            state.cached = comm;
            state.has_cached = true;
            return true;
        }
    }

    state.eof_reached = true;
    return false;
}

bool GlobalTrafficTrace::parseTraceLine(const string &line, TraceCommunication &comm)
{
    if (line.empty())
        return false;

    size_t first = line.find_first_not_of(" \t\r\n");
    if (first == string::npos)
        return false;

    char c = line[first];
    if (c == '%' || c == '#')
        return false;

    istringstream iss(line);
    int dst = -1;
    int message_size = 0;
    if (!(iss >> dst >> message_size))
        return false;

    if (dst < 0 || message_size < 0)
        return false;

    comm.dst = dst;
    comm.message_size = message_size;
    return true;
}

string GlobalTrafficTrace::buildTraceFilename(const string &base_filename, int src, int width)
{
    size_t slash = base_filename.find_last_of("/\\");

    string path;
    string base;

    if (slash == string::npos) {
        path = "";
        base = base_filename;
    } else {
        path = base_filename.substr(0, slash + 1);
        base = base_filename.substr(slash + 1);
    }

    ostringstream oss;
    oss << path << setfill('0') << setw(width) << src << "_" << base;
    return oss.str();
}
