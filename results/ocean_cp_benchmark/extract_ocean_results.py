#!/usr/bin/env python3
"""
extract_ocean_results.py
Computes geometric mean latency and throughput across 10 seeds per protocol.
"""
import glob
import math
import re
from pathlib import Path

PROTOCOLS = {
    'token_packet': 'Token Passing',
    'fuzzy_token': 'Fuzzy Token',
    'fuzzy_swj': 'FP-Jump',
}

REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_DIR = REPO_ROOT / "results"

print(f"\n{'Protocol':<20} {'Avg Latency (gmean)':<25} {'Throughput (gmean)'}")
print("-" * 65)

for proto_key, proto_name in PROTOCOLS.items():
    latencies = []
    throughputs = []

    for result_file in glob.glob(str(RESULTS_DIR / f'ocean_cp/{proto_key}_seed*.log')):
        try:
            text = Path(result_file).read_text()
            lat_match = re.search(r'% Global average delay \(cycles\):\s*([0-9.]+)', text)
            tput_match = re.search(r'% Network throughput \(flits/cycle\):\s*([0-9.]+)', text)
            if not lat_match or not tput_match:
                raise ValueError('missing latency/throughput fields')
            lat = float(lat_match.group(1))
            tput = float(tput_match.group(1))
            latencies.append(lat)
            throughputs.append(tput)
        except Exception as e:
            print(f"  Warning: could not parse {result_file}: {e}")

    if latencies:
        lat_gmean = math.exp(sum(math.log(x) for x in latencies) / len(latencies))
        tput_gmean = math.exp(sum(math.log(x) for x in throughputs) / len(throughputs))
        print(f"{proto_name:<20} {lat_gmean:<25.4f} {tput_gmean:.6f}")
    else:
        print(f"{proto_name:<20} NO RESULTS FOUND")
