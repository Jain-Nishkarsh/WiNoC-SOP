#!/usr/bin/env python3
import os
import sys
import argparse
from collections import defaultdict
from pathlib import Path

MESH_X = 8
MESH_Y = 8
NUM_NODES = 64
MC_NODE_IDS = {3, 24, 31, 59}

def convert_traces(input_dir, output_dir, num_nodes=NUM_NODES, filter_core_only=True):
    os.makedirs(output_dir, exist_ok=True)
    stats = defaultdict(int)
    per_node_counts = defaultdict(int)
    per_node_dest_distinct = defaultdict(set)

    print("[TraceConvert] Starting conversion...")
    print(f"  Input directory:   {input_dir}")
    print(f"  Output directory:  {output_dir}")
    print(f"  Num nodes:         {num_nodes}")
    print(f"  Filter core-only:  {filter_core_only}")

    missing_files = []
    for node_id in range(num_nodes):
        candidates = [
            os.path.join(input_dir, f"node{node_id:02d}.trace"),
            os.path.join(input_dir, f"node_{node_id}.trace"),
            os.path.join(input_dir, f"node{node_id}.trace"),
            os.path.join(input_dir, f"{node_id}.trace"),
            os.path.join(input_dir, f"{node_id:03d}_ocean_cp"),
            os.path.join(input_dir, f"{node_id:02d}_ocean_cp"),
        ]
        input_file = None
        for c in candidates:
            if os.path.exists(c):
                input_file = c
                break
        if not input_file:
            missing_files.append(node_id)
            continue

        output_file = os.path.join(output_dir, f"{node_id:03d}_ocean_cp")
        packets_written = 0
        packets_filtered = 0
        packets_parse_error = 0

        with open(input_file, 'r') as fin, open(output_file, 'w') as fout:
            for line in fin:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(None, 2)
                if len(parts) < 2:
                    packets_parse_error += 1
                    stats['parse_failures'] += 1
                    continue
                try:
                    dst = int(parts[0])
                    size = int(parts[1])
                except ValueError:
                    packets_parse_error += 1
                    stats['parse_failures'] += 1
                    continue
                if size < 8:
                    stats['tiny_messages'] += 1
                    continue
                if dst >= num_nodes and filter_core_only:
                    stats['non_core_dropped'] += 1
                    continue
                fout.write(f"{dst} {size}\n")
                packets_written += 1
                stats['total_packets'] += 1
                per_node_counts[node_id] += 1
                per_node_dest_distinct[node_id].add(dst)
                if filter_core_only and dst in MC_NODE_IDS:
                    stats['c2m_packets'] += 1
                else:
                    stats['c2c_packets'] += 1

        if packets_written > 0:
            print(f"  [node {node_id:2d}] Wrote {packets_written:,} packets "
                  f"(filtered {packets_filtered}, errors {packets_parse_error})")

    if missing_files:
        print(f"\nWarning: Missing trace files for nodes: {missing_files}")

    total = stats['c2c_packets'] + stats['c2m_packets']
    c2c_ratio = stats['c2c_packets'] / max(1, stats['c2m_packets'])

    print("\n" + "="*70)
    print("TRACE CONVERSION REPORT")
    print("="*70)
    print(f"  Total packets        : {stats['total_packets']:,}")
    print(f"  Core-to-core (C2C)   : {stats['c2c_packets']:,}")
    print(f"  Core-to-MC   (C2M)   : {stats['c2m_packets']:,}")
    print(f"  C2C / C2M ratio      : {c2c_ratio:.2f}x")
    print(f"  Non-core dropped     : {stats['non_core_dropped']:,}")
    print(f"  Tiny messages        : {stats['tiny_messages']:,}")
    print(f"  Parse failures       : {stats['parse_failures']:,}")

    if per_node_counts:
        counts = sorted(
            [(nid, per_node_counts[nid]) for nid in range(num_nodes) if per_node_counts[nid] > 0],
            key=lambda x: -x[1]
        )
        print(f"\n  Nodes with traffic   : {len(counts)}/{num_nodes}")
        if counts:
            print(f"  Max injector: node {counts[0][0]} -> {counts[0][1]:,} pkts")
            print(f"  Min injector: node {counts[-1][0]} -> {counts[-1][1]:,} pkts")
            print(f"  Mean packets/node   : {sum(c[1] for c in counts)/len(counts):,.0f}")

        print("\n  Per-node destination count (unique dsts):")
        for nid in sorted(per_node_dest_distinct.keys()):
            dsts = per_node_dest_distinct[nid]
            print(f"    node {nid:2d}: {len(dsts)} unique destinations")

    passed = True
    checks = []
    if c2c_ratio >= 5.0:
        checks.append(f"PASS C2C/C2M ratio {c2c_ratio:.1f}x >= 5x")
    else:
        checks.append(f"FAIL C2C/C2M ratio {c2c_ratio:.1f}x < 5x (coherence missing)")
        passed = False
    if stats['parse_failures'] == 0:
        checks.append("PASS No parse errors")
    else:
        checks.append(f"FAIL {stats['parse_failures']} parse errors")
        passed = False

    print("\n" + "-"*70)
    for c in checks:
        print(f"  {c}")
    if passed:
        print("\nResult: ALL CHECKS PASSED")
        return 0
    else:
        print("\nResult: SOME CHECKS FAILED")
        return 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Convert Ruby NI traces to Noxim format")
    parser.add_argument('--input_dir', default='ruby_ni_traces', help='Input trace directory')
    parser.add_argument('--output_dir', default='ocean_cp_64c', help='Output directory')
    parser.add_argument('--num_nodes', type=int, default=64, help='Number of mesh nodes')
    parser.add_argument('--no-filter', action='store_true', help='Include non-core destinations')
    args = parser.parse_args()
    sys.exit(convert_traces(
        args.input_dir, args.output_dir,
        num_nodes=args.num_nodes,
        filter_core_only=not args.no_filter
    ))
