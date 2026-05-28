#!/usr/bin/env python3
"""
convert_to_noxim.py
Converts gem5 MessageBuffer traces to per-node Noxim .trf format.

Input format  (from gem5): cycle src dst flits
Output format (for Noxim): dst flits   (one file per src node)
"""
import os
import glob
import collections
import argparse
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def convert(input_dir, output_dir, num_nodes=64,
            roi_start_cycle=None, roi_end_cycle=None):

    os.makedirs(output_dir, exist_ok=True)

    # Collect all packets from all trace files
    packets_by_src = collections.defaultdict(list)
    total = 0
    skipped = 0

    for trace_file in sorted(glob.glob(os.path.join(input_dir, '*'))):
        with open(trace_file) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) != 4:
                    skipped += 1
                    continue
                try:
                    cycle, src, dst, flits = int(parts[0]), int(parts[1]), \
                                             int(parts[2]), int(parts[3])
                except ValueError:
                    skipped += 1
                    continue

                # ROI filter
                if roi_start_cycle and cycle < roi_start_cycle:
                    continue
                if roi_end_cycle and cycle > roi_end_cycle:
                    continue

                # Sanity checks
                if src == dst:
                    continue
                if src < 0 or src >= num_nodes:
                    continue
                if dst < 0 or dst >= num_nodes:
                    continue

                packets_by_src[src].append((dst, flits))
                total += 1

    # Write one .trf file per source node
    for node_id in range(num_nodes):
        out_file = os.path.join(output_dir, f'node{node_id:02d}.trf')
        packets = packets_by_src.get(node_id, [])
        with open(out_file, 'w') as f:
            f.write(f"# ocean_cp trace for source node {node_id}\n")
            f.write("# Format: dst_node num_flits\n")
            for dst, flits in packets:
                f.write(f"{dst} {flits}\n")

    # Sanity report
    mc_nodes = {3, 24, 31, 59}
    c2c = sum(1 for packets in packets_by_src.values()
              for dst, _ in packets if dst not in mc_nodes)
    c2m = total - c2c

    print(f"\n{'=' * 50}")
    print("CONVERSION COMPLETE")
    print(f"Total packets written : {total:,}")
    print(f"Core-to-core (C2C)   : {c2c:,}")
    print(f"Core-to-MC   (C2M)   : {c2m:,}")
    if c2m > 0:
        print(f"C2C/C2M ratio        : {c2c / c2m:.1f}x  (want >= 5)")
    print(f"Skipped lines        : {skipped}")
    print(f"Output directory     : {output_dir}")

    all_counts = [len(packets_by_src.get(n, [])) for n in range(num_nodes)]
    if max(all_counts) > 0:
        uniformity = min(all_counts) / max(all_counts)
        print(f"Spatial uniformity   : {uniformity:.2f} (want < 0.70)")
        if uniformity > 0.70:
            print("WARNING: Traffic too uniform — spatial structure may be lost")
        if c2c == 0:
            print("CRITICAL: Zero C2C traffic — traces still broken (MC-only)")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--input',
        default=str(REPO_ROOT / 'results' / 'generate_traffic' / 'gem5_ocean_cp_run1' / 'm5out'),
    )
    parser.add_argument(
        '--output',
        default=str(REPO_ROOT / 'results' / 'generate_traffic' / 'traces' / 'ocean_cp_64c'),
    )
    parser.add_argument('--nodes', type=int, default=64)
    args = parser.parse_args()
    convert(args.input, args.output)
