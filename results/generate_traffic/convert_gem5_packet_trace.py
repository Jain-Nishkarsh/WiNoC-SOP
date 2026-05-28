import argparse
import re
import subprocess
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert gem5 MemTraceProbe output to Noxim trace files."
    )
    parser.add_argument(
        "--gem5-root",
        type=str,
        default=str(Path.home() / "gem5"),
        help="Path to gem5 repository root",
    )
    parser.add_argument(
        "--trace",
        type=str,
        required=True,
        help="MemTraceProbe output file (.trc or .trc.gz)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=str(Path.cwd() / "traces" / "ocean_cp_64c"),
        help="Directory for per-core Noxim traces",
    )
    parser.add_argument(
        "--trace-base",
        type=str,
        default="ocean_cp",
        help="Base filename for per-core traces",
    )
    parser.add_argument(
        "--num-cores",
        type=int,
        default=64,
        help="Number of cores (source nodes)",
    )
    parser.add_argument(
        "--mc-nodes",
        type=str,
        default="3,24,31,59",
        help="Comma-separated MC node IDs (index order)",
    )
    parser.add_argument(
        "--interleave-bits",
        type=int,
        default=12,
        help="Low bits for address interleaving (4KB = 12)",
    )
    parser.add_argument(
        "--payload-bytes",
        type=int,
        default=6,
        help="Fixed payload bytes per packet",
    )
    parser.add_argument(
        "--data-only",
        action="store_true",
        help="Include only dcache requestors",
    )
    parser.add_argument(
        "--all-cmds",
        action="store_true",
        help="Include non read/write requests",
    )
    return parser.parse_args()


def ensure_packet_proto(gem5_root: Path):
    util_dir = gem5_root / "util"
    sys.path.insert(0, str(util_dir))

    try:
        import packet_pb2  # noqa: F401
    except ImportError:
        subprocess.check_call(["make", "--quiet", "-C", str(util_dir), "packet_pb2.py"])
        import packet_pb2  # noqa: F401

    import protolib  # noqa: F401
    return util_dir


def build_requestor_map(header, data_only):
    req_to_core = {}
    pattern = re.compile(r"cpu(\d+)")
    for entry in header.id_strings:
        name = entry.value
        match = pattern.search(name)
        if not match:
            continue
        if data_only and "dcache" not in name:
            continue
        core = int(match.group(1))
        req_to_core[entry.key] = core
    return req_to_core


def main():
    args = parse_args()
    gem5_root = Path(args.gem5_root).resolve()
    trace_path = Path(args.trace).resolve()
    output_dir = Path(args.output_dir).resolve()

    if not trace_path.exists():
        print(f"Trace file not found: {trace_path}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)

    mc_nodes = [int(x.strip()) for x in args.mc_nodes.split(",") if x.strip()]
    if not mc_nodes:
        print("No MC nodes provided.", file=sys.stderr)
        return 1

    ensure_packet_proto(gem5_root)
    sys.path.insert(0, str(gem5_root / "util"))
    import packet_pb2
    import protolib

    proto_in = protolib.openFileRd(str(trace_path))
    magic = proto_in.read(4).decode()
    if magic != "gem5":
        print(f"Unrecognized trace file: {trace_path}", file=sys.stderr)
        return 1

    header = packet_pb2.PacketHeader()
    if not protolib.decodeMessage(proto_in, header):
        print("Failed to read packet header.", file=sys.stderr)
        return 1

    req_to_core = build_requestor_map(header, args.data_only)
    if not req_to_core:
        print("No requestor IDs matched CPU cores.", file=sys.stderr)
        return 1

    width = max(3, len(str(args.num_cores - 1)))
    writers = {}
    for core in range(args.num_cores):
        fname = f"{core:0{width}d}_{args.trace_base}"
        writers[core] = (output_dir / fname).open("w")

    packet = packet_pb2.Packet()
    written = 0
    skipped = 0

    while protolib.decodeMessage(proto_in, packet):
        if not packet.HasField("pkt_id"):
            skipped += 1
            continue

        if not args.all_cmds and packet.cmd not in (1, 4):
            skipped += 1
            continue

        core = req_to_core.get(packet.pkt_id)
        if core is None or core >= args.num_cores:
            skipped += 1
            continue

        mc_index = (packet.addr >> args.interleave_bits) % len(mc_nodes)
        dst = mc_nodes[mc_index]

        writers[core].write(f"{dst} {args.payload_bytes}\n")
        written += 1

    for fh in writers.values():
        fh.close()

    print(f"Wrote {written} packets into {output_dir}")
    print(f"Skipped {skipped} packets")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
