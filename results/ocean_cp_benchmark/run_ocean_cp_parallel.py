import argparse
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BIN_DIR = REPO_ROOT / "bin"
CONFIG_DIR = REPO_ROOT / "configs" / "custom_config"
OUTPUT_DIR = REPO_ROOT / "results" / "ocean_cp_runs"

DEFAULT_CONFIGS = {
    "token_packet": CONFIG_DIR / "ocean_cp_trace_token_packet_8x8_64hubs.yaml",
    "fuzzy_token": CONFIG_DIR / "ocean_cp_trace_fuzzy_token_8x8_64hubs.yaml",
    "fuzzy_swj": CONFIG_DIR / "ocean_cp_trace_fuzzy_swj_8x8_64hubs.yaml",
}

DEFAULT_TRACE_BASE = (
    REPO_ROOT / "results" / "generate_traffic" / "traces" / "ocean_cp_64c" / "ocean_cp"
).resolve()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run ocean_cp trace simulations in parallel."
    )
    parser.add_argument("--seed", type=int, default=1, help="RNG seed for all runs")
    parser.add_argument(
        "--sim-cycles",
        type=int,
        default=None,
        help="Override simulation_time for all runs",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=max(1, min(3, (os.cpu_count() or 2) - 1)),
        help="Parallel workers (default: min(3, cpu_count-1))",
    )
    parser.add_argument(
        "--trace-base",
        type=str,
        default=None,
        help="Override trace base filename (absolute or relative to bin)",
    )
    return parser.parse_args()


def build_cmd(config_path, seed, sim_cycles, trace_base):
    cmd = ["./noxim", "-config", str(config_path), "-seed", str(seed)]
    if sim_cycles is not None:
        cmd += ["-sim", str(sim_cycles)]
    if trace_base is not None:
        cmd += ["-traffic", "trace", str(trace_base)]
    return cmd


def run_one(name, config_path, seed, sim_cycles, trace_base, output_dir):
    log_path = output_dir / f"{name}.log"
    cmd = build_cmd(config_path, seed, sim_cycles, trace_base)
    with log_path.open("w") as log_file:
        proc = subprocess.run(
            cmd,
            cwd=BIN_DIR,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return name, proc.returncode, log_path


def main():
    args = parse_args()

    missing_configs = [
        str(path) for path in DEFAULT_CONFIGS.values() if not path.exists()
    ]
    if missing_configs:
        print("Missing config files:", file=sys.stderr)
        for path in missing_configs:
            print(f"  - {path}", file=sys.stderr)
        return 1

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    trace_base = args.trace_base
    if trace_base is None:
        trace_base = DEFAULT_TRACE_BASE
    else:
        trace_base = Path(trace_base)
        if not trace_base.is_absolute():
            trace_base = (BIN_DIR / trace_base).resolve()

    sample_trace = trace_base.parent / f"000_{trace_base.name}"
    if not sample_trace.exists():
        print(
            f"Warning: trace file not found: {sample_trace}",
            file=sys.stderr,
        )

    print(f"Running with seed={args.seed}, workers={args.workers}")
    print(f"Trace base: {trace_base}")
    print(f"Logs: {OUTPUT_DIR}")

    results = []
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = []
        for name, config_path in DEFAULT_CONFIGS.items():
            futures.append(
                executor.submit(
                    run_one,
                    name,
                    config_path,
                    args.seed,
                    args.sim_cycles,
                    trace_base,
                    OUTPUT_DIR,
                )
            )

        for future in as_completed(futures):
            results.append(future.result())

    print("\nSummary:")
    for name, returncode, log_path in sorted(results):
        status = "ok" if returncode == 0 else f"failed ({returncode})"
        print(f"- {name}: {status} -> {log_path}")

    return 0 if all(rc == 0 for _, rc, _ in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
