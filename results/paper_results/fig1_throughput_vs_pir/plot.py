#!/usr/bin/env python3
"""
Figure 1: Wireless Throughput vs. Packet Injection Rate (PIR)

Compares three MAC protocols:
- Token Passing (baseline)
- Fuzzy Token
- FP-Jump (FUZZY_SWJ)

Runs cycle-accurate NoC simulations and takes geometric mean across 10 seeds.
Results are cached to CSV for quick graph regeneration.
"""

import subprocess
import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import random
import time

# Configuration
SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent
BIN_DIR = PROJECT_ROOT / "bin"
NOXIM_BIN = BIN_DIR / "noxim"
POWER_CONFIG = BIN_DIR / "power.yaml"

# Base configs (relative to PROJECT_ROOT)
CONFIGS = {
    "Token Passing": "configs/custom_config/token_packet_soteriou_traffic.yaml",
    "Fuzzy Token": "configs/custom_config/fuzzy_token_soteriou_traffic.yaml",
    "FP-Jump": "configs/custom_config/fuzzy_swj_soteriou_traffic.yaml",
}

# Simulation parameters - tuned for better differentiation
PIR_LIST = [round(x, 4) for x in np.linspace(0.0001, 0.04, 15)]
NUM_SEEDS = 10

# Output files
CSV_OUTPUT = SCRIPT_DIR / "throughput_data.csv"


def parse_throughput(output: str) -> float:
    """Parse network throughput from noxim output."""
    match = re.search(r"% Network throughput \(flits/cycle\):\s*([0-9.]+)", output)
    if match:
        return float(match.group(1))
    return 0.0


def run_simulation(config_path: str, pir: float, seed: int) -> float:
    """Run a single noxim simulation and return throughput."""
    cmd = [
        str(NOXIM_BIN),
        "-config", str(config_path),
        "-power", str(POWER_CONFIG),
        "-pir", str(pir), "poisson",
        "-seed", str(seed),
    ]
    
    try:
        result = subprocess.run(
            cmd,
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=300
        )
        throughput = parse_throughput(result.stdout)
        return throughput
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 0.0


def calculate_gmean(values):
    """Calculate geometric mean of a list of values."""
    if not values or any(v <= 0 for v in values):
        return 0.0
    return np.exp(np.mean(np.log(values)))


def run_sweep(config_path: str, protocol_name: str):
    """Run PIR sweep for a single protocol."""
    print(f"Running sweep for {protocol_name}...")
    
    # Generate random seeds
    random.seed(42)
    seeds = random.sample(range(1, 10000), NUM_SEEDS)
    
    results = {"pir": [], "throughput": []}
    
    # Parallel execution across all PIR × seed combinations
    all_tasks = []
    for pir in PIR_LIST:
        for seed in seeds:
            all_tasks.append((pir, seed))
    
    # Use ThreadPoolExecutor for I/O-bound simulation launches
    max_workers = min(32, len(all_tasks))
    
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(run_simulation, config_path, pir, seed): (pir, seed) 
            for pir, seed in all_tasks
        }
        
        # Collect results
        results_by_pir = {pir: [] for pir in PIR_LIST}
        
        completed = 0
        for future in as_completed(futures):
            pir, seed = futures[future]
            throughput = future.result()
            if throughput > 0:
                results_by_pir[pir].append(throughput)
            completed += 1
            if completed % 10 == 0:
                print(f"  Progress: {completed}/{len(all_tasks)}")
    
    # Aggregate results
    for pir in PIR_LIST:
        gmean = calculate_gmean(results_by_pir[pir])
        results["pir"].append(pir)
        results["throughput"].append(gmean)
        print(f"  PIR={pir:.4f}: Throughput={gmean:.4f}")
    
    return results


def save_to_csv(all_results: dict):
    """Save results to CSV for caching."""
    rows = []
    for protocol, data in all_results.items():
        for pir, throughput in zip(data["pir"], data["throughput"]):
            rows.append({
                "protocol": protocol,
                "pir": pir,
                "throughput": throughput
            })
    
    df = pd.DataFrame(rows)
    df.to_csv(CSV_OUTPUT, index=False)
    print(f"Results cached to {CSV_OUTPUT}")


def load_from_csv() -> dict:
    """Load results from CSV if available."""
    if not CSV_OUTPUT.exists():
        return None
    
    df = pd.read_csv(CSV_OUTPUT)
    all_results = {}
    for protocol in df["protocol"].unique():
        protocol_data = df[df["protocol"] == protocol]
        all_results[protocol] = {
            "pir": protocol_data["pir"].tolist(),
            "throughput": protocol_data["throughput"].tolist()
        }
    print(f"Loaded cached results from {CSV_OUTPUT}")
    return all_results


def plot_results(all_results: dict):
    """Generate IEEE-quality throughput vs PIR plot."""
    
    # IEEE Journal standards
    plt.rcParams.update({
        'font.family': 'Times New Roman',
        'font.size': 9,
        'axes.titlesize': 10,
        'axes.labelsize': 9,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 8,
        'figure.figsize': (3.5, 2.5),
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'axes.linewidth': 0.5,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linewidth': 0.5,
    })
    
    fig, ax = plt.subplots()
    
    # Professional color scheme (colorblind-friendly)
    styles = {
        "Token Passing": {"color": "#0072B2", "marker": "o", "linestyle": "-", "markersize": 4},
        "Fuzzy Token": {"color": "#D55E00", "marker": "s", "linestyle": "--", "markersize": 4},
        "FP-Jump": {"color": "#009E73", "marker": "^", "linestyle": "-.", "markersize": 4},
    }
    
    for protocol, data in all_results.items():
        style = styles.get(protocol, {"color": "gray", "marker": "o", "linestyle": "-", "markersize": 4})
        ax.plot(
            data["pir"], 
            data["throughput"],
            marker=style["marker"],
            linestyle=style["linestyle"],
            color=style["color"],
            linewidth=1.0,
            markersize=style["markersize"],
            markeredgecolor=style["color"],
            markeredgewidth=0.5,
            label=protocol
        )
    
    # Axis labels
    ax.set_xlabel("Packet Injection Rate (flits/cycle/node)")
    ax.set_ylabel("Wireless Throughput (flits/cycle)")
    
    # Legend - outside plot, upper left
    ax.legend(loc='lower right', frameon=True, fancybox=False, edgecolor='black')
    
    # Grid
    ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.set_xlim(0, 0.042)
    ax.set_ylim(0, 0.55)
    
    # Add minor ticks
    ax.minorticks_on()
    ax.tick_params(which='minor', length=2)
    
    plt.tight_layout()
    
    # Save
    output_png = SCRIPT_DIR / "fig1_throughput.png"
    output_eps = SCRIPT_DIR / "fig1_throughput.eps"
    
    plt.savefig(output_png, format='png', bbox_inches='tight', pad_inches=0.05)
    plt.savefig(output_eps, format='eps', bbox_inches='tight', pad_inches=0.05)
    
    print(f"\nFigures saved to:")
    print(f"  - {output_png}")
    print(f"  - {output_eps}")
    
    plt.close()


def main():
    print("=" * 60)
    print("Figure 1: Throughput vs. PIR")
    print("=" * 60)
    print(f"PIR range: {PIR_LIST[0]} to {PIR_LIST[-1]} ({len(PIR_LIST)} points)")
    print(f"Seeds per point: {NUM_SEEDS}")
    print("=" * 60)
    
    # Check noxim binary
    if not NOXIM_BIN.exists():
        print(f"Error: noxim binary not found at {NOXIM_BIN}", file=sys.stderr)
        sys.exit(1)
    
    # Try loading from cache first
    force_resim = "--force" in sys.argv
    all_results = None if force_resim else load_from_csv()
    
    if all_results is None:
        print("\nRunning simulations...")
        start_time = time.time()
        
        # Run sweeps for all protocols
        all_results = {}
        for protocol, config_rel in CONFIGS.items():
            config_path = PROJECT_ROOT / config_rel
            if not config_path.exists():
                print(f"Warning: Config not found: {config_path}, skipping...")
                continue
            
            results = run_sweep(str(config_path), protocol)
            all_results[protocol] = results
        
        elapsed = time.time() - start_time
        print(f"\nSimulation completed in {elapsed/60:.1f} minutes")
        
        # Save to CSV
        save_to_csv(all_results)
    else:
        print("\nUsing cached results (use --force to re-simulate)")
    
    # Generate plot
    if all_results:
        plot_results(all_results)
    else:
        print("Error: No results collected", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
