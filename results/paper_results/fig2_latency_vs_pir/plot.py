#!/usr/bin/env python3
"""
Figure 2: Average Packet Latency vs. Packet Injection Rate (PIR)

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

# Simulation parameters
PIR_LIST = [round(x, 4) for x in np.linspace(0.0001, 0.04, 15)]
NUM_SEEDS = 10

# Output files
CSV_OUTPUT = SCRIPT_DIR / "latency_data.csv"


def parse_latency(output: str) -> float:
    """Parse average latency from noxim output."""
    match = re.search(r"% Global average delay \(cycles\):\s*([0-9.]+)", output)
    if match:
        return float(match.group(1))
    return 0.0


def run_simulation(config_path: str, pir: float, seed: int) -> float:
    """Run a single noxim simulation and return latency."""
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
        latency = parse_latency(result.stdout)
        return latency
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
    
    # Parallel execution across all PIR × seed combinations
    all_tasks = []
    for pir in PIR_LIST:
        for seed in seeds:
            all_tasks.append((pir, seed))
    
    max_workers = min(32, len(all_tasks))
    
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(run_simulation, config_path, pir, seed): (pir, seed) 
            for pir, seed in all_tasks
        }
        
        results_by_pir = {pir: [] for pir in PIR_LIST}
        
        completed = 0
        for future in as_completed(futures):
            pir, seed = futures[future]
            latency = future.result()
            if latency > 0:
                results_by_pir[pir].append(latency)
            completed += 1
            if completed % 50 == 0:
                print(f"  Progress: {completed}/{len(all_tasks)}")
    
    # Aggregate results
    results = {"pir": [], "latency": []}
    for pir in PIR_LIST:
        gmean = calculate_gmean(results_by_pir[pir])
        results["pir"].append(pir)
        results["latency"].append(gmean)
        print(f"  PIR={pir:.4f}: Latency={gmean:.2f}")
    
    return results


def save_to_csv(all_results: dict):
    """Save results to CSV for caching."""
    rows = []
    for protocol, data in all_results.items():
        for pir, latency in zip(data["pir"], data["latency"]):
            rows.append({
                "protocol": protocol,
                "pir": pir,
                "latency": latency
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
            "latency": protocol_data["latency"].tolist()
        }
    print(f"Loaded cached results from {CSV_OUTPUT}")
    return all_results


def plot_results(all_results: dict):
    """Generate IEEE-quality latency vs PIR plot."""
    
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
    
    styles = {
        "Token Passing": {"color": "#0072B2", "marker": "o", "linestyle": "-", "markersize": 4},
        "Fuzzy Token": {"color": "#D55E00", "marker": "s", "linestyle": "--", "markersize": 4},
        "FP-Jump": {"color": "#009E73", "marker": "^", "linestyle": "-.", "markersize": 4},
    }
    
    for protocol, data in all_results.items():
        style = styles.get(protocol, {"color": "gray", "marker": "o", "linestyle": "-", "markersize": 4})
        ax.plot(
            data["pir"], 
            data["latency"],
            marker=style["marker"],
            linestyle=style["linestyle"],
            color=style["color"],
            linewidth=1.0,
            markersize=style["markersize"],
            markeredgecolor=style["color"],
            markeredgewidth=0.5,
            label=protocol
        )
    
    ax.set_xlabel("Packet Injection Rate (flits/cycle/node)")
    ax.set_ylabel("Average Packet Latency (cycles)")
    
    ax.legend(loc='upper left', frameon=True, fancybox=False, edgecolor='black')
    
    ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.set_xlim(0, 0.042)
    
    ax.minorticks_on()
    ax.tick_params(which='minor', length=2)
    
    plt.tight_layout()
    
    output_png = SCRIPT_DIR / "fig2_latency.png"
    output_eps = SCRIPT_DIR / "fig2_latency.eps"
    
    plt.savefig(output_png, format='png', bbox_inches='tight', pad_inches=0.05)
    plt.savefig(output_eps, format='eps', bbox_inches='tight', pad_inches=0.05)
    
    print(f"\nFigures saved to:")
    print(f"  - {output_png}")
    print(f"  - {output_eps}")
    
    plt.close()


def main():
    print("=" * 60)
    print("Figure 2: Latency vs. PIR")
    print("=" * 60)
    print(f"PIR range: {PIR_LIST[0]} to {PIR_LIST[-1]} ({len(PIR_LIST)} points)")
    print(f"Seeds per point: {NUM_SEEDS}")
    print("=" * 60)
    
    if not NOXIM_BIN.exists():
        print(f"Error: noxim binary not found at {NOXIM_BIN}", file=sys.stderr)
        sys.exit(1)
    
    force_resim = "--force" in sys.argv
    all_results = None if force_resim else load_from_csv()
    
    if all_results is None:
        print("\nRunning simulations...")
        start_time = time.time()
        
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
        
        save_to_csv(all_results)
    else:
        print("\nUsing cached results (use --force to re-simulate)")
    
    if all_results:
        plot_results(all_results)
    else:
        print("Error: No results collected", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
