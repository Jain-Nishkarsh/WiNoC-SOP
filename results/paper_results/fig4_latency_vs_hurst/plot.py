#!/usr/bin/env python3
"""
Figure 4: Average Latency vs. Hurst Exponent (H)

Compares three MAC protocols:
- Token Passing (baseline)
- Fuzzy Token
- FP-Jump (FUZZY_SWJ)

Fixed parameters: σ=3.0, PIR=0.025
Sweeps H: [0.5, 0.6, 0.7, 0.8, 0.9]

Uses temp YAML configs for hurst parameter.
Results are cached to CSV.
"""

import subprocess
import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import yaml
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import random
import time
import tempfile

# Configuration
SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_ROOT = SCRIPT_DIR.parent.parent.parent
BIN_DIR = PROJECT_ROOT / "bin"
NOXIM_BIN = BIN_DIR / "noxim"
POWER_CONFIG = BIN_DIR / "power.yaml"

# Base configs (relative to PROJECT_ROOT)
BASE_CONFIGS = {
    "Token Passing": "configs/custom_config/token_packet_soteriou_traffic.yaml",
    "Fuzzy Token": "configs/custom_config/fuzzy_token_soteriou_traffic.yaml",
    "FP-Jump": "configs/custom_config/fuzzy_swj_soteriou_traffic.yaml",
}

# Simulation parameters - as specified in paper
HURST_LIST = [0.5, 0.6, 0.7, 0.8, 0.9]
FIXED_SIGMA = 3.0
FIXED_PIR = 0.025
NUM_SEEDS = 10

# Output files
CSV_OUTPUT = SCRIPT_DIR / "hurst_sweep_data.csv"


def load_base_config(config_path: str) -> dict:
    """Load base YAML configuration."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def create_temp_config(base_config: dict, hurst: float, protocol_name: str) -> str:
    """Create temp YAML config with modified hurst."""
    config = base_config.copy()
    config['traffic_soteriou_hurst'] = hurst
    
    # Write to temp file
    fd, temp_path = tempfile.mkstemp(suffix='.yaml', prefix=f'noxim_{protocol_name}_h{str(hurst).replace(".", "p")}')
    os.close(fd)
    with open(temp_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    return temp_path


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


def run_sweep(base_config_path: str, protocol_name: str):
    """Run hurst sweep for a single protocol."""
    print(f"Running sweep for {protocol_name}...")
    
    # Load base config
    base_config = load_base_config(base_config_path)
    
    # Generate random seeds
    random.seed(42)
    seeds = random.sample(range(1, 10000), NUM_SEEDS)
    
    results = {"hurst": [], "latency": []}
    
    for hurst in HURST_LIST:
        # Create temp config with this hurst
        temp_config_path = create_temp_config(base_config, hurst, protocol_name.replace(" ", "_"))
        
        latencies = []
        
        # Run simulations in parallel
        max_workers = min(16, NUM_SEEDS)
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                executor.submit(run_simulation, temp_config_path, FIXED_PIR, seed): seed 
                for seed in seeds
            }
            
            for future in as_completed(futures):
                latency = future.result()
                if latency > 0:
                    latencies.append(latency)
        
        # Clean up temp file
        os.unlink(temp_config_path)
        
        # Calculate geometric mean
        gmean_latency = calculate_gmean(latencies)
        
        results["hurst"].append(hurst)
        results["latency"].append(gmean_latency)
        print(f"  H={hurst:.1f}: Latency={gmean_latency:.2f}")
    
    return results


def save_to_csv(all_results: dict):
    """Save results to CSV for caching."""
    rows = []
    for protocol, data in all_results.items():
        for hurst, latency in zip(data["hurst"], data["latency"]):
            rows.append({
                "protocol": protocol,
                "hurst": hurst,
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
            "hurst": protocol_data["hurst"].tolist(),
            "latency": protocol_data["latency"].tolist()
        }
    print(f"Loaded cached results from {CSV_OUTPUT}")
    return all_results


def plot_results(all_results: dict):
    """Generate IEEE-quality latency vs hurst plot."""
    
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
            data["hurst"], 
            data["latency"],
            marker=style["marker"],
            linestyle=style["linestyle"],
            color=style["color"],
            linewidth=1.5,
            markersize=style["markersize"],
            markeredgecolor=style["color"],
            markeredgewidth=0.5,
            label=protocol
        )
    
    ax.set_xlabel(r"Hurst Exponent $H$")
    ax.set_ylabel("Average Packet Latency (cycles)")
    
    ax.legend(loc='upper right', frameon=True, fancybox=False, edgecolor='black')
    
    ax.grid(True, which='major', linestyle='-', linewidth=0.5, alpha=0.3)
    ax.set_xlim(0.45, 0.95)
    
    ax.minorticks_on()
    ax.tick_params(which='minor', length=2)
    
    plt.tight_layout()
    
    output_png = SCRIPT_DIR / "fig4_hurst.png"
    output_eps = SCRIPT_DIR / "fig4_hurst.eps"
    
    plt.savefig(output_png, format='png', bbox_inches='tight', pad_inches=0.05)
    plt.savefig(output_eps, format='eps', bbox_inches='tight', pad_inches=0.05)
    
    print(f"\nFigures saved to:")
    print(f"  - {output_png}")
    print(f"  - {output_eps}")
    
    plt.close()


def main():
    print("=" * 60)
    print("Figure 4: Latency vs. Hurst Exponent (H)")
    print("=" * 60)
    print(f"H range: {HURST_LIST}")
    print(f"Fixed: σ={FIXED_SIGMA}, PIR={FIXED_PIR}")
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
        for protocol, config_rel in BASE_CONFIGS.items():
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
