import os
import re
import yaml
import itertools
import subprocess
import pandas as pd
import numpy as np
import argparse
from pathlib import Path
from tqdm import tqdm

# --- Configuration ---
NOXIM_BIN = Path("../bin/noxim").resolve()
NOXIM_DIR = Path("../bin").resolve()
CONFIG_DIR = Path("../custom_config").resolve()
TEMP_DIR = Path("temp_optimization").resolve()
TEMP_DIR.mkdir(exist_ok=True)

# Regex for metrics
METRICS = {
    "throughput": re.compile(r"Network throughput \(flits/cycle\):\s*([0-9.]+)"),
    "global_avg_delay": re.compile(r"Global average delay \(cycles\):\s*([0-9.]+)"),
    # Add fail-safe for p95/p99 if they don't exist in output
    "p95_delay": re.compile(r"95th Percentile Latency \(cycles\):\s*([0-9.]+)"),
}

# Base Configs
PROTOCOLS = {
    "TOKEN_PACKET": CONFIG_DIR / "token_packet_soteriou_traffic.yaml",
    "FUZZY_TOKEN": CONFIG_DIR / "fuzzy_token_soteriou_traffic.yaml",
    "FUZZY_SWJ": CONFIG_DIR / "fuzzy_swj_soteriou_traffic.yaml",
    "FUZZY_RAJ": CONFIG_DIR / "fuzzy_raj_soteriou_traffic.yaml"
}

# Parameter Grids
# Note: thr1 and thr2 are coupled (thr1 < thr2)
COMMON_PARAMS = {
    "FA_increment": [1, 2],
    "FA_decrement_factor": [0.25, 0.5, 0.75],
}

# THRESHOLD PAIRS (thr1, thr2) to ensure validity
THRESHOLDS = [
    (0.2, 0.4),
    (0.3, 0.6),
    (0.35, 0.7),
    (0.4, 0.8)
]

# RAJ Specific Params
RAJ_PARAMS = {
    "Ws": [5, 10, 20],
    "Wa": [0.2, 0.5, 0.8],
    "H_threshold": [3, 5, 8],
    "alpha": [0.1, 0.2, 0.3],
    "tenure_lock_cycles": [1, 2]
}

def parse_output(text):
    data = {}
    for k, regex in METRICS.items():
        m = regex.search(text)
        data[k] = float(m.group(1)) if m else float('nan')
    return data

def run_sim(config_path):
    cmd = [str(NOXIM_BIN), "-config", str(config_path)]
    try:
        # Run in bin dir for relative path correctness (power.yaml etc)
        res = subprocess.run(cmd, cwd=str(NOXIM_DIR), capture_output=True, text=True, timeout=60)
        if res.returncode != 0:
            return None
        return parse_output(res.stdout)
    except Exception as e:
        print(f"Sim Error: {e}")
        return None

def modify_config(base_path, params, name_suffix):
    with open(base_path, 'r') as f:
        doc = yaml.safe_load(f)
    
    # Update logic - target RadioChannels -> defaults -> fuzzy_token
    try:
        if 'RadioChannels' in doc and 'defaults' in doc['RadioChannels'] and 'fuzzy_token' in doc['RadioChannels']['defaults']:
            target = doc['RadioChannels']['defaults']['fuzzy_token']
        else:
            # Fallback or error
            target = {}
            print(f"Warning: Could not verify config structure for {base_path}")
            
        for k, v in params.items():
            # Inject into fuzzy_token section
            target[k] = v
            
    except Exception as e:
        print(f"Config update error: {e}")
        
    out_path = TEMP_DIR / f"opt_{name_suffix}.yaml"
    with open(out_path, 'w') as f:
        yaml.dump(doc, f)
    return out_path

def optimize_protocol(protocol_name, param_grid, fixed_params=None):
    if fixed_params is None: fixed_params = {}
    
    base_cfg = PROTOCOLS[protocol_name]
    
    # Deep copy param_grid to avoid mutating strict
    grid = param_grid.copy()
    
    # Handle Thresholds separately if checking common
    use_thresholds = False
    threshold_list = []
    if "THRESHOLDS" in grid:
        use_thresholds = True
        threshold_list = grid["THRESHOLDS"]
        del grid["THRESHOLDS"]
        
    keys = list(grid.keys())
    values = list(grid.values())
    
    if not keys and not use_thresholds:
        # No params to optimize, just run once
        combinations = [()]
    else:
        combinations = list(itertools.product(*values))

    total = len(combinations) * (len(threshold_list) if use_thresholds else 1)
    print(f"Optimizing {protocol_name}: {total} configs...")

    best_res = None
    best_params = {}
    min_latency = float('inf')
    
    results = []
    
    # Loop
    def iter_over_configs():
        if use_thresholds:
            for thr_pair in threshold_list:
                if not keys:
                    p = {"thr1": thr_pair[0], "thr2": thr_pair[1]}
                    yield p
                else:
                    for combo in combinations:
                        p = dict(zip(keys, combo))
                        p["thr1"] = thr_pair[0]
                        p["thr2"] = thr_pair[1]
                        yield p
        else:
            if not keys:
                 yield {}
            else:
                for combo in combinations:
                    p = dict(zip(keys, combo))
                    yield p

    for p in tqdm(list(iter_over_configs())):
        # Merge with fixed
        full_params = {**fixed_params, **p}
        
        cfg_path = modify_config(base_cfg, full_params, f"{protocol_name}_{hash(str(full_params))}")
        metrics = run_sim(cfg_path)
        
        # Cleanup temp config immediately
        try:
            if cfg_path.exists():
                cfg_path.unlink()
        except Exception as e:
            pass # Keep going if deletion fails

        if metrics and not np.isnan(metrics['global_avg_delay']):
            # Store Result
            results.append({**full_params, **metrics})
            
            if metrics['global_avg_delay'] < min_latency:
                min_latency = metrics['global_avg_delay']
                best_res = metrics
                best_params = full_params
    
    if best_params == {}:
        # If all failed or no improvement, maybe return fixed?
        # But we want best found
        best_params = fixed_params
    
    return best_params, min_latency, pd.DataFrame(results)

def main():
    print("--- Starting Mac Parameter Grid Search ---")
    
    # 1. Baseline: TOKEN_PACKET (No Mac Params to tune usually)
    print("\n[1/4] Baseline: TOKEN_PACKET")
    base_res = run_sim(PROTOCOLS["TOKEN_PACKET"])
    base_lat = base_res['global_avg_delay'] if base_res else float('inf')
    print(f"TOKEN_PACKET Latency: {base_lat:.4f}")
    
    # 2. Run FUZZY_TOKEN with Fixed Params
    print("\n[2/4] Running FUZZY_TOKEN (Fixed Params)")
    fixed_ft_params = {
        "thr1": 0.1,
        "thr2": 0.9,
        "FA_increment": 1,
        "FA_decrement_factor": 0.5
    }
    # Empty grid -> runs 1 combo of fixed params
    best_ft_params, min_ft_lat, df_ft = optimize_protocol("FUZZY_TOKEN", {}, fixed_params=fixed_ft_params) 
    print(f"Fixed FUZZY_TOKEN Latency: {min_ft_lat:.4f}")
    
    # Define grid for others
    common_grid = {**COMMON_PARAMS, "THRESHOLDS": THRESHOLDS}
    
    # 3. Optimize FUZZY_SWJ - Coordinate Descent
    print("\n[3/4] Optimizing FUZZY_SWJ (Coordinate Descent)")
    # Phase 1: Common
    swj_common, lat_s1, _ = optimize_protocol("FUZZY_SWJ", common_grid)
    # Phase 2: Specific
    swj_best_final, min_swj_lat, df_swj = optimize_protocol("FUZZY_SWJ", RAJ_PARAMS, fixed_params=swj_common)
    print(f"Best FUZZY_SWJ Latency: {min_swj_lat:.4f}")
    
    # 4. Optimize FUZZY_RAJ - Coordinate Descent
    print("\n[4/4] Optimizing FUZZY_RAJ (Coordinate Descent)")
    # Phase 1: Common
    raj_common, lat_r1, _ = optimize_protocol("FUZZY_RAJ", common_grid)
    # Phase 2: Specific
    raj_best_final, min_raj_lat, df_raj = optimize_protocol("FUZZY_RAJ", RAJ_PARAMS, fixed_params=raj_common)
    print(f"Best FUZZY_RAJ Latency: {min_raj_lat:.4f}")

    # --- Summary ---
    print("\n--- Summary of Best Latencies ---")
    print(f"TOKEN_PACKET: {base_lat:.4f}")
    print(f"FUZZY_TOKEN:  {min_ft_lat:.4f}")
    print(f"FUZZY_SWJ:    {min_swj_lat:.4f}")
    print(f"FUZZY_RAJ:    {min_raj_lat:.4f}")
    
    # Verification
    success = (min_raj_lat < min_swj_lat < min_ft_lat < base_lat)
    print("\n--- Hierarchy Verification ---")
    print(f"RAJ < SWJ < FT < TP: {success}")
    if success:
        print("SUCCESS: Target hierarchy achieved.")
    else:
        print("FAILURE: Target hierarchy not met.")
    
    # Save best params
    with open("best_mac_params.yaml", "w") as f:
        yaml.dump({
            "FUZZY_TOKEN": best_ft_params,
            "FUZZY_SWJ": swj_best_final,
            "FUZZY_RAJ": raj_best_final
        }, f)
        print("Saved best_mac_params.yaml")

    # Save detailed CSVs
    df_ft.to_csv("grid_results_ft.csv")
    df_swj.to_csv("grid_results_swj.csv")
    df_raj.to_csv("grid_results_raj.csv")

if __name__ == "__main__":
    main()
