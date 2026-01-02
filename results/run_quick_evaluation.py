#!/usr/bin/env python3
"""
Quick evaluation script for selected Noxim configurations.
This script runs evaluations on a subset of config files for faster testing.
"""

import subprocess
import pickle
import datetime
import os
import sys
from pathlib import Path
import matplotlib.pyplot as plt
from numpy import linspace

def parse_noxim_output(output):
    """Parse the Noxim simulation output and extract relevant metrics."""
    metrics = {}
    lines = output.splitlines()
    
    for line in lines:
        if "Total received packets" in line:
            metrics["total_received_packets"] = int(line.split(":")[-1].strip())
        elif "Total received flits" in line:
            metrics["total_received_flits"] = int(line.split(":")[-1].strip())
        elif "Global average delay (cycles)" in line:
            metrics["global_average_delay"] = float(line.split(":")[-1].strip())
        elif "Network throughput (flits/cycle)" in line:
            metrics["network_throughput"] = float(line.split(":")[-1].strip())
        elif "Total energy (J)" in line:
            metrics["total_energy"] = float(line.split(":")[-1].strip())
        elif "Dynamic energy (J)" in line:
            metrics["dynamic_energy"] = float(line.split(":")[-1].strip())
        elif "Static energy (J)" in line:
            metrics["static_energy"] = float(line.split(":")[-1].strip())
    
    return metrics

def save_to_pickle(data, filename):
    """Save the parsed data to a pickle file."""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'wb') as f:
        pickle.dump(data, f)
    print(f"Data saved to {filename}")

def run_quick_evaluation():
    """Run evaluation on selected configurations."""
    
    # Set up paths
    script_dir = Path(__file__).parent
    noxim_root = script_dir.parent
    bin_dir = noxim_root / 'bin'
    
    # Create results directory
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    results_base_dir = script_dir / f'quick_evaluation_{timestamp}'
    results_base_dir.mkdir(parents=True, exist_ok=True)
    
    # Selected configurations for quick testing
    selected_configs = [
        # 'fuzzy_token_8x8_da.yaml',
        # 'fuzzy_token_8x8_dca.yaml',
        # 'token_packet_8x8_da.yaml',
        # 'token_packet_8x8_dca.yaml',
        'token_packet_8x8.yaml',
        'fuzzy_token_8x8.yaml',
    ]
    
    print(f"Quick Noxim Evaluation")
    print(f"=====================")
    print(f"Selected configurations: {selected_configs}")
    print(f"Results directory: {results_base_dir}")
    
    # Check noxim binary
    noxim_binary = bin_dir / 'noxim'
    if not noxim_binary.exists():
        print(f"Error: Noxim binary not found at {noxim_binary}")
        return 1
    
    all_config_results = {}
    
    # Run evaluations
    for config_file in selected_configs:
        config_name = Path(config_file).stem
        print(f"\nEvaluating {config_name}...")
        
        config_results = {}
        injection_rates = linspace(0.0001, 0.005, 10) # Fewer points for speed
        injection_rates = [round(x, 4) for x in injection_rates]
        injection_rates.extend([0.006, 0.007, 0.008, 0.009, 0.01, 0.02])

        for pir in injection_rates:
            print(f"  PIR={pir}")
            try:
                result = subprocess.run(
                    ['./noxim', '-config', f'../custom_config/{config_file}', '-pir', str(pir), 'poisson', '-seed', '69', '-traffic', 'random'],
                    capture_output=True, text=True, cwd=str(bin_dir), timeout=120
                )
                
                if result.returncode == 0:
                    metrics = parse_noxim_output(result.stdout)
                    config_results[pir] = metrics
                    print(f"    ✓ Throughput: {metrics.get('network_throughput', 'N/A')}")
                else:
                    print(f"    ✗ Failed")
                    
            except Exception as e:
                print(f"    ✗ Error: {e}")
        
        if config_results:
            all_config_results[config_name] = config_results
            # Save individual config results
            config_dir = results_base_dir / config_name
            save_to_pickle(config_results, config_dir / f"{config_name}_results_{timestamp}.pkl")
    
    # Generate plots
    if all_config_results:
        create_quick_plots(all_config_results, results_base_dir)
        print(f"\nQuick evaluation completed! Results in: {results_base_dir}")
    else:
        print("No successful evaluations!")
        return 1
    
    return 0

def create_quick_plots(all_configs_data, results_dir):
    """Create comparison plots for the evaluated configurations."""
    
    colors = ['#2E86AB', '#F18F01', '#C73E1D', '#228B22', '#8B0000', '#4169E1', 
                  '#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD']
    markers = ['o', 's', '^', 'D', 'v', 'p', '*', 'h', '+', 'x', '8', 'P']
    
    # Create a figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    
    for i, (config_name, config_data) in enumerate(all_configs_data.items()):
        injection_rates = sorted(config_data.keys())
        throughput_values = [config_data[k]['network_throughput'] for k in injection_rates]
        delay_values = [config_data[k]['global_average_delay'] for k in injection_rates]
        energy_values = [config_data[k]['total_energy'] for k in injection_rates]
        
        color = colors[i % len(colors)]
        marker = markers[i % len(markers)]
        
        # Throughput plot
        ax1.plot(injection_rates, throughput_values, f'{marker}-', 
                linewidth=2, markersize=6, color=color, label=config_name)
        
        # Delay plot
        ax2.plot(injection_rates, delay_values, f'{marker}-', 
                linewidth=2, markersize=6, color=color, label=config_name)
        
        # Energy plot
        ax3.plot(injection_rates, energy_values, f'{marker}-', 
                linewidth=2, markersize=6, color=color, label=config_name)
    
    # Configure throughput plot
    ax1.set_xlabel('Injection Rate (flits/cycle/node)', fontweight='bold')
    ax1.set_ylabel('Network Throughput (flits/cycle)', fontweight='bold')
    ax1.set_title('Network Throughput', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Configure delay plot
    ax2.set_xlabel('Injection Rate (flits/cycle/node)', fontweight='bold')
    ax2.set_ylabel('Global Average Delay (cycles)', fontweight='bold')
    ax2.set_title('Average Delay', fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Configure energy plot
    ax3.set_xlabel('Injection Rate (flits/cycle/node)', fontweight='bold')
    ax3.set_ylabel('Total Energy Consumption (J)', fontweight='bold')
    ax3.set_title('Total Energy', fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig(results_dir / 'quick_comparison.png', dpi=300, bbox_inches='tight')
    plt.savefig(results_dir / 'quick_comparison.pdf', bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    sys.exit(run_quick_evaluation())
