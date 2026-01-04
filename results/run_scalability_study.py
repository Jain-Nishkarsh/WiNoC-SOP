import subprocess
import re
import matplotlib.pyplot as plt
import yaml

# Configuration
SIZES = [4, 8, 16, 32]
PROTOCOLS = ["TOKEN_PACKET", "FUZZY_TOKEN", "FUZZY_SWJ", "FUZZY_RAJ"]
BASE_CONFIG_PATH = "../custom_config/fuzzy_swj_soteriou_traffic.yaml"
TEMP_CONFIG_PATH = "../bin/temp_scalability.yaml"
BIN_DIR = "../bin"
NOXIM_BIN = "./noxim"
POWER_CONFIG = "power.yaml"

# Results storage
results = {p: [] for p in PROTOCOLS}

def load_base_config():
    with open(BASE_CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)

def generate_hubs(size):
    hubs = {"defaults": {
        "rx_radio_channels": [0],
        "tx_radio_channels": [0],
        "attached_nodes": [],
        "to_tile_buffer_size": 32,
        "from_tile_buffer_size": 32,
        "rx_buffer_size": 64,
        "tx_buffer_size": 64
    }}
    num_nodes = size * size
    for i in range(num_nodes):
        hubs[i] = {"attached_nodes": [i]}
    return hubs

def run_simulation(size, protocol):
    config = load_base_config()
    
    # Update Topology
    config['mesh_dim_x'] = size
    config['mesh_dim_y'] = size
    
    # Update Hubs
    config['Hubs'] = generate_hubs(size)
    
    # Update Protocol
    config['RadioChannels']['defaults']['mac_policy'] = [protocol]
    
    # Specific tweaks
    if protocol == "FUZZY_RAJ":
        config['RadioChannels']['defaults']['fuzzy_token']['control_minislot_delay'] = 4
    
    # Update Hotspot (Center-ish)
    center = size // 2 - 1
    config['traffic_soteriou_hotspot'] = [center, center]
    
    # Adjust simulation time for larger networks to save time
    if size >= 32:
        config['simulation_time'] = 1000
    
    # Ensure CSV logging is off to save time/space or use a temp file
    config['csv_log_enabled'] = False
    
    # Write temp config
    with open(TEMP_CONFIG_PATH, 'w') as f:
        yaml.dump(config, f)
        
    # Run Noxim
    cmd = [NOXIM_BIN, "-power", POWER_CONFIG, "-config", "temp_scalability.yaml"]
    print(f"Running {protocol} on {size}x{size}...", end="", flush=True)
    
    try:
        result = subprocess.run(
            cmd,
            cwd=BIN_DIR,
            capture_output=True,
            text=True,
            check=True,
            timeout=300 # 5 minutes timeout
        )
        
        # Parse Global Average Delay
        match = re.search(r"Global average delay \(cycles\)\s*:\s*([\d\.]+)", result.stdout)
        if match:
            delay = float(match.group(1))
            print(f" Delay: {delay}")
            return delay
        else:
            print(" Delay not found!")
            return None
            
    except subprocess.CalledProcessError as e:
        print(f" Error: {e}")
        print(e.stdout)
        print(e.stderr)
        return None

# Main Loop
print("Starting Scalability Study...")
for size in SIZES:
    for protocol in PROTOCOLS:
        delay = run_simulation(size, protocol)
        if delay is not None:
            results[protocol].append(delay)
        else:
            results[protocol].append(0) # Or NaN

# Plotting
print("\nPlotting results...")
plt.figure(figsize=(10, 6))

bar_width = 0.2
index = range(len(SIZES))
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

for i, protocol in enumerate(PROTOCOLS):
    plt.bar(
        [x + i * bar_width for x in index], 
        results[protocol], 
        bar_width, 
        label=protocol,
        color=colors[i]
    )

plt.xlabel('Network Size (Nodes)', fontsize=12)
plt.ylabel('Average Discovery Latency (Cycles)', fontsize=12)
plt.title('Discovery Latency vs. Network Size', fontsize=14)
plt.xticks([x + 1.5 * bar_width for x in index], [f"{s}x{s} ({s*s})" for s in SIZES])
plt.legend()
plt.grid(True, axis='y', alpha=0.3)
plt.tight_layout()

# Save plot
plt.savefig('scalability_study.png')
print("Plot saved to results/scalability_study.png")

# Print Data for verification
print("\nResults Data:")
print(f"{'Size':<10} | {'TOKEN_PACKET':<15} | {'FUZZY_TOKEN':<15} | {'FUZZY_SWJ':<15} | {'FUZZY_RAJ':<15}")
print("-" * 80)
for i, size in enumerate(SIZES):
    print(f"{size}x{size:<6} | {results['TOKEN_PACKET'][i]:<15.2f} | {results['FUZZY_TOKEN'][i]:<15.2f} | {results['FUZZY_SWJ'][i]:<15.2f} | {results['FUZZY_RAJ'][i]:<15.2f}")
