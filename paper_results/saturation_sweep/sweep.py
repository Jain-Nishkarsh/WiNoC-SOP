import subprocess
import re
import matplotlib.pyplot as plt
from pathlib import Path
import random

# --- CONFIGURATION ---
SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_ROOT = SCRIPT_DIR.parent.parent
BIN_DIR = (PROJECT_ROOT / "bin").resolve()
SIM_TIME = 20000
MESH_DIM = 8
HOTSPOT_NODE = 27
# Scattered AI Cluster (Diagonal nodes) to favor "Jumping" over sequential token passing
AI_CLUSTER = [0, 9, 18, 27, 36, 45, 54, 63]

SEED = 42
random.seed(SEED)

# Update these paths to where your YAML files are actually located
PROTOCOLS = {
    "TOKEN_PACKET": (PROJECT_ROOT / "custom_config/token_packet_8x8_traffic_table.yaml").resolve(),
    "FUZZY_TOKEN": (PROJECT_ROOT / "custom_config/fuzzy_token_8x8_traffic_table.yaml").resolve(),
    "FUZZY_SWJ": (PROJECT_ROOT / "custom_config/fuzzy_swj_8x8_traffic_table.yaml").resolve()
}

# Load Sweep (Scaling PIR)
LOAD_FACTORS = [0.2, 0.4, 0.6, 0.8, 1.0, 1.1, 1.2]

def generate_burst_times(duty_cycle=0.3):
    period = random.randint(3000, 6000) # Longer periods
    on_duration = int(period * duty_cycle)
    max_start = period - on_duration - 100
    t_on = random.randint(0, max(0, max_start))
    return t_on, t_on + on_duration, period

def create_traffic_table(factor, filename="temp_traffic.txt"):
    h_pir = 0.12 * factor # Very High Hotspot intensity
    c_pir = 0.06 * factor # High Cluster intensity
    b_pir = 0.0001 * factor # Minimal Background noise
    
    with open(filename, "w") as f:
        f.write("# Source Dest PIR POR T_on T_off T_period\n")
        for _ in range(8):
            dest = random.choice([n for n in range(64) if n != HOTSPOT_NODE])
            t_on, t_off, t_p = generate_burst_times(0.5)
            f.write(f"{HOTSPOT_NODE} {dest} {h_pir} {h_pir} {t_on} {t_off} {t_p}\n")
        for src in AI_CLUSTER:
            dest = random.choice([n for n in range(64) if n in AI_CLUSTER and n != src])
            t_on, t_off, t_p = generate_burst_times(0.3)
            f.write(f"{src} {dest} {c_pir} {c_pir} {t_on} {t_off} {t_p}\n")
        for src in range(64):
            dest = (src + 1) % 64
            f.write(f"{src} {dest} {b_pir} {b_pir} 0 {SIM_TIME-1} {SIM_TIME}\n")

def run_simulation(config_path, traffic_table):
    # Using -config and -traffic_table
    # Use absolute path for traffic table to avoid CWD issues
    traffic_table_abs = (Path(traffic_table).resolve())
    cmd = f"./noxim -config {config_path} -traffic table {traffic_table_abs} -seed {SEED}"
    try:
        result = subprocess.run(cmd, cwd=str(BIN_DIR), shell=True, capture_output=True, text=True)
        output = result.stdout
        
        # Improved RegEx: handles '%', whitespace, and 'inf'
        def get_val(pattern, text):
            match = re.search(pattern, text)
            if match:
                val = match.group(1).lower()
                return float('inf') if val == 'inf' else float(val)
            return None

        lat = get_val(r"Global average delay \(cycles\):\s*([\d.infINF]+)", output)
        thr = get_val(r"Network throughput \(flits/cycle\):\s*([\d.infINF]+)", output)
        max_d = get_val(r"Max delay \(cycles\):\s*([\d.infINF]+)", output)

        if lat is None:
            print(f"\n[!] Error: Could not parse Noxim output for {config_path}")
            print("Raw Output Snippet:\n", output[-500:]) # Print last 500 chars
            return None, None, None

        return lat, thr, max_d
    except Exception as e:
        print(f"Subprocess Error: {e}")
        return None, None, None

# --- EXECUTION ---
results = {p: {"x": [], "y": [], "max": []} for p in PROTOCOLS}

for factor in LOAD_FACTORS:
    print(f"\n--- Testing Load Factor: {factor} ---")
    create_traffic_table(factor)
    for name, config in PROTOCOLS.items():
        # if not os.path.exists(config):
        #     print(f"Skipping {name}: Config file not found at {config}")
        #     continue
            
        lat, thr, m_lat = run_simulation(config, "temp_traffic.txt")
        if lat is not None:
            print(f"  {name}: Latency={lat:.2f}, Throughput={thr:.4f}")
            results[name]["x"].append(thr)
            results[name]["y"].append(lat)
            results[name]["max"].append(m_lat)

# --- PLOTTING ---
plt.figure(figsize=(10, 6))
for name, data in results.items():
    if data["x"]:
        # Filter out 'inf' for plotting purposes or clip them
        plt.plot(data["x"], [min(v, 20000) for v in data["y"]], marker='o', label=name)

plt.xlabel("Throughput (flits/cycle)")
plt.ylabel("Average Latency (cycles)")
plt.title("Saturation Throughput Sweep (8x8 Mesh)")
plt.legend()
plt.grid(True, ls="--")
plt.savefig("saturation_curve.png")
print("\nSaturation curve saved to saturation_curve.png")