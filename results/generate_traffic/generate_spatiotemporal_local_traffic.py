import random

# --- CONFIGURATION (8x8 Mesh) ---
MESH_X, MESH_Y = 8, 8
TOTAL_NODES = MESH_X * MESH_Y
SIM_DURATION = 20000

# --- PIR VALUES (Adjusted for 64-node universal activity) ---
BASE_PIR = 0.0008       # Low-level noise on EVERY node
CLUSTER_PIR = 0.012     # Local AI traffic
HOTSPOT_PIR = 0.05      # High pressure source (Node 27)

# --- SPATIAL DEFINITIONS ---
HOTSPOT_SRC = 27       
AI_CLUSTER_NODES = [y * MESH_X + x for y in range(4, 8) for x in range(4, 8)]

def generate_valid_bursty_times(duty_cycle=0.3):
    t_period = random.randint(2000, 5000)
    on_duration = int(t_period * duty_cycle)
    max_start = t_period - on_duration - 100
    t_on = random.randint(0, max(0, max_start))
    t_off = t_on + on_duration
    return t_on, t_off, t_period

def create_universal_traffic_table(filename="bin/traffic_tables/spatiotemporal.txt"):
    with open(filename, "w") as f:
        f.write("# Source Dest PIR POR T_on T_off T_period\n")

        # 1. TASK A: Source-Hotspot (GPGPU Master)
        for _ in range(8):
            dest = random.choice([n for n in range(64) if n != HOTSPOT_SRC])
            t_on, t_off, t_period = generate_valid_bursty_times(duty_cycle=0.5)
            f.write(f"{HOTSPOT_SRC} {dest} {HOTSPOT_PIR} {HOTSPOT_PIR} {t_on} {t_off} {t_period}\n")

        # 2. TASK B: AI Cluster (Local Communication)
        for src in AI_CLUSTER_NODES:
            dest = random.choice([n for n in range(64) if n in AI_CLUSTER_NODES and n != src])
            t_on, t_off, t_period = generate_valid_bursty_times(duty_cycle=0.3)
            f.write(f"{src} {dest} {CLUSTER_PIR} {CLUSTER_PIR} {t_on} {t_off} {t_period}\n")

        # 3. TASK C: Universal Background Noise (The "Honesty" Traffic)
        # Every node sends to a random destination once per simulation
        for src in range(TOTAL_NODES):
            dest = (src + random.randint(1, 63)) % 64
            # Background traffic is steady/constant
            f.write(f"{src} {dest} {BASE_PIR} {BASE_PIR} 0 {SIM_DURATION-1} {SIM_DURATION}\n")

    print(f"Universal Traffic Table Generated: {filename}")

if __name__ == "__main__":
    create_universal_traffic_table()