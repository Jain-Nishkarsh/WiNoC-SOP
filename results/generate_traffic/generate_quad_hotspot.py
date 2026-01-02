# Configuration
TOTAL_NODES = 64
HOT_NODES = [0, 7, 56, 63]
HOT_PIR = 0.1      # High intensity
COLD_PIR = 0.005    # Background noise
POR = 1.0           # Packet-to-message ratio (usually 1.0)

# Timing (Constant traffic for the whole simulation)
T_ON = 0
T_OFF = 1000000
T_PERIOD = 1000001

filename = "bin/traffic_tables/quad_hotspot.txt"

with open(filename, "w") as f:
    f.write(f"# src dest pir por t_on t_off t_period\n")
    
    # 1. Generate Hot-to-Hot flows (The "Core" communication)
    for src in HOT_NODES:
        for dst in HOT_NODES:
            if src != dst:
                f.write(f"{src} {dst} {HOT_PIR} {POR} {T_ON} {T_OFF} {T_PERIOD}\n")
    
    # 2. Generate Cold background flows (To ensure fairness is tested)
    for src in range(TOTAL_NODES):
        if src not in HOT_NODES:
            # Each cold node sends to one random neighbor to create sparse traffic
            dst = (src + 1) % TOTAL_NODES
            f.write(f"{src} {dst} {COLD_PIR} {POR} {T_ON} {T_OFF} {T_PERIOD}\n")

print(f"Noxim-compatible traffic table generated: {filename}")