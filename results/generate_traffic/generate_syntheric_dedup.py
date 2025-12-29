import math

# Configuration
DIM_X, DIM_Y = 8, 8
NUM_NODES = DIM_X * DIM_Y
FILENAME = "bin/traffic_tables/dedup_hotspot_table.txt"

# dedup role designations
MANAGER = 0
COMPRESSORS = [27, 28, 35, 36]  # Central high-activity nodes
SPATIAL_ALPHA = 1.2             # Spatial decay factor

# Traffic Behavior
PIR_BASE_HOTSPOT = 0.05
PIR_BASE_MANAGER = 0.02
PIR_BASE_WORKER = 0.005

def get_manhattan_dist(s, d):
    return abs((s % DIM_X) - (d % DIM_X)) + abs((s // DIM_X) - (d // DIM_X))

with open(FILENAME, "w") as f:
    f.write("# src\tdest\tpir\n")
    
    for src in range(NUM_NODES):
        # Determine base PIR by role
        if src == MANAGER:
            base_pir = PIR_BASE_MANAGER
        elif src in COMPRESSORS:
            base_pir = PIR_BASE_HOTSPOT
        else:
            base_pir = PIR_BASE_WORKER
            
        for dst in range(NUM_NODES):
            if src == dst: continue
            
            dist = get_manhattan_dist(src, dst)
            
            # Column 3: PIR (Probability of Injection)
            # Decays with distance to represent Spatiotemporal Locality
            pir = base_pir * (1.0 / math.pow(dist, SPATIAL_ALPHA))
            
            f.write(f"{src}\t{dst}\t{pir:.6f}\n")

print(f"Publication-ready traffic table generated: {FILENAME}")