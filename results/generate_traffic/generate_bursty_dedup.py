import math

# Configuration
DIM_X, DIM_Y = 8, 8
NUM_NODES = DIM_X * DIM_Y
FILENAME = "bin/traffic_tables/bursty_dedup_table.txt"

# Roles
MANAGER = 0
HOTSPOTS = [27, 28, 35, 36]  # Central Compressors
WORKERS = [i for i in range(NUM_NODES) if i not in HOTSPOTS and i != MANAGER]

# Timing parameters
SIM_DURATION = 20000
BURST_PERIOD = 4000  # Repeat the pipeline every 4000 cycles

def write_burst(f, src, dst, pir, start, end):
    """
    Writes a burst entry. 
    Format: src dest pir por t_on t_off t_period
    """
    # por (Prob of retransmission) is 0
    # t_period is set to BURST_PERIOD to make the burst repeat
    f.write(f"{src}\t{dst}\t{pir:.4f}\t0.0\t{start}\t{end}\t{BURST_PERIOD}\n")

with open(FILENAME, "w") as f:
    f.write("# src\tdest\tpir\tpor\tt_on\tt_off\tt_period\n")

    # 1. MANAGER -> WORKERS (Distribution Phase)
    # Starts at the beginning of the period
    for worker in WORKERS:
        write_burst(f, MANAGER, worker, 0.05, 0, 500)

    # 2. WORKERS -> HOTSPOTS (Hashing/Processing Phase)
    # Starts after manager finishes distribution
    for worker in WORKERS:
        # Each worker sends to one of the compressors (spatial preference)
        target_hotspot = HOTSPOTS[worker % len(HOTSPOTS)]
        write_burst(f, worker, target_hotspot, 0.08, 600, 1800)

    # 3. HOTSPOTS -> MANAGER (Compression/Writeback Phase)
    # The heaviest burst, happening at the end of the pipeline
    for hs in HOTSPOTS:
        write_burst(f, hs, MANAGER, 0.25, 2000, 3500)

    # 4. BACKGROUND NOISE (Worker-to-Worker)
    # Very low probability, throughout the duration
    for src in WORKERS:
        dst = (src + 1) % NUM_NODES
        if dst != src and dst not in HOTSPOTS and dst != MANAGER:
            write_burst(f, src, dst, 0.001, 0, SIM_DURATION)

print(f"Successfully generated {FILENAME}")