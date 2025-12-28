import random

# Configuration
NUM_NODES = 64
MESH_DIM = 8
SIM_DURATION = 100001
PHASE_DURATION = 25000  # Shift hotspot every 25k cycles
HIGH_PIR = 0.15         # High Packet Injection Rate for hotspot
LOW_PIR = 0.001         # Background noise for other nodes

def get_quadrant_nodes(quadrant):
    """Returns node IDs for a specific 4x4 quadrant in an 8x8 mesh."""
    nodes = []
    # Q0: Top-Left, Q1: Top-Right, Q2: Bottom-Left, Q3: Bottom-Right
    x_offsets = [0, 4, 0, 4]
    y_offsets = [0, 0, 4, 4]
    
    start_x = x_offsets[quadrant]
    start_y = y_offsets[quadrant]
    
    for y in range(start_y, start_y + 4):
        for x in range(start_x, start_x + 4):
            nodes.append(y * MESH_DIM + x)
    return nodes

def generate_traffic():
    with open("bin/traffic_tables/shifting_hotspot_traffic.txt", "w") as f:
        f.write("# src\tdest\tpir\tpor\tt_on\tt_off\tt_duration\n")
        
        for phase in range(4):
            t_on = phase * PHASE_DURATION
            t_off = t_on + PHASE_DURATION
            hotspot_nodes = get_quadrant_nodes(phase)
            
            for src in range(NUM_NODES):
                # Assign PIR based on whether node is in the current hotspot quadrant
                pir = HIGH_PIR if src in hotspot_nodes else LOW_PIR
                
                # Distribution: Each node sends to a random destination (Uniformly Distributed)
                # This ensures we are testing SOURCE hotspotting, not destination hotspotting.
                dest = random.choice([n for n in range(NUM_NODES) if n != src])
                
                # Format: src dest pir por t_on t_off t_duration
                # por (Probability of Retransmission) is usually set to 1.0 or pir
                f.write(f"{src}\t{dest}\t{pir}\t{pir}\t{t_on}\t{t_off}\t{100001}\n")

if __name__ == "__main__":
    generate_traffic()
    print("Traffic table 'shifting_hotspot_traffic.txt' generated successfully.")