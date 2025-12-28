
filename = "bin/traffic_tables/sparse_dual_hotspot.txt"
pir = 0.05
num_nodes = 64
active_nodes = [0, 32]

with open(filename, "w") as f:
    f.write("% Source Destination PIR\n")
    
    for src in active_nodes:
        # Distribute PIR equally among all other nodes
        # Total PIR for this node should be 0.05
        # Number of destinations = num_nodes - 1
        
        individual_pir = pir / (num_nodes - 1)
        
        for dst in range(num_nodes):
            if src != dst:
                f.write(f"{src} {dst} {individual_pir:.9f}\n")

print(f"Generated {filename}")
