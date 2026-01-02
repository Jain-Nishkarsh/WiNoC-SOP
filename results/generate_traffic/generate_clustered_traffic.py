import os

def generate_clustered_hotspot(filename, mesh_dim_x, mesh_dim_y, cluster_nodes, active_pir):
    num_nodes = mesh_dim_x * mesh_dim_y
    
    print(f"Generating Clustered Hotspot table for {mesh_dim_x}x{mesh_dim_y} mesh.")
    print(f"Active Nodes (The Cluster): {cluster_nodes}")
    print(f"PIR per Active Node: {active_pir}")
    
    with open(filename, 'w') as f:
        f.write("% Source Destination PIR\n")
        
        for src in range(num_nodes):
            if src in cluster_nodes:
                # Active nodes in the cluster send to everyone else
                # This ensures the buffers stay full and the 'Success' score is high
                pair_pir = active_pir / (num_nodes - 1)
                for dst in range(num_nodes):
                    if src != dst:
                        f.write(f"{src} {dst} {pair_pir:.9f}\n")
            else:
                # All other 60 nodes are silent (PIR = 0)
                pass

if __name__ == "__main__":
    output_dir = "bin/traffic_tables"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    # Cluster: Top-left 2x2 (Nodes 0, 1, 8, 9 in row-major 8x8 mesh)
    cluster = [0, 1, 8, 9]
    filename = os.path.join(output_dir, "cluster_hotspot_2x2.txt")
    
    # We use a high PIR (0.05 flits/cycle) to ensure the cluster is a true hotspot
    generate_clustered_hotspot(filename, 8, 8, cluster, 0.05)
    print(f"Traffic table generated at {filename}")