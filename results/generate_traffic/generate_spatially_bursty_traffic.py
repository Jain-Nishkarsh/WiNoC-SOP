import os

def generate_spatial_stress_test(filename="bin/traffic_tables/spatial_stress_test.txt"):
    num_nodes = 64
    # Define 3 clusters (Sources and Destinations stay within the cluster)
    # Cluster 1: Top-Left (0, 1, 8, 9)
    # Cluster 2: Center (27, 28, 35, 36)
    # Cluster 3: Bottom-Right (54, 55, 62, 63)
    clusters = [
        [0, 1, 8, 9],
        [27, 28, 35, 36],
        [54, 55, 62, 63]
    ]
    
    intra_cluster_pir = 0.005  # High traffic inside clusters
    background_pir = 0.0001    # Near-zero traffic elsewhere
    
    with open(filename, 'w') as f:
        f.write("% Source Destination PIR\n")
        # Generate Cluster Traffic
        for cluster in clusters:
            for src in cluster:
                for dst in cluster:
                    if src != dst:
                        f.write(f"{src} {dst} {intra_cluster_pir:.6f}\n")
        
        # Generate some background noise to keep it realistic
        for src in range(num_nodes):
            dst = (src + 32) % 64 # Distant destination
            f.write(f"{src} {dst} {background_pir:.6f}\n")

generate_spatial_stress_test()
print("Traffic table 'spatial_stress_test.txt' generated.")