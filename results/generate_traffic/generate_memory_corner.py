def generate_memory_corner_hotspot_16x16(filename="bin/traffic_tables/memory_corners_16x16.txt"):
    mesh_dim = 16
    num_nodes = mesh_dim * mesh_dim
    hotspot_nodes = [0, 15, 240, 255] # The 4 Corners
    hotspot_traffic_ratio = 0.6       # 60% of all traffic goes to Memory Controllers
    
    with open(filename, 'w') as f:
        f.write("% Source Destination PIR\n")
        for src in range(num_nodes):
            # 60% of traffic distributed among the 4 corners
            for dst in hotspot_nodes:
                if src != dst:
                    pir = (hotspot_traffic_ratio / len(hotspot_nodes)) / (num_nodes - 1)
                    f.write(f"{src} {dst} {pir:.9f}\n")
            
            # 40% of traffic is uniform random background noise
            uniform_pir = (1.0 - hotspot_traffic_ratio) / (num_nodes - 1)
            for dst in range(num_nodes):
                if src != dst and dst not in hotspot_nodes:
                    f.write(f"{src} {dst} {uniform_pir:.9f}\n")

generate_memory_corner_hotspot_16x16()