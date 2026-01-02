import os

def generate_traffic_table(filename, mesh_dim_x, mesh_dim_y, target_avg_pir, hotspot_node, hotspot_percentage):
    num_nodes = mesh_dim_x * mesh_dim_y
    total_injection = target_avg_pir * num_nodes
    
    hotspot_share = hotspot_percentage / 100.0
    others_share = 1.0 - hotspot_share
    
    hotspot_pir = total_injection * hotspot_share
    others_total_pir = total_injection * others_share
    other_node_pir = others_total_pir / (num_nodes - 1)
    
    print(f"Generating traffic table for {mesh_dim_x}x{mesh_dim_y} mesh.")
    print(f"Target Average PIR: {target_avg_pir}")
    print(f"Hotspot Node: {hotspot_node} with {hotspot_percentage}% traffic")
    print(f"Hotspot Node PIR: {hotspot_pir:.6f}")
    print(f"Other Nodes PIR: {other_node_pir:.6f}")
    
    with open(filename, 'w') as f:
        f.write("% Source Destination PIR\n")
        
        for src in range(num_nodes):
            # Determine source PIR
            if src == hotspot_node:
                src_pir = hotspot_pir
            else:
                src_pir = other_node_pir
            
            # Distribute equally to all other nodes
            # We have (num_nodes - 1) destinations
            pair_pir = src_pir / (num_nodes - 1)
            
            for dst in range(num_nodes):
                if src != dst:
                    f.write(f"{src} {dst} {pair_pir:.9f}\n")

if __name__ == "__main__":
    output_dir = "traffic_tables"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    filename = os.path.join(output_dir, "source_hotspot_40pct.txt")
    generate_traffic_table(filename, 8, 8, 0.004, 0, 40)
    print(f"Traffic table generated at {filename}")
