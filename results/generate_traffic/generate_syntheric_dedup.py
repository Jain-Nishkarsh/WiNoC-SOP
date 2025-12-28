# generate_dedup_mimic.py
import random

with open("bin/traffic_tables/dedup_hotspot.txt", "w") as f:
    for src in range(64):
        # Designate center nodes as "Source Hotspots" (Compressors)
        is_hotspot = src in [27, 28, 35, 36]
        injection_rate = 0.05 if is_hotspot else 0.01
        
        for dst in range(64):
            if src == dst: continue
            # Probability of communication
            f.write(f"{src} {dst} {injection_rate} 8\n")