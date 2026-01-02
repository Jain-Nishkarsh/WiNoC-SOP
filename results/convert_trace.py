input_file = "../netrace/golden_slice.txt"
output_file = "bin/traffic_tables/canneal-simmedium-slice.txt"

# First, find the offset (the cycle of the very first packet)
with open(input_file, 'r') as f:
    first_line = f.readline()
    offset = int(first_line.split()[0])

print(f"Normalizing trace. Subtracting offset: {offset}")

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    f_out.write("% Source Destination PIR POR T_on T_off T_period\n")
    # Reset file pointer to read from the start again
    f_in.seek(0)
    
    count = 0
    for line in f_in:
        p = line.split()
        if len(p) < 3: continue
        
        # Subtract the offset to start at (or near) 0
        original_cycle = int(p[0])
        new_cycle = original_cycle - offset
        
        # Ensure we don't have negative cycles
        if new_cycle < 0: new_cycle = 0
        
        src = p[1]
        dst = p[2]
        
        # Format: src dst PIR POR T_on T_off T_period
        f_out.write(f"{src} {dst} 1.0 1.0 {new_cycle} {new_cycle + 2} 1000000000\n")
        count += 1

print(f"✅ Created normalized table with {count} packets.")