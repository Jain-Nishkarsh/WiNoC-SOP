# Figure 1: Wireless Throughput vs. Packet Injection Rate

## Description

This figure demonstrates the relationship between packet injection rate (PIR) and wireless network throughput for three different MAC protocols:

1. **Token Passing** - Baseline round-robin token passing protocol
2. **Fuzzy Token** - Prior art with fuzzy area contention
3. **FP-Jump** - Proposed Fuzzy-Predictive Jump protocol

## Simulation Parameters

| Parameter | Value |
|-----------|-------|
| Topology | 8×8 Mesh (64 nodes) |
| Routing | Deterministic XY |
| Buffer Depth | 64 flits |
| Packet Size | 5 flits |
| Modulation | OOK @ 60 GHz |
| Data Rate | 20 Gbps |
| Traffic Model | Soteriou (H=0.8, σ=1.0) |
| Simulation Time | 20,000 cycles |
| Warmup | 1,000 cycles |
| Seeds | 10 (geometric mean) |

## Output Files

- `fig1_throughput.png` - High-resolution PNG (300 DPI)
- `fig1_throughput.eps` - Vector EPS for IEEE publication

## Running the Script

```bash
cd results/paper_results/fig1_throughput_vs_pir
python plot.py
```

## Expected Behavior

- Under light load (low PIR), all protocols exhibit similar throughput
- As PIR increases, FP-Jump maintains higher throughput by adaptively jumping to active nodes
- Token Passing saturates earliest due to blind token rotation
- Fuzzy Token shows improvement over Token Passing but less than FP-Jump
