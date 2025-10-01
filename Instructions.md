Nice — I read your uploaded paper and inspected Noxim’s public repo, then mapped FUZZY TOKEN onto Noxim’s architecture and produced a concrete, step-by-step implementation plan (with file-level edits, pseudo-code, config changes, tests, and pitfalls). Below you’ll find:

* a short summary of the FUZZY TOKEN behavior you must model (with paper citations),
* a mapping from that behaviour to Noxim internals (with repo citations), and
* a detailed, actionable implementation checklist (classes/files to add or change, SystemC-style pseudocode for the hard parts, config updates, and tests).

I did this in one pass (no waiting). If you want, I can expand any single code snippet into a full file ready to drop into your tree.

---

# Quick FUZZY TOKEN summary (what to implement)

* FUZZY TOKEN alternates between **focused** and **fuzzy** steps. Each step lasts exactly the duration of the event that happened in that step (successful payload transmission: **C cycles**; collision detection: **2 cycles**; nothing attempted/silence: **1 cycle**). All nodes are synchronized and operate in lockstep. .
* Important state (per step, globally agreed): `periodMode` (fuzzy/focused), `tokenID` (who would have token in focused), a **fuzzy area** (bit vector marking nodes allowed to contend), and a per-node transmission probability vector `p[i]`. Nodes inside fuzzy area may attempt to transmit with probability `p[i]` when in fuzzy mode. .
* PHY/medium behaviour required: preamble + sensing. In fuzzy mode a contender first sends a **preamble** then senses channel; the token holder detects simultaneous preambles and then sends a **NACK** (short) to cancel transmissions before payload to save time/energy. If no NACK is observed, sender proceeds to transmit payload for `C` cycles. If collision detected, fuzzy area is multiplicatively reduced (AIMD: e.g., FA_new = ceil(FA_old/2)). On silence the fuzzy area increases (additive, e.g., +1). Thresholds `thr1` and `thr2` determine transitions between modes. .
* Paper defaults used in evaluation: `pi = 1/k` (equal inside fuzzy area), increment `+1` on silence, decrement to half on collision, `thr1 = 0.1×N`, `thr2 = 0.9×N`, static ring order. .

---

# What in Noxim you can reuse / where to plug the MAC

* Noxim already has **wireless transmission support** and a concept of radio/hubs that mediate wireless activity; it is designed to be extended (README and docs). ([GitHub][1]).
* Past Noxim extensions and papers show people modify `Hub.cpp`, `NoC.cpp`, `NoC.h`, `Router.*`, `DataStructs.h`, and the configuration loader when adding wireless features or traffic modes — so those are the natural places to touch. ([Departamento de Informática][2]).
* Some Noxim forks/extensions use token-ring semantics or extend wireless hub models (papers describing WiBS / OFDMA with Noxim explain how the hub/channel is used). Use the hub/channel model as the place to implement preamble detection and token holder NACK behaviour. ([MDPI][3]).

---

# High-level implementation approach (one paragraph)

Implement FUZZY TOKEN as a **MAC layer plugin attached to each wireless RadioHub / Tile**, plus a **single global controller** (a SystemC module/singleton inside Noxim) that keeps synchronized global state (`periodMode`, `tokenID`, `fuzzyArea`, `p[]`) and enforces lockstep end-of-step transitions. Extend the wireless channel / hub code to support a short **PREAMBLE** phase (1 cycle) that multiple senders can start; during that preamble the token holder inspects for multiple active preambles and issues (in 1 cycle) a **NACK** if a collision is detected. If no NACK arrives, sender proceeds to transmit payload for `C` cycles. Update fuzzy area following AIMD and rotate tokenID at step end. (This architecture maps cleanly to Noxim’s hub/channel + tile model.) .

---

# Concrete step-by-step plan (developer checklist)

Below each numbered step I indicate what files to add/modify and give pseudo-code where helpful.

### 1) Add configuration parameters (YAML/config)

**What to change**

* `config_examples/*` and the config loader (likely `ConfigurationManager.cpp` / `LoadConfig`): add new parameters:

  * `mac_type: FUZZY_TOKEN`
  * `fuzzy_token: { C: <cycles>, preamble_cycles: 1, initial_FA: <int>, thr1: <float|count>, thr2: <float|count>, FA_inc: 1, FA_dec_factor: 0.5, pi_type: equal|gaussian, initial_token: 0, token_order: static|pseudo_random }`

**Files**

* `src/ConfigurationManager.cpp` (or the equivalent ) — parse the above params. (Trace docs show extensions routinely modify config loader.) ([Departamento de Informática][2])

---

### 2) Add new MAC support files (new code)

**Files to add**

* `src/mac/FuzzyTokenController.h` / `.cpp` — **singleton global controller** containing `tokenID`, `periodMode`, `fuzzyArea` (bitset), per-node `p[]`, `FA_size`, thresholds and methods to compute the next step and update FA via AIMD. This module also exposes events/methods the hubs will call (see API below).
* `src/mac/FuzzyTokenNode.h` / `.cpp` — per-node helper that is instantiated/attached to each Tile or RadioHub (handles per-node decision to send preamble and reacts to NACK).
* (Optional) `src/mac/FuzzyTokenConfig.h` — small struct with parsed configuration.

**Why a central controller?**
Although the paper says on-chip nodes have a consistent view and can perform implicit token passing, in a simulator it’s **much simpler and less error-prone** to maintain the protocol state in one controller so you avoid subtle race conditions across multiple SystemC processes; the controller will run a step scheduler that all hubs consult. This is standard in Noxim extensions that add coordinated wireless protocols. ([Departamento de Informática][2])

---

### 3) Extend the hub/channel code to support **preamble**, **NACK** and short sensing

**Files to change**

* `src/Hub.cpp` / `src/Hub.h` (or RadioHub equivalent). These already exist in many forks; earlier extension works edited `Hub.cpp`. ([Departamento de Informática][2])
* `src/DataStructs.h` (if you need to add new packet types / flit types e.g., PREAMBLE, NACK, PAYLOAD). Examples in Noxim modify data structs when adding features. ([GitHub][4])

**What to implement**

* Change the wireless transmit flow:

  1. **During a step**: a node that decided to attempt (per FuzzyNode decision — see next) **sends a PREAMBLE** (modeled as a 1-cycle event or a special flit). Hub/channel records active preambles this cycle.
  2. The channel/hub checks: if `count(active_preambles) > 1` during preamble phase, mark `collision_during_preamble`.
  3. **Token holder** (the hub/tile corresponding to `tokenID`) inspects active preambles at the end of preamble phase; if `collision_during_preamble == true`, it generates a **NACK** (1 cycle) which all senders hear and **abort**; else NACK is not sent and senders proceed to **PAYLOAD** (C cycles). (In focused mode token holder is allowed to transmit — policy: if token holder transmits, no preamble competition happens; only in fuzzy steps token holder must be listening.) 

**Notes / timing**

* Use cycles exactly as paper: preamble detection → NACK (if collision) in the same step. Paper states collision detection path takes ~2 cycles, silence 1 cycle, payload C cycles. Model accordingly. 

**Channel data structure**

* Channel must be able to accumulate short preamble "claims" for one cycle and then let controller/hub inspect a list of attempters. Implement a `vector<int> preamble_attempters` per wireless channel cleared each step.

---

### 4) Per-node behaviour (attach to the Tile or NI)

**Files to change**

* `src/Tile.cpp` / `src/Tile.h` or `src/NetworkInterface.cpp` (Noxim naming may vary). In many Noxim mods, people change Tile/Router/NI to attach functionality. ([GitHub][4])

**Behaviour to implement (pseudocode)**

```cpp
// inside NodeTxProcess (called every step)
if (FuzzyTokenController::instance().periodMode == FUZZY) {
    if (this->inFuzzyArea()) {
        double r = uniform_random();
        if (r < p[this->id] && this->hasReadyPacket()) {
            // signal hub: attempt PREAMBLE this step
            hub->request_preamble_send(this->id);
            wait_for_preamble_result();
            if (hub->received_nack_for(this->id)) {
                // cancel; do not transmit payload this step
                report_event(COLLISION_PREAMBLE);
            } else {
                // proceed to payload (C cycles) -> call hub transmit_payload(...)
                hub->transmit_payload(this->id, C);
                report_event(SUCCESS);
            }
        } else {
           // silence: report nothing (the controller will count silence)
        }
    } // else not in fuzzy area: silent
} else { // FOCUSED
    if (this->id == FuzzyTokenController::instance().tokenID &&
        this->hasReadyPacket()) {
        // token holder transmits for C cycles
        hub->transmit_payload(this->id, C);
        report_event(SUCCESS);
    } else {
        // silence
    }
}
```

Where `report_event(...)` notifies the global controller of the step outcome (success/collision/silence). The controller then performs FA updates and token rotation. 

---

### 5) Global step termination and updates (FuzzyTokenController)

**Controller responsibilities**

* At the end of each step (controller will be notified by hubs of what happened), compute the step outcome:

  * If a payload completed successfully: `outcome = SUCCESS`, duration = `C`.
  * If multiple preambles -> token holder sent NACK: `outcome = COLLISION`, duration = `2` cycles (paper).
  * If nobody attempted: `outcome = SILENCE`, duration = `1` cycle. 
* Update `tokenID = (tokenID + 1) % N` and rotate `fuzzyArea` (rotate bitset by +1 in the ring order).
* Update fuzzy area **size**:

  * SILENCE: `FA_size += FA_inc` (e.g., +1).
  * COLLISION: `FA_size = ceil(FA_size * FA_dec_factor)` (e.g., 0.5).
  * SUCCESS: do not change FA_size (paper choice).
* Update `periodMode` according to thresholds `thr1` and `thr2` (if FA_size < thr1 -> focused, if FA_size > thr2 -> fuzzy else follow state machine). 
* Recompute `p[i]` inside the fuzzy area (paper default: `p = 1/k` where `k` is FA_size). 

**SystemC note**: The controller will trigger an `sc_event` (or set a simulation time delta) to cause nodes/hubs to start the next step after the chosen duration.

---

### 6) Data types & logging

* Add new packet/flit type flags: `FLIT_TYPE::PREAMBLE`, `FLIT_TYPE::NACK`, `FLIT_TYPE::PAYLOAD`. Modify `DataStructs.h` accordingly. (Many Noxim extensions do this.) ([GitHub][4])
* Add counters for: preambles attempted, preamble collisions, NACKs, successful payloads, per-node latency; energy accounting: preamble energy vs payload energy (paper uses these metrics). Use existing Noxim flows for metrics collection. 

---

### 7) Build system & glue

* Add the new source files to `src/Makefile` / `CMakeLists.txt`.
* Wire creation: in `NoC.cpp` (or the module that constructs tiles/hubs), instantiate and register `FuzzyTokenNode` with `FuzzyTokenController::instance()` for each wireless tile/hub.

---

### 8) Tests & verification plan (start small)

1. **Unit**: Simulate 4 nodes, fixed traffic schedule (e.g., node 0 and node 1 attempt), step through by hand; verify preamble collision -> NACK cancels both (no payload), check FA updates.
2. **Micro**: Reproduce the simple walkthrough example from the paper (Fig.4) — use the same initial FA and nodes wanting to transmit; log step outcomes and compare to the paper example. 
3. **Macro**: Run synthetic Poisson injection experiments comparing Token vs FUZZY_TOKEN vs BRS (if BRS exists in your branch) and verify latency/throughput trends approx match the paper graphs (latency low at low loads, converging to Token at very high loads). Use the paper default config: `thr1 = 0.1*N`, `thr2 = 0.9*N`. 

---

# Pseudocode: FuzzyTokenController (core)

```cpp
class FuzzyTokenController {
  static FuzzyTokenController* inst;
  int N;
  int tokenID;
  bool periodModeFuzzy; // true => FUZZY, false => FOCUSED
  std::vector<bool> fuzzyArea; // size N
  std::vector<double> p; // per-node prob
  int FA_size;
  int thr1, thr2;
  int C; // payload cycles
  int preamble_cycles;

  // per-step bookkeeping
  std::vector<int> preamble_attempts; // nodes that requested preamble this step
  std::vector<int> payload_attempts;  // nodes that started payload

  void register_preamble_attempt(int nodeID) {
    preamble_attempts.push_back(nodeID);
  }

  void on_step_evaluate_and_advance() {
    if(payload_attempts.size() > 0) { // success (should be 1)
       // SUCCESS
       duration = C;
       // notify nodes/hubs to complete payload
       // do NOT change FA_size
    } else if (preamble_attempts.size() > 1) {
       // COLLISION
       duration = 2;
       FA_size = ceil(FA_size * FA_dec_factor);
       // token holder will have sent NACK (modeled by hub)
    } else if (preamble_attempts.size() == 1) {
       // single preamble -> no NACK -> payload happens => handled as payload_attempts
    } else {
       // SILENCE
       duration = 1;
       FA_size += FA_inc;
    }

    // rotate token and fuzzyArea
    tokenID = (tokenID + 1) % N;
    rotate(fuzzyArea.begin(), fuzzyArea.begin()+1, fuzzyArea.end());

    // update periodMode based on thr1/thr2
    if (FA_size <= thr1) periodModeFuzzy = false;
    else if (FA_size >= thr2) periodModeFuzzy = true;
    // else keep current or follow state-machine rules from paper

    // recompute p[i] inside FA (default equal)
    int k = count(fuzzyArea.begin(), fuzzyArea.end(), true);
    for(i) p[i] = fuzzyArea[i] ? (1.0/k) : 0.0;

    // reset per-step bookkeeping
    preamble_attempts.clear();
    payload_attempts.clear();

    // schedule next step start after `duration` cycles (SystemC sc_time conversion).
  }
};
```

---

# File / symbol map: *exact* places you'll likely edit in Noxim

> NOTE: Noxim repo is laid out with `src/` containing `NoC.cpp`, `Tile.cpp`, `Router.*`, `Hub.*`, `DataStructs.h`, and `ConfigurationManager` (past extension work edited these). Use the repo’s structure as the map for where to add/modify code. ([GitHub][1])

Suggested concrete edits:

* **Add** `src/mac/FuzzyTokenController.{h,cpp}`, `src/mac/FuzzyTokenNode.{h,cpp}`, `src/mac/FuzzyTokenConfig.{h,cpp}`
* **Modify** `src/Hub.cpp` / `src/Hub.h`:

  * Add `request_preamble_send(nodeID)` and `transmit_payload(nodeID, C)` handlers; implement preamble bookkeeping and NACK issuance when appropriate.
* **Modify** (or attach to) `src/Tile.cpp` / `src/NetworkInterface.cpp`:

  * Instantiate `FuzzyTokenNode` for nodes that have a radio, and replace the “send full packet directly” logic with the preamble→sense→payload flow when `mac_type==FUZZY_TOKEN`.
* **Modify** `src/DataStructs.h` / `Packet.h`:

  * Add PREAMBLE/NACK flags (short control flits) and small sizes.
* **Modify** `src/ConfigurationManager.cpp`:

  * Parse new YAML keys and fill `FuzzyTokenConfig`.
* **Modify** `src/NoC.cpp`:

  * Register nodes with `FuzzyTokenController` on NoC building.
* **Makefile/CMakeLists**: add new sources.

(Existing Noxim forks and papers indicate these are the files typically touched for wireless/mac additions.) ([Departamento de Informática][2])

---

# Important implementation pitfalls & tips

* **Synchronization/lockstep**: the entire algorithm relies on all nodes advancing in lockstep each step. In SystemC this must be enforced carefully — use the `FuzzyTokenController` to schedule next-step events and have hubs/nodes wait on that event rather than racing on local timers. Race conditions otherwise lead to inconsistent tokenID/fuzzyArea views. 
* **Where to implement NACK**: The paper has the token holder send the NACK (energy saving). In Noxim, that means the hub/tile that corresponds to the token holder must be able to send a *short control frame* fast enough. Make sure the hub has permission to transmit a NACK during the preamble window. 
* **Energy accounting**: preamble + aborts change the energy model. The paper calculates energy-per-bit including preamble & re-transmissions; if you use Noxim’s power model you must add preamble energy and NACK overhead appropriately. 
* **Testing with deterministic seeds**: because FA updates can be pseudo-random (if you choose randomized token order), guarantee reproducible runs by making RNG seeds part of config. The paper explicitly mentions synchronized RNG if you choose runtime token order changes. 

---

# Validation experiments (matching paper)

Use the same experiments the paper ran to be confident your model matches:

* **Synthetic loads**: Poisson with uniform injection — sweep injection rate and measure latency/throughput/energy; use paper defaults (FA inc +1, FA dec = half, thr1 = 0.1N thr2 = 0.9N). 
* **Real applications**: paper evaluated Replica + Splash2/Parsec/Crono via Multi2Sim; you can use the same traffic traces if available, or simpler benchmarks to validate trends. The paper used OOK, 20Gb/s and a preamble of 20 bits (80-bit total packet with 20-bit preamble). Add those PHY assumptions into the Noxim config for energy/latency mapping. 

---

# What I inspected and where I got the information

* Your uploaded paper **Fuzzy Token Adaptive MAC Protocol.pdf** — used to extract the algorithm, timing, defaults, state machine and examples. (Multiple in-paper snippets used above.)
* Noxim repo & README — confirmed Noxim already supports **wireless** and hub abstractions; repo is the right place to add MAC logic. ([GitHub][1])
* Papers / forks that show where Noxim wireless/hub changes normally occur (people modify `Hub.cpp`, `NoC.cpp`, `ConfigurationManager` etc.). Use those as a guide to the file locations to edit. ([Departamento de Informática][2])

---

# Minimal reproducible example to get you started quickly

If you want the quickest path to a working simulation to validate basic FUZZY_TOKEN semantics:

1. Add `FuzzyTokenController` (singleton) that only logs preamble attempts (no actual channel splitting yet).
2. Modify the Tile/NI to call `controller->register_preamble_attempt(nodeID)` when it decides to attempt in fuzzy mode.
3. Make the controller immediately (synchronously) decide: if more than one preamble → collision else success; rotate token and FA accordingly.
4. Once that logic yields the same step outcomes as the paper’s small example, implement the hub/channel preamble/NACK modelling and energy model changes.

This lets you iterate quickly (logic + FA updates) before diving into channel timing and SystemC event scheduling.

---

# Final notes & next steps I can do for you (pick any)

* Turn one of the pseudocode blocks above into a full SystemC `FuzzyTokenController.{h,cpp}` and `FuzzyTokenNode.{h,cpp}` ready to drop into `src/` (I can produce complete .h/.cpp files).
* Generate a patch/PR checklist with exact line ranges to change in a checked-out copy of `davidepatti/noxim` (I’ll need the exact branch you use — or I can target `master`).
* Produce a minimal YAML example and a 4-node test harness config that reproduces the Fig.4 walkthrough.

Tell me which of those you want next and I’ll output the files / patch content right away. (I already used the paper you uploaded and public Noxim sources to produce this plan; citations above point to the exact places in each source I relied on.)  ([GitHub][1])

---

If you want me to start generating code files now, say “generate code” and tell me whether you want the *quick prototype* (controller logic only) or the *full implementation* (controller + hub/channel preamble/NACK + NI changes + data types + config).

[1]: https://github.com/davidepatti/noxim "GitHub - davidepatti/noxim: Network on Chip Simulator"
[2]: https://web.inf.ufpr.br/mazalves/wp-content/uploads/sites/13/2019/10/daes2019.pdf?utm_source=chatgpt.com "[PDF] Trace-driven and processing time extensions for Noxim simulator"
[3]: https://www.mdpi.com/2079-9268/10/4/37?utm_source=chatgpt.com "Framework for Design Exploration and Performance Analysis of RF ..."
[4]: https://github.com/davidepatti/noxim/issues/93?utm_source=chatgpt.com "Modification of the Mesh Topology #93 - davidepatti/noxim"
