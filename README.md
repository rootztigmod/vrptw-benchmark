# VRPTW Benchmark

Benchmarks TIG vehicle-routing algorithms against the Gehring & Homberger (HG)
1000-customer instances, comparing results against:

- **SINTEF Best-Known Solutions (BKS)** — the reference optimal distances
- **OR-Tools (Google)** — a production-grade VRPTW solver used as a SOTA baseline

No Docker required. The benchmark compiles a standalone Rust binary that pulls
algorithm source directly from your local `tig-monorepo` clone.

---

## Example Output

```
════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
Instance       BKS V   BKS Dist │   my_algorithm   NV    Gap%    Time │   OR-Tools   NV    Gap%    Time
────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
c1_2_1            20    2704.57 │        2690.00   20  -0.54%    0.5s │    2691.00   20  -0.50%   60.0s
c1_2_2            18    2917.89 │        2685.00   20  -7.98%    0.7s │    2721.00   20  -6.75%   60.0s
c1_2_3            18    2707.35 │        2664.00   20  -1.60%    0.6s │    2757.00   20  +1.83%   60.0s
c1_2_4            18    2643.31 │        2645.00   20  +0.06%    0.9s │    2795.00   20  +5.74%   60.0s
c1_2_5            20    2702.05 │        2686.00   20  -0.59%    0.7s │    2689.00   20  -0.48%   60.0s
────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
AVERAGE                         │                      -1.88%         │                  +1.09%
════════════════════════════════════════════════════════════════════════════════════════════════════════════════════

    my_algorithm  avg gap vs BKS: -1.88%  (10/10 solved)
        OR-Tools  avg gap vs BKS: +1.09%  (10/10 solved)
```

A negative gap % means the algorithm **beats** the best known solution.

---

## Prerequisites

### 1. Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

### 2. Python dependencies

```bash
pip install ortools numpy
```

### 3. tig-monorepo

Clone it into your home directory (the benchmark looks for it at `~/tig-monorepo`):

```bash
git clone https://github.com/tig-foundation/tig-monorepo ~/tig-monorepo
```

### 4. This repo

```bash
git clone https://github.com/rootztigmod/vrptw-benchmark ~/vrptw-benchmark
cd ~/vrptw-benchmark
```

The HG instance files are bundled in `src/HG/`. No separate download needed.

---

## Quick Start

```bash
python3 benchmark.py --algo <your_algo> --group C1
```

The first run compiles the Rust binary (~15s). Subsequent runs reuse it unless
the algorithm changes.

---

## All Command-Line Options

| Flag | Default | Description |
|---|---|---|
| `--algo NAME` | — | TIG algorithm name to benchmark (required) |
| `--group GROUP` | all | Instance group: `C1 C2 R1 R2 RC1 RC2` |
| `--instances A B` | — | Run specific instances by name |
| `--exploration N` | `4` | exploration_level passed to the algorithm (0–4) |
| `--ortools-time N` | `60` | OR-Tools time limit per instance in seconds |
| `--no-ortools` | — | Skip OR-Tools, show TIG vs BKS only |
| `--ortools-only` | — | Skip TIG, run OR-Tools only |
| `--compare A B ...` | — | Compare multiple algorithms side-by-side |
| `--csv FILE` | — | Save results to a CSV file |
| `--hg-dir PATH` | `src/HG` | Override HG dataset directory |
| `--tig-monorepo PATH` | `~/tig-monorepo` | Override tig-monorepo location |

---

## Usage Examples

```bash
# One group, no OR-Tools (fastest)
python3 benchmark.py --algo <your_algo> --group C1 --no-ortools

# One group with OR-Tools comparison (30s per instance)
python3 benchmark.py --algo <your_algo> --group C1 --ortools-time 30

# All 60 HG instances, no OR-Tools
python3 benchmark.py --algo <your_algo> --no-ortools

# Full benchmark — all 60 instances with OR-Tools comparison
python3 benchmark.py --algo <your_algo> --ortools-time 30

# Compare two algorithms side-by-side
python3 benchmark.py --compare <algo_a> <algo_b> --group C1

# Save results to CSV
python3 benchmark.py --algo <your_algo> --group C1 --csv results.csv

# Run specific instances
python3 benchmark.py --algo <your_algo> --instances c1_2_1 c1_2_2 c1_2_3
```

---

## Adding Your Algorithm

Your algorithm must exist in `tig-monorepo` at:

```
~/tig-monorepo/tig-algorithms/src/vehicle_routing/<algo_name>/mod.rs
```

The `mod.rs` must export a `solve_challenge` function with this signature:

```rust
pub fn solve_challenge(
    challenge: &Challenge,
    save_fn: &dyn Fn(&Solution) -> Result<()>,
    hyperparams: &Option<Map<String, Value>>,
) -> Result<()>
```

This is the standard TIG algorithm interface — any existing TIG VRPTW algorithm
works without modification. Then run:

```bash
python3 benchmark.py --algo <your_algo_name> --group C1
```

The script generates the dispatch module and recompiles automatically.

---

## Instance Groups

The HG benchmark contains 60 instances across 6 groups (10 instances each):

| Group | Type | Characteristics |
|---|---|---|
| `C1` | Clustered | Narrow time windows |
| `C2` | Clustered | Wide time windows |
| `R1` | Random | Narrow time windows |
| `R2` | Random | Wide time windows |
| `RC1` | Mixed | Narrow time windows |
| `RC2` | Mixed | Wide time windows |

All instances have 1000 customers.

---

## How It Works

1. `benchmark.py` writes `src/algo_dispatch.rs` — a thin Rust module that uses
   `#[path]` to include your algorithm's `mod.rs` directly from `tig-monorepo`
   without modifying any files there.
2. `cargo build --release` compiles the `vrptw-evaluator` binary (only when the
   algorithm changes).
3. The binary runs all instances in parallel across all available CPU threads,
   streaming results back as NDJSON.
4. OR-Tools solves the same instances sequentially as a SOTA comparison.
5. Results are printed as a formatted table with gap % vs SINTEF BKS.
