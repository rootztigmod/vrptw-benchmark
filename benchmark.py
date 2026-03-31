"""
VRPTW Benchmark — TIG algorithms vs OR-Tools vs SINTEF Best-Known Solutions
============================================================================
Benchmarks TIG vehicle-routing algorithms against Gehring & Homberger
instances, reporting gap % vs SINTEF BKS and vs OR-Tools (Google).

No Docker required.  Pass --algo <name> and this script handles everything:
  • Writes src/algo_dispatch.rs pointing at the algorithm in tig-monorepo
  • Rebuilds the Rust binary if the algorithm changed
  • Runs all requested instances and prints the results table

Quick start (from WSL):
    python3 benchmark.py --algo <your_algo> --group C1
    python3 benchmark.py --algo <your_algo> --exploration 4 --csv results.csv
    python3 benchmark.py --compare <algo_a> <algo_b>
    python3 benchmark.py --algo <your_algo> --no-ortools
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG  (edit if your layout differs)
# ─────────────────────────────────────────────────────────────────────────────

# Root of this repo (Cargo.toml lives here)
REPO = Path(__file__).resolve().parent

# Directory containing the HG .TXT instance files — bundled inside this repo
HG_DIR = REPO / "src" / "HG"

# Root of tig-monorepo (always cloned to home directory)
TIG_MONOREPO = Path.home() / "tig-monorepo"

DEFAULT_ALGO         = ""
DEFAULT_EXPLORATION  = 4
DEFAULT_ORTOOLS_TIME = 60.0

# ─────────────────────────────────────────────────────────────────────────────
# SINTEF BEST-KNOWN SOLUTIONS  (vehicles, distance)
# Source: https://www.sintef.no/projectweb/top/vrptw/homberger-benchmark/
# ─────────────────────────────────────────────────────────────────────────────

BKS = {
    "c1_2_1":  (20, 2704.57), "c1_2_2":  (18, 2917.89), "c1_2_3":  (18, 2707.35),
    "c1_2_4":  (18, 2643.31), "c1_2_5":  (20, 2702.05), "c1_2_6":  (20, 2701.04),
    "c1_2_7":  (20, 2701.04), "c1_2_8":  (19, 2775.48), "c1_2_9":  (18, 2687.83),
    "c1_2_10": (18, 2643.51),
    "c2_2_1":  ( 6, 1931.44), "c2_2_2":  ( 6, 1863.16), "c2_2_3":  ( 6, 1775.08),
    "c2_2_4":  ( 6, 1703.43), "c2_2_5":  ( 6, 1878.85), "c2_2_6":  ( 6, 1857.35),
    "c2_2_7":  ( 6, 1849.46), "c2_2_8":  ( 6, 1820.53), "c2_2_9":  ( 6, 1830.05),
    "c2_2_10": ( 6, 1806.58),
    "r1_2_1":  (20, 4784.11), "r1_2_2":  (18, 4039.86), "r1_2_3":  (18, 3381.96),
    "r1_2_4":  (18, 3057.81), "r1_2_5":  (18, 4107.86), "r1_2_6":  (18, 3583.14),
    "r1_2_7":  (18, 3150.11), "r1_2_8":  (18, 2951.99), "r1_2_9":  (18, 3760.58),
    "r1_2_10": (18, 3301.18),
    "r2_2_1":  ( 4, 4483.16), "r2_2_2":  ( 4, 3621.20), "r2_2_3":  ( 4, 2880.62),
    "r2_2_4":  ( 4, 1981.30), "r2_2_5":  ( 4, 3366.79), "r2_2_6":  ( 4, 2913.03),
    "r2_2_7":  ( 4, 2451.14), "r2_2_8":  ( 4, 1849.87), "r2_2_9":  ( 4, 3092.04),
    "r2_2_10": ( 4, 2654.97),
    "rc1_2_1": (18, 3602.80), "rc1_2_2": (18, 3249.05), "rc1_2_3": (18, 3008.33),
    "rc1_2_4": (18, 2851.68), "rc1_2_5": (18, 3371.00), "rc1_2_6": (18, 3324.80),
    "rc1_2_7": (18, 3189.32), "rc1_2_8": (18, 3083.93), "rc1_2_9": (18, 3081.13),
    "rc1_2_10":(18, 3000.30),
    "rc2_2_1": ( 6, 3099.53), "rc2_2_2": ( 5, 2825.24), "rc2_2_3": ( 4, 2601.87),
    "rc2_2_4": ( 4, 2038.56), "rc2_2_5": ( 4, 2911.46), "rc2_2_6": ( 4, 2873.12),
    "rc2_2_7": ( 4, 2525.83), "rc2_2_8": ( 4, 2292.53), "rc2_2_9": ( 4, 2175.04),
    "rc2_2_10":( 4, 2015.61),
}

def _discover_instances(hg_dir: Path) -> list[str]:
    """
    Scan hg_dir for .TXT files and return sorted lowercase instance names.
    Filters out Windows Zone.Identifier sidecar files (e.g. C1_2_1.TXT:Zone.Identifier).
    """
    names = []
    for p in hg_dir.iterdir():
        name = p.name
        # Must end with .TXT or .txt, not contain ':' (Zone.Identifier sidecars)
        if ":" in name:
            continue
        if not (name.upper().endswith(".TXT")):
            continue
        stem = p.stem.lower()
        names.append(stem)
    return sorted(names)


def _build_group_map(instances: list[str]) -> dict:
    """Derive group map from discovered instance names (groups by uppercase prefix before _2_)."""
    groups: dict[str, list[str]] = {}
    for name in instances:
        # e.g. "c1_2_1" → prefix "C1",  "rc2_2_10" → prefix "RC2"
        prefix = name.split("_2_")[0].upper() if "_2_" in name else name.split("_")[0].upper()
        groups.setdefault(prefix, []).append(name)
    return {k: sorted(v) for k, v in sorted(groups.items())}


# Discovered at import time from the bundled HG directory
ALL_INSTANCES = _discover_instances(HG_DIR) if HG_DIR.exists() else []
GROUP_MAP     = _build_group_map(ALL_INSTANCES)

# ─────────────────────────────────────────────────────────────────────────────
# ALGO DISPATCH WRITER
# ─────────────────────────────────────────────────────────────────────────────

DISPATCH_FILE = REPO / "src" / "algo_dispatch.rs"

def write_algo_dispatch(algo: str, monorepo: Path = TIG_MONOREPO) -> bool:
    """
    Generate src/algo_dispatch.rs for the given algorithm name.

    Uses Rust's #[path] attribute to point directly at the algorithm's mod.rs
    inside tig-monorepo — no files in tig-monorepo are ever modified.

    Returns True if the file changed (rebuild needed), False if unchanged.
    """
    algo_mod_path = monorepo / "tig-algorithms" / "src" / "vehicle_routing" / algo / "mod.rs"

    if not algo_mod_path.exists():
        print(f"ERROR: algorithm source not found at {algo_mod_path}")
        print(f"  Make sure '{algo}' exists in tig-monorepo/tig-algorithms/src/vehicle_routing/")
        sys.exit(1)

    content = f"""\
// AUTO-GENERATED by benchmark.py — do not edit by hand.
// Re-generated every time --algo changes; cargo then recompiles only this file.
use anyhow::Result;
use serde_json::{{Map, Value}};
use tig_challenges::vehicle_routing::{{Challenge, Solution}};

pub const ALGO_NAME: &str = "{algo}";

#[path = "{algo_mod_path}"]
mod algo_impl;

pub fn dispatch(
    challenge: &Challenge,
    save_fn: &dyn Fn(&Solution) -> Result<()>,
    hyperparams: &Option<Map<String, Value>>,
) -> Result<()> {{
    algo_impl::solve_challenge(challenge, save_fn, hyperparams)
}}
"""

    if DISPATCH_FILE.exists() and DISPATCH_FILE.read_text() == content:
        return False  # unchanged

    DISPATCH_FILE.write_text(content)
    print(f"  Written src/algo_dispatch.rs  (algo={algo})")
    return True  # changed, rebuild needed


# ─────────────────────────────────────────────────────────────────────────────
# BUILD
# ─────────────────────────────────────────────────────────────────────────────

EVALUATOR_BIN = REPO / "target" / "release" / "vrptw-evaluator"

def build(force: bool = False):
    """Build the Rust evaluator. Skipped if binary exists and dispatch unchanged."""
    if not force and EVALUATOR_BIN.exists():
        return
    print("Building vrptw-evaluator (first build ~1-2 min, incremental rebuilds are fast)...")
    result = subprocess.run(
        ["cargo", "build", "--release", "--manifest-path", str(REPO / "Cargo.toml")],
    )
    if result.returncode != 0:
        print("Build failed — see errors above.")
        sys.exit(1)
    print("Build complete.\n")


def prepare_algo(algo: str, monorepo: Path = TIG_MONOREPO):
    """Write algo_dispatch.rs and rebuild only if algo changed."""
    changed = write_algo_dispatch(algo, monorepo)
    build(force=changed)


# ─────────────────────────────────────────────────────────────────────────────
# TIG EVALUATOR RUNNER
# ─────────────────────────────────────────────────────────────────────────────

def run_tig(instances: list[str], exploration_level: int, hg_dir: Path) -> dict:
    """
    Run the compiled vrptw-evaluator binary, streaming progress in real-time.
    Returns dict  { instance_name_lower: result_dict }
    """
    cmd = [
        str(EVALUATOR_BIN),
        "--hg-dir", str(hg_dir),
        "--exploration-level", str(exploration_level),
    ]
    if instances:
        cmd += ["--instances"] + instances

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    stdout_lines = []

    # Stream stderr (progress) live; collect stdout (NDJSON results)
    import threading

    def stream_stderr():
        for line in proc.stderr:
            print(f"  {line}", end="", flush=True)

    stderr_thread = threading.Thread(target=stream_stderr, daemon=True)
    stderr_thread.start()

    for line in proc.stdout:
        stdout_lines.append(line)

    proc.wait()
    stderr_thread.join()

    results = {}
    for line in stdout_lines:
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
            results[r["instance"]] = r
        except json.JSONDecodeError:
            pass

    return results


# ─────────────────────────────────────────────────────────────────────────────
# PYVRP RUNNER
# ─────────────────────────────────────────────────────────────────────────────

def _find_hg_path(name: str, hg_dir: Path) -> Optional[Path]:
    for suffix in [name.upper(), name.lower(), name]:
        for ext in [".TXT", ".txt"]:
            p = hg_dir / (suffix + ext)
            if p.exists():
                return p
    return None


def _parse_hg_instance(path: Path) -> dict:
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip()]
    vi = next(i for i, l in enumerate(lines) if l.startswith("NUMBER"))
    parts = lines[vi + 1].split()
    nb_vehicles, capacity = int(parts[0]), int(parts[1])
    ci = next(i for i, l in enumerate(lines) if l.startswith("CUST"))
    customers = []
    for l in lines[ci + 1:]:
        p = l.split()
        if len(p) < 7:
            continue
        try:
            int(p[0])
        except ValueError:
            continue
        customers.append({
            "x": float(p[1]), "y": float(p[2]),
            "demand": int(p[3]), "ready": int(p[4]),
            "due": int(p[5]), "service": int(p[6]),
        })
    return {"nb_vehicles": nb_vehicles, "capacity": capacity, "customers": customers}


def _run_ortools_instance(inst: dict, time_limit: float) -> Optional[tuple]:
    try:
        from ortools.constraint_solver import routing_enums_pb2, pywrapcp
    except ImportError:
        return None
    import math

    customers  = inst["customers"]
    n          = len(customers)
    nb_vehicles = inst["nb_vehicles"]
    capacity   = inst["capacity"]
    depot      = customers[0]
    horizon    = depot["due"]

    # Integer Euclidean distance matrix
    def edist(i, j):
        dx = customers[i]["x"] - customers[j]["x"]
        dy = customers[i]["y"] - customers[j]["y"]
        return int(round(math.sqrt(dx * dx + dy * dy)))

    dist_matrix = [[edist(i, j) for j in range(n)] for i in range(n)]

    manager = pywrapcp.RoutingIndexManager(n, nb_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    # Arc cost = Euclidean distance
    def dist_cb(fi, ti):
        return dist_matrix[manager.IndexToNode(fi)][manager.IndexToNode(ti)]
    dist_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)

    # Capacity constraint
    def demand_cb(fi):
        return customers[manager.IndexToNode(fi)]["demand"]
    dem_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        dem_idx, 0, [capacity] * nb_vehicles, True, "Capacity")

    # Time dimension: transit = travel time + service time at origin node
    def time_cb(fi, ti):
        fn = manager.IndexToNode(fi)
        tn = manager.IndexToNode(ti)
        return dist_matrix[fn][tn] + customers[fn]["service"]
    time_idx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(time_idx, horizon, horizon, False, "Time")
    td = routing.GetDimensionOrDie("Time")

    # Time windows for every node (including depot)
    for node in range(n):
        td.CumulVar(manager.NodeToIndex(node)).SetRange(
            customers[node]["ready"], customers[node]["due"])

    # Depot time window on vehicle start/end arcs
    for v in range(nb_vehicles):
        td.CumulVar(routing.Start(v)).SetRange(depot["ready"], depot["due"])
        td.CumulVar(routing.End(v)).SetRange(depot["ready"], depot["due"])
        routing.AddVariableMinimizedByFinalizer(td.CumulVar(routing.Start(v)))
        routing.AddVariableMinimizedByFinalizer(td.CumulVar(routing.End(v)))

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    params.time_limit.seconds = int(max(1, time_limit))
    params.log_search = False

    t0 = time.time()
    try:
        solution = routing.SolveWithParameters(params)
        elapsed  = time.time() - t0
    except Exception as e:
        print(f" [OR-Tools exception: {e}]", end="")
        return None

    if solution:
        total_dist, nv_used = 0, 0
        for v in range(nb_vehicles):
            idx = routing.Start(v)
            if not routing.IsEnd(solution.Value(routing.NextVar(idx))):
                nv_used += 1
            while not routing.IsEnd(idx):
                nxt = solution.Value(routing.NextVar(idx))
                total_dist += routing.GetArcCostForVehicle(idx, nxt, v)
                idx = nxt
        return float(total_dist), nv_used, elapsed

    print(f" [no solution found]", end="")
    return None


def run_ortools_all(instances: list[str], time_limit: float, hg_dir: Path) -> dict:
    results = {}
    for i, name in enumerate(instances, 1):
        print(f"  OR-Tools [{i}/{len(instances)}] {name} ...", end="", flush=True)
        path = _find_hg_path(name, hg_dir)
        if path is None:
            print(" (instance file not found)")
            continue
        inst = _parse_hg_instance(path)
        r = _run_ortools_instance(inst, time_limit)
        if r:
            dist, nv, elapsed = r
            results[name] = {"distance": dist, "num_vehicles": nv, "elapsed_s": elapsed}
            print(f" dist={dist:.1f}  nv={nv}  t={elapsed:.1f}s")
        else:
            print(" no feasible solution found")
    return results


# ─────────────────────────────────────────────────────────────────────────────
# TABLE PRINTERS
# ─────────────────────────────────────────────────────────────────────────────

def _gap(our_dist: float, bks_dist: float) -> float:
    return (our_dist - bks_dist) / bks_dist * 100.0


def print_single_table(
    instances: list[str],
    algo: str,
    tig_results: dict,
    ortools_results: dict,
    show_ortools: bool,
):
    cols = 116 if show_ortools else 80
    print(f"\n{'═' * cols}")
    hdr = (f"{'Instance':<13} {'BKS V':>6} {'BKS Dist':>10}"
           f" │ {algo[:14]:>14} {'NV':>4} {'Gap%':>7} {'Time':>7}")
    if show_ortools:
        hdr += f" │ {'OR-Tools':>10} {'NV':>4} {'Gap%':>7} {'Time':>7}"
    print(hdr)
    print(f"{'─' * cols}")

    tig_gaps, ot_gaps = [], []

    for name in instances:
        bks_v, bks_d = BKS.get(name, (0, 0.0))
        tr = tig_results.get(name, {})
        pr = ortools_results.get(name, {}) if show_ortools else {}

        tig_d  = tr.get("distance", 0.0)
        tig_ok = tr.get("status") == "ok" and tig_d > 0

        tig_gap_str = ""
        if tig_ok and bks_d > 0:
            g = _gap(tig_d, bks_d)
            tig_gap_str = f"{g:+.2f}%"
            tig_gaps.append(g)

        tig_d_str  = f"{tig_d:.2f}"                          if tig_ok else tr.get("status", "-")
        tig_nv_str = str(tr.get("num_vehicles", "-"))         if tig_ok else "-"
        tig_t_str  = f"{tr.get('elapsed_ms', 0)/1000:.1f}s"  if tig_ok else "-"

        row = (f"{name:<13} {bks_v:>6} {bks_d:>10.2f}"
               f" │ {tig_d_str:>14} {tig_nv_str:>4} {tig_gap_str:>7} {tig_t_str:>7}")

        if show_ortools:
            ot_d  = pr.get("distance", 0.0)
            ot_ok = bool(pr)
            ot_gap_str = ""
            if ot_ok and bks_d > 0:
                g = _gap(ot_d, bks_d)
                ot_gap_str = f"{g:+.2f}%"
                ot_gaps.append(g)
            ot_d_str  = f"{ot_d:.2f}"                         if ot_ok else "-"
            ot_nv_str = str(pr.get("num_vehicles", "-"))       if ot_ok else "-"
            ot_t_str  = f"{pr.get('elapsed_s', 0):.1f}s"      if ot_ok else "-"
            row += (f" │ {ot_d_str:>10} {ot_nv_str:>4} {ot_gap_str:>7} {ot_t_str:>7}")

        print(row)

    print(f"{'─' * cols}")

    avg_tig = f"{sum(tig_gaps)/len(tig_gaps):+.2f}%" if tig_gaps else "-"
    avg_ot  = f"{sum(ot_gaps)/len(ot_gaps):+.2f}%"  if ot_gaps  else "-"
    summary = (f"{'AVERAGE':<13} {'':>6} {'':>10}"
               f" │ {'':>14} {'':>4} {avg_tig:>7} {'':>7}")
    if show_ortools:
        summary += f" │ {'':>10} {'':>4} {avg_ot:>7} {'':>7}"
    print(summary)
    print(f"{'═' * cols}\n")

    if tig_gaps:
        print(f"  {algo:>16}  avg gap vs BKS: {sum(tig_gaps)/len(tig_gaps):+.2f}%"
              f"  ({len(tig_gaps)}/{len(instances)} solved)")
    if show_ortools and ot_gaps:
        print(f"  {'OR-Tools':>16}  avg gap vs BKS: {sum(ot_gaps)/len(ot_gaps):+.2f}%"
              f"  ({len(ot_gaps)}/{len(instances)} solved)")
    print()


def print_compare_table(instances: list[str], algos: list[str], all_results: list[dict]):
    cols = 13 + 18 + len(algos) * 22 + 4
    print(f"\n{'═' * cols}")
    hdr = f"{'Instance':<13} {'BKS Dist':>10}"
    for a in algos:
        hdr += f" │ {a[:12]:>12} {'Gap%':>7}"
    print(hdr)
    print(f"{'─' * cols}")

    avg_gaps = [[] for _ in algos]

    for name in instances:
        _, bks_d = BKS.get(name, (0, 0.0))
        row = f"{name:<13} {bks_d:>10.2f}"
        for idx, results in enumerate(all_results):
            tr = results.get(name, {})
            d  = tr.get("distance", 0.0)
            ok = tr.get("status") == "ok" and d > 0
            d_str = f"{d:.2f}" if ok else tr.get("status", "-")
            gap_str = ""
            if ok and bks_d > 0:
                g = _gap(d, bks_d)
                gap_str = f"{g:+.2f}%"
                avg_gaps[idx].append(g)
            row += f" │ {d_str:>12} {gap_str:>7}"
        print(row)

    print(f"{'─' * cols}")
    avg_row = f"{'AVERAGE':<13} {'':>10}"
    for idx, gaps in enumerate(avg_gaps):
        avg_str = f"{sum(gaps)/len(gaps):+.2f}%" if gaps else "-"
        avg_row += f" │ {'':>12} {avg_str:>7}"
    print(avg_row)
    print(f"{'═' * cols}\n")

    for algo, gaps in zip(algos, avg_gaps):
        if gaps:
            print(f"  {algo:>16}  avg gap vs BKS: {sum(gaps)/len(gaps):+.2f}%"
                  f"  ({len(gaps)}/{len(instances)} solved)")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# CSV EXPORT
# ─────────────────────────────────────────────────────────────────────────────

def save_csv(path: str, instances: list[str], algo: str,
             tig_results: dict, ortools_results: dict):
    import csv
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["instance", "bks_vehicles", "bks_dist",
                    "tig_dist", "tig_vehicles", "tig_gap_pct", "tig_time_s", "tig_status",
                    "ortools_dist", "ortools_vehicles", "ortools_gap_pct", "ortools_time_s"])
        for name in instances:
            bks_v, bks_d = BKS.get(name, (0, 0.0))
            tr = tig_results.get(name, {})
            pr = ortools_results.get(name, {})
            d  = tr.get("distance", 0.0)
            ok = tr.get("status") == "ok" and d > 0
            w.writerow([
                name, bks_v, bks_d,
                f"{d:.2f}"  if ok else "", tr.get("num_vehicles","") if ok else "",
                f"{_gap(d,bks_d):.4f}" if ok and bks_d else "",
                f"{tr.get('elapsed_ms',0)/1000:.2f}" if ok else "",
                tr.get("status",""),
                f"{pr.get('distance',0):.2f}" if pr else "",
                pr.get("num_vehicles","") if pr else "",
                f"{_gap(pr['distance'],bks_d):.4f}" if pr and bks_d else "",
                f"{pr.get('elapsed_s',0):.2f}" if pr else "",
            ])
    print(f"Results saved to {path}")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global HG_DIR, TIG_MONOREPO

    parser = argparse.ArgumentParser(
        description="VRPTW Benchmark: TIG algorithm vs OR-Tools vs SINTEF BKS",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--algo", default=DEFAULT_ALGO,
        help="TIG algorithm name (must exist in ~/tig-monorepo/tig-algorithms/src/vehicle_routing/)",
    )
    parser.add_argument(
        "--compare", nargs="+", metavar="ALGO",
        help="Compare multiple algorithms side-by-side (overrides --algo)",
    )
    parser.add_argument(
        "--instances", nargs="+", metavar="NAME",
        help="Specific instance names, e.g. C1_2_1 C1_2_2",
    )
    parser.add_argument(
        "--group", choices=list(GROUP_MAP.keys()) or None,
        help=f"Run all instances in a group: {' '.join(GROUP_MAP.keys())}",
    )
    parser.add_argument(
        "--exploration", type=int, default=DEFAULT_EXPLORATION,
        help=f"exploration_level for TIG algorithm 0-4 (default: {DEFAULT_EXPLORATION})",
    )
    parser.add_argument(
        "--no-ortools", action="store_true",
        help="Skip OR-Tools comparison",
    )
    parser.add_argument(
        "--ortools-only", action="store_true",
        help="Skip TIG algorithm, run OR-Tools only",
    )
    parser.add_argument(
        "--ortools-time", type=float, default=DEFAULT_ORTOOLS_TIME,
        help=f"OR-Tools time limit per instance in seconds (default: {DEFAULT_ORTOOLS_TIME})",
    )
    parser.add_argument(
        "--csv", metavar="FILE",
        help="Save results to a CSV file",
    )
    parser.add_argument(
        "--hg-dir", default=str(HG_DIR),
        help=f"Directory containing HG .TXT files (default: {HG_DIR})",
    )
    parser.add_argument(
        "--tig-monorepo", default=str(TIG_MONOREPO),
        help=f"Path to tig-monorepo root (default: {TIG_MONOREPO})",
    )

    args = parser.parse_args()

    HG_DIR       = Path(args.hg_dir)
    TIG_MONOREPO = Path(args.tig_monorepo)

    if not HG_DIR.exists():
        print(f"ERROR: HG directory not found: {HG_DIR}")
        print("Pass the correct path with --hg-dir")
        sys.exit(1)

    # Re-discover instances in case --hg-dir was overridden
    discovered   = _discover_instances(HG_DIR)
    group_map    = _build_group_map(discovered)

    if args.instances:
        instances = [n.lower() for n in args.instances]
    elif args.group:
        instances = group_map.get(args.group.upper(), [])
        if not instances:
            print(f"ERROR: group '{args.group}' not found. Available: {list(group_map.keys())}")
            sys.exit(1)
    else:
        instances = discovered

    run_tig_flag    = not args.ortools_only
    run_ortools_flag = not args.no_ortools and not args.compare

    # ── COMPARE MODE ──────────────────────────────────────────────────────────
    if args.compare:
        algos = args.compare
        print(f"\nComparing {len(algos)} algorithms on {len(instances)} instances "
              f"(exploration_level={args.exploration})")
        all_results = []
        for algo in algos:
            print(f"\n  Preparing {algo}...")
            prepare_algo(algo, TIG_MONOREPO)
            print(f"  Running {algo}...")
            r = run_tig(instances, args.exploration, HG_DIR)
            all_results.append(r)
        print_compare_table(instances, algos, all_results)
        if args.csv and all_results:
            save_csv(args.csv, instances, algos[0], all_results[0], {})
        return

    # ── SINGLE ALGO MODE ──────────────────────────────────────────────────────
    algo = args.algo
    if not algo and not args.compare:
        parser.error("--algo is required. Example: python3 benchmark.py --algo <your_algo_name>")

    if run_tig_flag:
        print(f"\nPreparing {algo}...")
        prepare_algo(algo, TIG_MONOREPO)

    tig_results, ortools_results = {}, {}

    if run_tig_flag:
        print(f"Running {algo} on {len(instances)} instances "
              f"(exploration_level={args.exploration})...")
        tig_results = run_tig(instances, args.exploration, HG_DIR)

    if run_ortools_flag:
        try:
            from ortools.constraint_solver import pywrapcp  # noqa: F401
        except ImportError:
            print("\nOR-Tools not installed — skipping (pip install ortools)")
            run_ortools_flag = False

    if run_ortools_flag:
        print(f"\nRunning OR-Tools (time_limit={args.ortools_time}s per instance)...")
        ortools_results = run_ortools_all(instances, args.ortools_time, HG_DIR)

    print_single_table(instances, algo, tig_results, ortools_results, run_ortools_flag)

    if args.csv:
        save_csv(args.csv, instances, algo, tig_results, ortools_results)


if __name__ == "__main__":
    main()
