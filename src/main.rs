/// VRPTW Evaluator — standalone binary for benchmarking TIG vehicle-routing
/// algorithms against Gehring & Homberger (HG) instances.
///
/// Zero modifications to tig-monorepo required.  The algorithm to run is
/// selected by benchmark.py, which generates src/algo_dispatch.rs and then
/// calls `cargo build --release`.  This binary just calls algo_dispatch::dispatch().
///
/// Output: one NDJSON line per instance on stdout; progress on stderr.
mod algo_dispatch;

use anyhow::{anyhow, Result};
use clap::Parser;
use rayon::prelude::*;
use serde::Serialize;
use serde_json::{json, Map, Value};
use std::cell::RefCell;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::Instant;
use tig_challenges::vehicle_routing::{Challenge, Solution};

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(name = "vrptw-evaluator")]
#[command(about = "Benchmark TIG VRPTW algorithms against Gehring & Homberger instances")]
#[command(version)]
struct Cli {
    /// Directory containing HG .TXT files
    #[arg(short = 'd', long)]
    hg_dir: PathBuf,

    /// Specific instance names (e.g. C1_2_1 C1_2_2). Omit to run all .TXT files.
    #[arg(short, long, num_args = 0..)]
    instances: Vec<String>,

    /// Run only instances whose name starts with this prefix (e.g. C1, R2, RC1)
    #[arg(short, long)]
    group: Option<String>,

    /// exploration_level passed to the algorithm (0 = fastest … 4 = best quality)
    #[arg(short, long, default_value = "3")]
    exploration_level: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// HG INSTANCE PARSER
// ─────────────────────────────────────────────────────────────────────────────

struct HgCustomer {
    x: f64,
    y: f64,
    demand: i32,
    ready: i32,
    due: i32,
    service: i32,
}

struct HgInstance {
    fleet_size: usize,
    capacity: i32,
    /// Index 0 = depot
    customers: Vec<HgCustomer>,
}

fn parse_hg_txt(path: &Path) -> Result<HgInstance> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| anyhow!("cannot read {:?}: {}", path, e))?;

    let lines: Vec<&str> = text
        .lines()
        .map(|l| l.trim())
        .filter(|l| !l.is_empty())
        .collect();

    let vi = lines
        .iter()
        .position(|l| l.starts_with("NUMBER"))
        .ok_or_else(|| anyhow!("no NUMBER/CAPACITY header found"))?;
    let vdata: Vec<&str> = lines
        .get(vi + 1)
        .ok_or_else(|| anyhow!("missing vehicle data row"))?
        .split_whitespace()
        .collect();
    let fleet_size = vdata
        .first()
        .ok_or_else(|| anyhow!("missing fleet size"))?
        .parse::<usize>()?;
    let capacity = vdata
        .get(1)
        .ok_or_else(|| anyhow!("missing capacity"))?
        .parse::<i32>()?;

    let ci = lines
        .iter()
        .position(|l| l.starts_with("CUST"))
        .ok_or_else(|| anyhow!("no CUSTOMER section found"))?;

    let mut customers = Vec::new();
    for l in &lines[ci + 1..] {
        let p: Vec<&str> = l.split_whitespace().collect();
        if p.len() < 7 {
            continue;
        }
        if p[0].parse::<usize>().is_err() {
            continue;
        }
        customers.push(HgCustomer {
            x: p[1].parse::<f64>()?,
            y: p[2].parse::<f64>()?,
            demand: p[3].parse()?,
            ready: p[4].parse()?,
            due: p[5].parse()?,
            service: p[6].parse()?,
        });
    }

    if customers.is_empty() {
        return Err(anyhow!("no customers parsed from {:?}", path));
    }

    Ok(HgInstance { fleet_size, capacity, customers })
}

// ─────────────────────────────────────────────────────────────────────────────
// CHALLENGE BUILDER
// ─────────────────────────────────────────────────────────────────────────────

fn build_challenge(inst: &HgInstance) -> Result<Challenge> {
    let n = inst.customers.len();
    let service_time = inst.customers.get(1).map(|c| c.service).unwrap_or(0);

    let demands: Vec<i32>    = inst.customers.iter().map(|c| c.demand).collect();
    let ready_times: Vec<i32> = inst.customers.iter().map(|c| c.ready).collect();
    let due_times: Vec<i32>   = inst.customers.iter().map(|c| c.due).collect();
    let node_positions: Vec<[i32; 2]> = inst
        .customers
        .iter()
        .map(|c| [c.x.round() as i32, c.y.round() as i32])
        .collect();

    let mut dm = vec![vec![0i32; n]; n];
    for i in 0..n {
        for j in 0..n {
            if i != j {
                let dx = inst.customers[i].x - inst.customers[j].x;
                let dy = inst.customers[i].y - inst.customers[j].y;
                dm[i][j] = dx.hypot(dy).round() as i32;
            }
        }
    }

    let seed_zeros: Vec<u8> = vec![0u8; 32];
    let json_val = json!({
        "seed": seed_zeros,
        "num_nodes": n,
        "demands": demands,
        "node_positions": node_positions,
        "distance_matrix": dm,
        "max_capacity": inst.capacity,
        "fleet_size": inst.fleet_size,
        "service_time": service_time,
        "ready_times": ready_times,
        "due_times": due_times,
        "greedy_baseline_total_distance": 0u32
    });

    serde_json::from_value(json_val)
        .map_err(|e| anyhow!("failed to deserialise Challenge: {}", e))
}

// ─────────────────────────────────────────────────────────────────────────────
// SOLVER
// ─────────────────────────────────────────────────────────────────────────────

fn solve(challenge: &Challenge, exploration_level: usize) -> Result<Option<(Solution, u128)>> {
    let mut hp_map = Map::new();
    hp_map.insert(
        "exploration_level".to_string(),
        Value::Number(exploration_level.into()),
    );
    let hyperparams = Some(hp_map);

    let saved: RefCell<Option<Solution>> = RefCell::new(None);
    let save_fn = |sol: &Solution| -> Result<()> {
        *saved.borrow_mut() = Some(sol.clone());
        Ok(())
    };

    let t0 = Instant::now();
    algo_dispatch::dispatch(challenge, &save_fn, &hyperparams)?;
    let elapsed_ms = t0.elapsed().as_millis();

    Ok(saved.into_inner().map(|sol| (sol, elapsed_ms)))
}

// ─────────────────────────────────────────────────────────────────────────────
// PER-INSTANCE RESULT
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct InstanceResult {
    instance: String,
    algo: String,
    status: String,
    distance: f64,
    num_vehicles: usize,
    elapsed_ms: u128,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

fn run_instance(path: &Path, exploration_level: usize) -> InstanceResult {
    let stem = path
        .file_stem()
        .unwrap_or_default()
        .to_string_lossy()
        .to_lowercase();

    macro_rules! fail {
        ($status:expr, $msg:expr) => {
            return InstanceResult {
                instance: stem.clone(),
                algo: algo_dispatch::ALGO_NAME.to_string(),
                status: $status.to_string(),
                distance: 0.0,
                num_vehicles: 0,
                elapsed_ms: 0,
                error: Some($msg),
            }
        };
    }

    let inst = match parse_hg_txt(path) {
        Ok(i) => i,
        Err(e) => fail!("parse_error", e.to_string()),
    };

    let challenge = match build_challenge(&inst) {
        Ok(c) => c,
        Err(e) => fail!("build_error", e.to_string()),
    };

    let (solution, elapsed_ms) = match solve(&challenge, exploration_level) {
        Ok(Some(pair)) => pair,
        Ok(None) => fail!("no_solution", "algorithm returned no solution".to_string()),
        Err(e) => fail!("solver_error", e.to_string()),
    };

    match challenge.evaluate_total_distance(&solution) {
        Ok(dist) => {
            // Solution.routes is private so we can't access it directly.
            // serde serialisation works (Serialize derive is inside tig-challenges),
            // but as a belt-and-braces fallback we also parse the Debug string.
            // Each route starts with depot 0 so "[0," appears once per route.
            let num_vehicles = serde_json::to_value(&solution)
                .ok()
                .and_then(|v| v.get("routes").and_then(|r| r.as_array()).map(|a| a.len()))
                .unwrap_or_else(|| {
                    // Fallback: Debug format  →  "Solution { routes: [[0, 1, .., 0], ..] }"
                    format!("{:?}", solution).matches("[0,").count()
                });

            InstanceResult {
                instance: stem,
                algo: algo_dispatch::ALGO_NAME.to_string(),
                status: "ok".to_string(),
                distance: dist as f64,
                num_vehicles,
                elapsed_ms,
                error: None,
            }
        }
        Err(e) => InstanceResult {
            instance: stem,
            algo: algo_dispatch::ALGO_NAME.to_string(),
            status: "invalid_solution".to_string(),
            distance: 0.0,
            num_vehicles: 0,
            elapsed_ms,
            error: Some(e.to_string()),
        },
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// INSTANCE DISCOVERY
// ─────────────────────────────────────────────────────────────────────────────

fn discover_instances(
    hg_dir: &Path,
    names: &[String],
    group: Option<&str>,
) -> Result<Vec<PathBuf>> {
    if !names.is_empty() {
        let mut paths = Vec::new();
        for name in names {
            let found = ["TXT", "txt"]
                .iter()
                .flat_map(|ext| {
                    let ext = *ext;
                    [name.to_uppercase(), name.to_lowercase(), name.clone()]
                        .into_iter()
                        .map(move |n| hg_dir.join(format!("{}.{}", n, ext)))
                })
                .find(|p| p.exists());
            match found {
                Some(p) => paths.push(p),
                None => eprintln!("WARNING: instance '{}' not found in {:?}", name, hg_dir),
            }
        }
        return Ok(paths);
    }

    let mut all: Vec<PathBuf> = std::fs::read_dir(hg_dir)
        .map_err(|e| anyhow!("cannot read hg_dir {:?}: {}", hg_dir, e))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| {
            // Accept only plain .TXT files; reject Windows Zone.Identifier sidecars
            // (which have names like "C1_2_1.TXT:Zone.Identifier" and appear as
            // extension "Identifier" with a colon in the stem on some FS mounts).
            let fname = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
            if fname.contains(':') {
                return false;
            }
            p.extension()
                .and_then(|x| x.to_str())
                .map(|x| x.eq_ignore_ascii_case("txt"))
                .unwrap_or(false)
        })
        .filter(|p| {
            if let Some(g) = group {
                p.file_stem()
                    .and_then(|s| s.to_str())
                    .map(|s| s.to_uppercase().starts_with(&g.to_uppercase()))
                    .unwrap_or(false)
            } else {
                true
            }
        })
        .collect();

    all.sort();
    Ok(all)
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    let args = Cli::parse();
    let paths = discover_instances(&args.hg_dir, &args.instances, args.group.as_deref())?;

    if paths.is_empty() {
        eprintln!("No .TXT instances found in {:?}", args.hg_dir);
        return Ok(());
    }

    let n_threads = rayon::current_num_threads();
    eprintln!(
        "vrptw-evaluator | algo={} | exploration_level={} | instances={} | threads={}",
        algo_dispatch::ALGO_NAME,
        args.exploration_level,
        paths.len(),
        n_threads,
    );

    // Run instances in parallel. Progress lines are streamed to stderr as each
    // instance completes. NDJSON lines are collected and printed in input order
    // afterwards so Python always gets deterministic output.
    let exploration_level = args.exploration_level;
    let stderr = Mutex::new(std::io::stderr());
    let results: Vec<InstanceResult> = paths
        .par_iter()
        .map(|path| {
            let result = run_instance(path, exploration_level);
            // Stream progress immediately as each instance finishes
            let line = format!(
                "  {:14} → {:18} dist={:8.1}  nv={:2}  t={}ms{}\n",
                result.instance,
                result.status,
                result.distance,
                result.num_vehicles,
                result.elapsed_ms,
                result
                    .error
                    .as_deref()
                    .map(|e| format!("  [{}]", e))
                    .unwrap_or_default()
            );
            let _ = stderr.lock().map(|mut s| s.write_all(line.as_bytes()));
            result
        })
        .collect();

    // NDJSON to stdout in stable order for Python parsing
    for result in &results {
        println!("{}", serde_json::to_string(&result)?);
    }

    Ok(())
}
