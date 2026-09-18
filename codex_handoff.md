# SIH Codex Handoff: Collection Pipeline, Thermal Mitigation, and Pose-Sync Investigation

## Read this first

This repository has an `AGENTS.md` policy. Do **not** run Gazebo, fleet launchers, final collection scripts, or long-running simulations. Perform read-only log analysis and static checks only unless the user explicitly authorizes an implementation after reviewing a forensic plan.

This handoff records verified facts as of 2026-09-19. It intentionally distinguishes evidence from hypotheses.

## Current collection setup

- Desktop: Ryzen 5 5600X, RTX 3070 8 GB, 16 GB RAM, MSI B450M-A PRO MAX.
- Standard desktop collection profile: 4 AMRs, 0.46 m/s tracking speed, 5 Hz LiDAR, headless Gazebo, kinematic LiDAR carrier backend.
- Desktop telemetry root: `~/amr_ws/log/sih_data_collection/desktop_data_collection_*/**/fleet_telemetry.jsonl`.
- Run-specific startup metadata is in each sibling `run_events.jsonl`.
- Compiled desktop dataset: `collected_datasets_desktop.csv`; dashboard copy: `frontend/collected_datasets_desktop.csv`.
- The user operates final collection runs; agents must not launch them.

## Work completed before this handoff

### Dataset collection organization

Commit `93e01d1` (`Add profile-specific dataset collection`) established profile-specific collection behavior:

- Desktop and laptop runners default to `-f 4`.
- `-d` rebuilds the relevant desktop or laptop aggregate from saved telemetry.
- Desktop output: `collected_datasets_desktop.csv`.
- Laptop output: `collected_datasets_laptop.csv`.
- The legacy `all_collected_dataset.csv` was removed because it can be regenerated from JSONL telemetry.
- Collection logs were consolidated under `~/amr_ws/log/sih_data_collection/`.

### Desktop telemetry monitor

An executable host-only monitor exists at:

```bash
~/Documents/monitor_fleet_resources.sh
```

It writes timestamped CSVs in `~/Documents/` and records CPU temperature/utilization/frequency/load/RAM/swap plus Nvidia GPU temperature/utilization/memory/power. Normal use:

```bash
~/Documents/monitor_fleet_resources.sh 5
```

No monitor source is tracked in this repository.

### Thermal history and mitigation

Before the power cap, active 4-AMR collection repeatedly reached 95–96 C. The CPU is a Ryzen 5 5600X, whose thermal ceiling is 95 C; the GPU was cool and not the limiting component.

The user tested AMD Eco Mode / a 45 W package-power cap. It worked substantially better than the attempted RTF-only mitigation:

- Latest monitor: `~/Documents/fleet_resource_metrics_20260919_011346.csv`.
- 151 samples at 5-second intervals.
- Active (CPU utilization >= 50%) CPU temperature: **78.0 C average**, **79.0 C maximum**.
- Overall CPU peak: **79.6 C**; zero samples at or above 85 C.
- GPU: 36.9 C average, 39 C maximum; no swap use; roughly 11.7 GB RAM available.

An apples-to-apples six-task comparison used the same seed (1001), robot count (4), speed (0.46 m/s), and unthrottled server setting:

| Run | Wall makespan | Task rate | Effective RTF |
| --- | ---: | ---: | ---: |
| Pre-Eco: `desktop_data_collection_20260919_000806` | 395.20 s | 54.66/h | 6.581x |
| Eco 45 W: `desktop_data_collection_20260919_011835` | 394.64 s | 54.73/h | 6.498x |

The observed loss is about 1.3% by full-run RTF and indistinguishable from normal run variance by task makespan. Eco 45 W is the preferred temporary collection profile while waiting for the replacement CPU cooler and rear exhaust fan.

### Optional RTF control

Commit `3d49b8c` (`updated data, script argument agmentation`) includes an opt-in RTF control:

```bash
python3 scripts/run_desktop_data_collection.py -rtf 2.5 ...
```

- Omit `-rtf` to retain historic unthrottled behavior.
- The runner converts RTF to a Gazebo server wall-clock update rate using the existing 0.02 s physics step. Example: `-rtf 2.5` -> `gz sim -z 125`.
- `scripts/launch_fleet_amrs.sh` logs `target_rtf` into `run_events.jsonl`.
- `-w/--world PATH` is also supported as a separate world-file override.

An early `warehouse_clean_rtf3.sdf` copy was created outside this repository at:

`~/amr_ws/src/warehouse_world_custom/worlds/small_warehouse/warehouse_clean_rtf3.sdf`

Do not rely on that world file's `<real_time_factor>` tags alone: they did not enforce the intended throttle in this Harmonic launcher. Use `-rtf` if an RTF cap is desired.

## The issue requiring forensic analysis: carrier pose synchronization failures

### Executive finding

The observation that robots 3 and 4 seemed to stop is **not isolated** and is **not attributable to Eco 45 W**. It coincides with repeated kinematic-carrier-to-Gazebo pose synchronization failures. The resulting missing peer state causes the CBBA layer to safely withhold assignments/commits and the path follower to hold blocked corridors.

This is an evidence-backed correlation. The deeper source of `set_pose_vector` timeouts has **not** been proven yet; do not claim a root cause beyond the documented failed service calls without further investigation.

### Most relevant problematic run

Run directory:

`~/amr_ws/log/sih_data_collection/desktop_data_collection_20260919_011835/desktop_run_001_20260919_011835/`

Summary:

- 6 completed tasks, normal shutdown, 394.64 s measured makespan.
- Carrier log telemetry ended at **7,278 pose-sync failures out of 21,831 attempts (33.3%)**.
- 94 `WITHHOLD_ASSIGNMENT` events were recorded.

Task sequence shows this was not specifically robots 3 and 4 being retired:

| Task | Assigned robot | Reached terminal task phase |
| --- | --- | --- |
| 001 | robot_1 | yes |
| 002 | robot_2 | yes |
| 003 | robot_3 | yes |
| 004 | robot_4 | yes |
| 005 | robot_1 | yes |
| 006 | robot_3 | yes |
| 007 | robot_4 | no |
| 008 | robot_2 | no |
| 009 | robot_1 | no |
| 010 | robot_3 | no |

Robot 3 received task 010 after completing task 006. Robot 4 had already received task 007. All four robots therefore became affected after tasks 7–10 remained in flight; the next task could not reach consensus reliably.

Representative telemetry messages from this run:

```text
[Carrier] Gazebo set_pose_vector failed (timeout/unreachable). Sync failures: 7278/21831.
[robot_3:CBBA] Decision: WITHHOLD_ASSIGNMENT ... missing CBBA participants=[...].
[robot_4:CBBA] Decision: WITHHOLD_COMMIT ... missing_claims=[...].
[robot_3:PathFollower] Decision: HOLD_CORRIDOR_BLOCKED ... speed_cap=0.0.
```

### Prevalence in recent runs

The carrier failure signature is consistent across both short and 20-task runs, before and after the Eco cap:

| Collection run | Completed tasks | Last cumulative carrier failure count | CBBA assignment withholds |
| --- | ---: | ---: | ---: |
| `20260919_000806/run_001` (pre-Eco comparable run) | 6 | 7,360 / 22,107 = 33.3% | 94 |
| `20260919_011333/run_001` (`-rtf 2.5`) | 6 | 7,385 / 22,779 = 32.4% | 74 |
| `20260919_011835/run_001` (Eco 45 W, unthrottled) | 6 | 7,278 / 21,831 = 33.3% | 94 |
| `20260919_013058/run_001` (Eco 45 W, unthrottled) | 20 | 15,535 / 43,777 = 35.5% | 231 |
| `20260918_224551/run_002` | 20 | 16,487 / 45,159 = 36.5% | 225 |

The latest completed 20-task run still distributed completions across the fleet (robot 1: 5, robot 2: 5, robot 3: 6, robot 4: 4), but the pose-sync failures and assignment withholding remained abundant. Thus successful completion does not mean the issue is absent.

### Relevant code and logs

- Carrier source: `src/sih_amr_fleet/sih_amr_fleet/kinematic_carrier_node.py`.
- The warning originates from `_dispatch_gz_pose` around line 451 in the built source recorded by telemetry.
- Launch flow: `scripts/launch_fleet_amrs.sh`.
- Per-run files of interest:
  - `fleet_telemetry.jsonl`: task assignments, terminal task phases, health, CBBA/path-follower warnings, carrier warnings.
  - `run_events.jsonl`: profile, server start timestamp, target RTF.
  - `kinematic_carrier.log`, `gazebo_server.log`, and `fleet.log`: supporting raw process logs.

## Recommended next-agent analysis: read-only first

1. Scan every saved desktop telemetry JSONL and extract, per run:
   - task target/completed count, wall makespan, completion distribution by robot;
   - first/last carrier failure numerator and denominator, plus failure rate;
   - counts/times of `WITHHOLD_ASSIGNMENT`, `WITHHOLD_COMMIT`, and `HOLD_CORRIDOR_BLOCKED`;
   - whether tasks remain assigned without reaching terminal phase 4.
2. Join this with `run_events.jsonl` to segment by `target_rtf`. Eco 45 W is not directly recorded in the telemetry; infer it only from the chronology above or ask the user.
3. Test whether higher carrier failure rates predict stalled tasks, long makespans, or robot imbalance. Preserve a distinction between correlation and causation.
4. Inspect `kinematic_carrier_node.py` and the raw carrier/server logs to establish why `set_pose_vector` times out: request rate, service availability, request-response lifetime, queueing, or a Gazebo-side condition are all candidates, but none is yet proven.
5. Present the quantified forensic result and an implementation plan to the user. Do not modify code or run live simulations until the user explicitly approves that plan.

## Repository state at handoff

- Current top commit: `3d49b8c updated data, script argument agmentation`.
- The SIH repository was clean immediately before this handoff file was added.
- `~/amr_ws/src/warehouse_world_custom` is a separate, already-dirty workspace with unrelated modifications and untracked assets. Treat all existing changes there as user-owned. Its untracked `warehouse_clean_rtf3.sdf` is the early nonfunctional SDF-only RTF experiment described above.
- Do not overwrite or delete collection logs or generated datasets during analysis.
