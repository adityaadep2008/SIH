#!/usr/bin/env python3
"""Generate Unified Tabular ML Dataset for Multi-AMR Warehouse Fleet Congestion and ETA Modeling.

Extracts the definitive 22-column ML dataset incorporating all routing, spatial congestion,
reservation, queue, velocity, stop time, and dwell metrics:
 1. run_id
 2. task_id
 3. robot_id
 4. start_zone
 5. goal_zone
 6. static_path_length_m
 7. turn_count
 8. junction_crossings_count
 9. nominal_speed_mps
10. fleet_size
11. candidate_corridor_count
12. mean_nearby_robot_count
13. avg_nearby_robot_speed_mps
14. max_corridor_queue_length
15. reservation_count
16. corridor_occupancy_ratio
17. active_blockage_count
18. waiting_time_s
19. total_stop_time_s
20. mean_peer_freshness_ms
21. task_load_count
22. actual_travel_time_s (Target)
"""

import argparse
import csv
import glob
import json
import math
import os
import pathlib
import sys
from collections import defaultdict


def default_log_root():
    """Return the portable, canonical location for collected SIH telemetry."""
    override = os.environ.get('SIH_DATA_LOG_DIR')
    if override:
        return pathlib.Path(override).expanduser()

    legacy_root = os.environ.get('AMR_WS_LOG_DIR')
    if legacy_root:
        return pathlib.Path(legacy_root).expanduser() / 'sih_data_collection'

    return pathlib.Path.home() / 'amr_ws' / 'log' / 'sih_data_collection'


def compute_topological_distance(px, py, dx, dy):
    """Compute true warehouse topological path distance through aisle egress and main transit lanes."""
    start_zone = "South" if py < 0 else "North"
    goal_zone = "South" if dy < 0 else "North"

    if start_zone == goal_zone:
        aisle_egress_y = -20.0 if start_zone == "South" else 20.0
        dist = abs(py - aisle_egress_y) + abs(dx - px) + abs(dy - aisle_egress_y)
        turns = 2
        junctions = 0 if abs(dx - px) < 10.0 else 1
    else:
        egress_y1 = -20.0 if start_zone == "South" else 20.0
        ingress_y2 = -20.0 if goal_zone == "South" else 20.0
        dist = abs(py - egress_y1) + abs(dx - px) + abs(egress_y1 - ingress_y2) + abs(dy - ingress_y2)
        turns = 4
        junctions = 2

    return max(round(dist, 2), round(math.hypot(dx - px, dy - py) * 1.1, 2)), turns, junctions


def parse_telemetry_to_dataset(jsonl_paths, output_csv_path):
    all_dataset_rows = []
    total_files_processed = 0
    total_runs_processed = 0

    for path_str in jsonl_paths:
        p = pathlib.Path(path_str)
        if not p.exists():
            continue

        run_manifest = {}
        task_announcements = {}
        task_assignments = {}
        task_routes = {}
        task_starts = {}
        task_completions = {}
        task_dwells = {}  # tid -> (pickup_wait_s, dropoff_wait_s)
        robot_state_by_time = defaultdict(list)  # t_bucket -> list of (robot_id, x, y, theta, vx, vy)
        robot_state_timestamps = defaultdict(list)  # robot_id -> list of timestamps
        robot_states_timeline = defaultdict(list)  # robot_id -> list of (t, x, y, speed)
        blockage_intervals = []  # list of (t_start, t_valid_until)
        corridor_events = []  # list of (t_event, corridor_id, robot_id, event_type)
        all_robots_seen = set()

        try:
            with open(p, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        record = json.loads(line)
                    except Exception:
                        continue

                    event_type = record.get('event_type')
                    t = record.get('logged_at', record.get('wall_logged_at', 0.0))

                    if event_type == 'run_manifest':
                        run_manifest = record
                    elif event_type == 'task_announcement':
                        tid = record.get('task_id')
                        if tid:
                            task_announcements[tid] = record
                            p_wait = record.get('pickup_wait_s', 2.0)
                            d_wait = record.get('dropoff_wait_s', 2.0)
                            task_dwells[tid] = (float(p_wait), float(d_wait))
                    elif event_type == 'task_assignment':
                        tid = record.get('task_id')
                        if tid:
                            task_assignments[tid] = record
                    elif event_type == 'planned_route':
                        tid = record.get('task_id')
                        if tid and 'waypoints' in record:
                            task_routes[tid] = record['waypoints']
                    elif event_type == 'task_execution':
                        phase = record.get('phase')
                        tid = record.get('task_id')
                        rid = record.get('robot_id')
                        if rid:
                            all_robots_seen.add(rid)

                        if phase == 0:  # EN_ROUTE_PICKUP
                            task_starts.setdefault(tid, (t, rid))
                        elif phase in (3, 4):  # DROPOFF_WAIT or COMPLETED
                            task_completions[tid] = (t, rid)
                    elif event_type == 'robot_state':
                        rid = record.get('robot_id')
                        if rid:
                            all_robots_seen.add(rid)
                            rx, ry = record.get('x', 0.0), record.get('y', 0.0)
                            vx, vy = record.get('vx', 0.0), record.get('vy', 0.0)
                            spd = math.hypot(vx, vy)
                            robot_state_timestamps[rid].append(t)
                            robot_states_timeline[rid].append((t, rx, ry, spd))
                            t_bucket = round(t * 2) / 2.0  # 0.5s resolution bucket
                            robot_state_by_time[t_bucket].append((rid, rx, ry, spd))
                    elif event_type == 'blockage_observation':
                        valid_until = record.get('valid_until_s', t + 25.0)
                        blockage_intervals.append((t, valid_until))
                    elif event_type == 'corridor_protocol':
                        cid = record.get('corridor_id', '')
                        rid = record.get('robot_id', '')
                        ev = record.get('event', 0)
                        corridor_events.append((t, cid, rid, ev))
        except Exception as e:
            print(f"Warning: error reading {p}: {e}")
            continue

        total_files_processed += 1
        run_id = run_manifest.get('run_id', p.parent.name)
        if run_id.startswith('desktop_run_') or run_id.startswith('laptop_run_') or run_id.startswith('run_01_'):
            run_id = p.parent.name

        nominal_speed = float(run_manifest.get('speed_limits', {}).get('tracking_speed_mps', 0.46))
        fleet_size = int(run_manifest.get('robot_count', len(all_robots_seen) or 8))

        # Match completed tasks
        completed_tasks_in_run = []
        for tid, (t_start, start_robot) in task_starts.items():
            if tid not in task_completions:
                continue
            t_end, end_robot = task_completions[tid]
            if t_end <= t_start:
                continue
            duration = max(1.0, t_end - t_start)
            completed_tasks_in_run.append((tid, t_start, t_end, duration, end_robot or start_robot))

        if not completed_tasks_in_run:
            continue

        total_runs_processed += 1

        # Extract features for each completed task
        for tid, t_start, t_end, duration, assigned_robot in completed_tasks_in_run:
            ann = task_announcements.get(tid, {})
            assign = task_assignments.get(tid, {})
            robot_id = assigned_robot or assign.get('robot_id', ann.get('source_robot_id', 'robot_1'))

            px, py = ann.get('pickup_x', 0.0), ann.get('pickup_y', -20.0)
            dx, dy = ann.get('dropoff_x', 0.0), ann.get('dropoff_y', 20.0)

            start_zone = "South" if py < -5.0 else ("North" if py > 5.0 else "Central")
            goal_zone = "South" if dy < -5.0 else ("North" if dy > 5.0 else "Central")

            # 1. Static Path Length & Turns (with robust fallback for truncated routes)
            topo_len, topo_turns, topo_junctions = compute_topological_distance(px, py, dx, dy)
            if tid in task_routes and len(task_routes[tid]) >= 2:
                wps = task_routes[tid]
                exec_len = sum(
                    math.hypot(wps[i+1][0] - wps[i][0], wps[i+1][1] - wps[i][1])
                    for i in range(len(wps) - 1)
                )
                exec_len = round(exec_len, 2)
                # If recorded route length is plausible (>= 60% of topological distance), use it;
                # otherwise only the final rolling-horizon horizon was retained, so use topological distance
                if exec_len >= 0.6 * topo_len:
                    path_len = exec_len
                    turns = 0
                    for i in range(len(wps) - 2):
                        dx1, dy1 = wps[i+1][0] - wps[i][0], wps[i+1][1] - wps[i][1]
                        dx2, dy2 = wps[i+2][0] - wps[i+1][0], wps[i+2][1] - wps[i+1][1]
                        angle1 = math.atan2(dy1, dx1)
                        angle2 = math.atan2(dy2, dx2)
                        diff = abs((angle2 - angle1 + math.pi) % (2 * math.pi) - math.pi)
                        if diff > 0.6:  # > 35 degrees
                            turns += 1
                    junction_crossings = 2 if start_zone != goal_zone else (1 if abs(dx - px) > 10.0 else 0)
                else:
                    path_len, turns, junction_crossings = topo_len, topo_turns, topo_junctions
            else:
                path_len, turns, junction_crossings = topo_len, topo_turns, topo_junctions

            # 2. Candidate Corridor Count
            candidate_corridors = 2 if start_zone != goal_zone else 1

            # 3. Spatial Density & Nearby Robot Speed at Decision Time [t_start - 2.0, t_start]
            nearby_counts = []
            nearby_speeds = []
            start_b = round((t_start - 2.0) * 2) / 2.0
            bid_b = round(t_start * 2) / 2.0
            cur_b = start_b
            while cur_b <= bid_b:
                states = robot_state_by_time.get(cur_b, [])
                this_pos = next(((x, y) for (rid, x, y, spd) in states if rid == robot_id), None)
                if this_pos:
                    tx, ty = this_pos
                    for (rid, ox, oy, ospd) in states:
                        if rid != robot_id and math.hypot(ox - tx, oy - ty) <= 4.0:
                            nearby_speeds.append(ospd)
                    cnt = sum(1 for (rid, ox, oy, ospd) in states if rid != robot_id and math.hypot(ox - tx, oy - ty) <= 3.5)
                    nearby_counts.append(cnt)
                cur_b += 0.5

            mean_nearby = round(sum(nearby_counts) / max(1, len(nearby_counts)), 2) if nearby_counts else 0.0
            avg_nearby_spd = round(sum(nearby_speeds) / max(1, len(nearby_speeds)), 2) if nearby_speeds else round(nominal_speed * 0.4, 2)

            # 4. Corridor Reservations, Queue & Occupancy Ratio at Decision Time
            # Look only at events up to bid time t_start
            res_events_at_bid = [
                (t_ev, cid, r_ev, ev) for (t_ev, cid, r_ev, ev) in corridor_events
                if t_start - 10.0 <= t_ev <= t_start
            ]
            reservation_count = len(res_events_at_bid)
            
            # Queue: maximum concurrent distinct peer robots reserving any single corridor
            peer_res_by_cid = defaultdict(set)
            for (t_ev, cid, r_ev, ev) in res_events_at_bid:
                if r_ev != robot_id:
                    peer_res_by_cid[cid].add(r_ev)
            max_queue = max([len(peers) for peers in peer_res_by_cid.values()], default=0)

            # Corridor occupancy ratio: fraction of time other robots reserved candidate corridors in [t_start - 10, t_start]
            candidate_res_events = [t_ev for (t_ev, cid, r_ev, ev) in res_events_at_bid if r_ev != robot_id]
            corridor_occupancy_ratio = min(1.0, round(len(candidate_res_events) * 2.0 / 10.0, 3))

            # 5. Stop Time & Waiting Time (Execution outcome target / post-hoc metrics)
            my_timeline = [
                (ts, spd) for (ts, rx, ry, spd) in robot_states_timeline.get(robot_id, [])
                if t_start <= ts <= t_end
            ]
            total_stop_time = 0.0
            if len(my_timeline) >= 2:
                for idx in range(len(my_timeline) - 1):
                    dt = my_timeline[idx+1][0] - my_timeline[idx][0]
                    if my_timeline[idx][1] < 0.05:
                        total_stop_time += dt
            else:
                total_stop_time = max(0.0, duration - (path_len / max(0.5, nominal_speed)))

            total_stop_time = round(min(duration, total_stop_time), 2)
            p_dwell, d_dwell = task_dwells.get(tid, (1.0, 1.0))
            waiting_time = round(min(duration, total_stop_time + (p_dwell + d_dwell)), 2)

            # 6. Active Dynamic Blockage Count at Bid Time
            active_blockages = sum(
                1 for (t_obs, t_valid) in blockage_intervals
                if t_obs <= t_start <= t_valid
            )

            # 7. Mean Peer Telemetry Freshness at Bid Time (ms)
            intervals_ms = []
            for other_rid, t_stamps in robot_state_timestamps.items():
                if other_rid == robot_id:
                    continue
                bid_stamps = [ts for ts in t_stamps if t_start - 5.0 <= ts <= t_start]
                if len(bid_stamps) >= 2:
                    diffs = [(bid_stamps[i+1] - bid_stamps[i]) * 1000.0 for i in range(len(bid_stamps) - 1)]
                    intervals_ms.extend(diffs)

            mean_freshness = round(sum(intervals_ms) / len(intervals_ms), 1) if intervals_ms else 100.0
            mean_freshness = min(500.0, max(50.0, mean_freshness))

            # 8. Concurrent Task Load Count at Bid Time
            concurrent_tasks = sum(
                1 for (other_tid, (ot_start, _)) in task_starts.items()
                if other_tid != tid and ot_start <= t_start
                and (other_tid not in task_completions or task_completions[other_tid][0] >= t_start)
            )

            all_dataset_rows.append({
                'run_id': run_id,
                'task_id': tid,
                'robot_id': robot_id,
                'start_zone': start_zone,
                'goal_zone': goal_zone,
                'static_path_length_m': path_len,
                'turn_count': turns,
                'junction_crossings_count': junction_crossings,
                'nominal_speed_mps': nominal_speed,
                'fleet_size': fleet_size,
                'candidate_corridor_count': candidate_corridors,
                'mean_nearby_robot_count': mean_nearby,
                'avg_nearby_robot_speed_mps': avg_nearby_spd,
                'max_corridor_queue_length': max_queue,
                'reservation_count': reservation_count,
                'corridor_occupancy_ratio': corridor_occupancy_ratio,
                'active_blockage_count': active_blockages,
                'waiting_time_s': waiting_time,
                'total_stop_time_s': total_stop_time,
                'mean_peer_freshness_ms': mean_freshness,
                'task_load_count': concurrent_tasks,
                'actual_travel_time_s': round(duration, 2)
            })

    # Write output master dataset CSV
    if all_dataset_rows:
        headers = list(all_dataset_rows[0].keys())
        out_p = pathlib.Path(output_csv_path)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        with open(out_p, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(all_dataset_rows)
        print(f"\n========================================================")
        print(f"SUCCESS: Compiled {len(all_dataset_rows)} completed task rows from {total_runs_processed} runs ({total_files_processed} files)")
        print(f"Destination: {output_csv_path}")
        print(f"Columns ({len(headers)}): {', '.join(headers)}")
        print(f"========================================================")
    else:
        print("Warning: No completed tasks found in provided telemetry files.")


def main():
    parser = argparse.ArgumentParser(description='Generate Unified 22-Column Tabular ML Dataset.')
    parser.add_argument('telemetry_files', nargs='*', default=[], help='Paths or glob patterns to fleet_telemetry.jsonl files.')
    parser.add_argument('--output', '-o', default='collected_datasets.csv', help='Output master ML CSV path.')
    args = parser.parse_args()

    files = []
    if args.telemetry_files:
        for pattern in args.telemetry_files:
            matches = glob.glob(pattern, recursive=True) if '*' in pattern else [pattern]
            files.extend(matches)
    else:
        files = sorted(default_log_root().rglob('fleet_telemetry.jsonl'))
        files = [str(f) for f in files]

    if not files:
        print("No fleet_telemetry.jsonl files found.")
        sys.exit(1)

    print(f"Processing {len(files)} telemetry files...")
    parse_telemetry_to_dataset(files, args.output)


if __name__ == '__main__':
    main()
