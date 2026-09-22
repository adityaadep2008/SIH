#!/usr/bin/env python3
"""Dataset Merger & Rebuilder Script.

Combines `collected_datasets_desktop.csv` and `collected_datasets_laptop.csv`
into `collected_datasets_combined.csv`. Recreates or overwrites the output
file upon execution.
"""

import argparse
import csv
import os
import sys
from pathlib import Path


def get_default_workspace() -> Path:
    script_dir = Path(__file__).resolve().parent
    return script_dir.parent


def combine_datasets(desktop_path: Path, laptop_path: Path, output_path: Path, dedup: bool = True) -> int:
    """Read desktop and laptop CSV datasets and write the combined dataset."""
    if not desktop_path.exists() and not laptop_path.exists():
        print(f"Error: Neither {desktop_path} nor {laptop_path} exists.", file=sys.stderr)
        return 1

    header = None
    all_rows = []
    seen_keys = set()
    desktop_count = 0
    laptop_count = 0
    dup_count = 0

    # 1. Process Desktop Dataset
    if desktop_path.exists():
        with open(desktop_path, mode="r", encoding="utf-8", newline="") as f:
            reader = csv.reader(f)
            try:
                header = next(reader)
            except StopIteration:
                pass
            if header:
                for row in reader:
                    if not row:
                        continue
                    desktop_count += 1
                    # (run_id, task_id, robot_id) as deduplication key
                    key = tuple(row[:3]) if len(row) >= 3 else tuple(row)
                    if dedup and key in seen_keys:
                        dup_count += 1
                        continue
                    if dedup:
                        seen_keys.add(key)
                    all_rows.append(row)
        print(f"Loaded {desktop_count} rows from desktop dataset: {desktop_path.name}")
    else:
        print(f"Notice: Desktop dataset not found at {desktop_path}")

    # 2. Process Laptop Dataset
    if laptop_path.exists():
        with open(laptop_path, mode="r", encoding="utf-8", newline="") as f:
            reader = csv.reader(f)
            try:
                lap_header = next(reader)
                if header is None:
                    header = lap_header
                elif lap_header != header:
                    print(
                        f"Warning: Laptop dataset header differs from desktop header!\n"
                        f"  Desktop: {header}\n"
                        f"  Laptop:  {lap_header}",
                        file=sys.stderr,
                    )
            except StopIteration:
                pass

            if header:
                for row in reader:
                    if not row:
                        continue
                    laptop_count += 1
                    key = tuple(row[:3]) if len(row) >= 3 else tuple(row)
                    if dedup and key in seen_keys:
                        dup_count += 1
                        continue
                    if dedup:
                        seen_keys.add(key)
                    all_rows.append(row)
        print(f"Loaded {laptop_count} rows from laptop dataset: {laptop_path.name}")
    else:
        print(f"Notice: Laptop dataset not found at {laptop_path}")

    if not header:
        print("Error: No valid CSV header found in input datasets.", file=sys.stderr)
        return 1

    # 3. Write Output Combined Dataset
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, mode="w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(all_rows)

    print(f"\nSuccessfully combined datasets into: {output_path.resolve()}")
    print(f"  • Total input rows: {desktop_count + laptop_count} ({desktop_count} desktop + {laptop_count} laptop)")
    if dedup and dup_count > 0:
        print(f"  • Deduplicated: {dup_count} duplicate rows removed")
    print(f"  • Final output rows: {len(all_rows)} rows (plus header)\n")
    return 0


def main() -> int:
    workspace = get_default_workspace()
    parser = argparse.ArgumentParser(
        description="Combine desktop and laptop benchmark datasets into a single combined CSV dataset."
    )
    parser.add_argument(
        "--desktop",
        type=Path,
        default=workspace / "collected_datasets_desktop.csv",
        help="Path to collected_datasets_desktop.csv (default: workspace root)",
    )
    parser.add_argument(
        "--laptop",
        type=Path,
        default=workspace / "collected_datasets_laptop.csv",
        help="Path to collected_datasets_laptop.csv (default: workspace root)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=workspace / "collected_datasets_combined.csv",
        help="Path to output collected_datasets_combined.csv (default: workspace root)",
    )
    parser.add_argument(
        "--no-dedup",
        action="store_true",
        default=False,
        help="Disable row deduplication by (run_id, task_id, robot_id)",
    )

    args = parser.parse_args()
    return combine_datasets(
        desktop_path=args.desktop,
        laptop_path=args.laptop,
        output_path=args.output,
        dedup=not args.no_dedup,
    )


if __name__ == "__main__":
    sys.exit(main())
