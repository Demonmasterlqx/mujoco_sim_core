#!/usr/bin/env python3
"""Reject SimulationCore complexity and source-size regressions."""

import argparse
import csv
import io
from pathlib import Path
import subprocess
import sys


MAX_CCN = 15
MAX_FUNCTION_NLOC = 80
MAX_CPP_LINES = 500
ANALYZED_PATHS = (
    "core/mujoco_sim_core/src",
    "core/mujoco_sim_core/include",
)

FunctionKey = tuple[str, str]
FunctionMetrics = tuple[int, int]


def parse_report(contents: str) -> dict[FunctionKey, FunctionMetrics]:
    functions: dict[FunctionKey, FunctionMetrics] = {}
    for row in csv.reader(io.StringIO(contents)):
        if len(row) < 10:
            continue
        # Lizard's stable identity is the source path plus full signature.
        # The start line is deliberately excluded because unrelated edits above
        # a function must not make its baseline record disappear.
        key = (row[6], row[8])
        functions[key] = (int(row[0]), int(row[1]))
    return functions


def find_function_failures(
    current: dict[FunctionKey, FunctionMetrics],
    baseline: dict[FunctionKey, FunctionMetrics],
) -> list[str]:
    failures = []
    for key, (nloc, ccn) in sorted(current.items()):
        path, signature = key
        if nloc > MAX_FUNCTION_NLOC or ccn > MAX_CCN:
            failures.append(
                f"{path}: {signature}: NLOC={nloc}, CCN={ccn} exceeds "
                f"NLOC<={MAX_FUNCTION_NLOC}, CCN<={MAX_CCN}"
            )
        previous = baseline.get(key)
        if previous and (nloc > previous[0] or ccn > previous[1]):
            failures.append(
                f"{path}: {signature}: regressed from NLOC={previous[0]}, "
                f"CCN={previous[1]} to NLOC={nloc}, CCN={ccn}"
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lizard", default="lizard")
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--root", required=True, type=Path)
    args = parser.parse_args()

    baseline = parse_report(args.baseline.read_text(encoding="utf-8"))
    completed = subprocess.run(
        [args.lizard, "--csv", *ANALYZED_PATHS],
        cwd=args.root,
        check=True,
        capture_output=True,
        text=True,
    )
    current = parse_report(completed.stdout)
    failures = find_function_failures(current, baseline)

    source_dir = args.root / "core/mujoco_sim_core/src"
    for path in sorted(source_dir.rglob("*.cpp")):
        line_count = len(path.read_text(encoding="utf-8").splitlines())
        if line_count > MAX_CPP_LINES:
            failures.append(
                f"{path.relative_to(args.root)}: {line_count} lines exceeds "
                f"{MAX_CPP_LINES}"
            )

    if failures:
        print("SimulationCore quality gate failed:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(
        f"SimulationCore quality gate passed: {len(current)} functions, "
        f"CCN<={MAX_CCN}, NLOC<={MAX_FUNCTION_NLOC}, "
        f".cpp lines<={MAX_CPP_LINES}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
