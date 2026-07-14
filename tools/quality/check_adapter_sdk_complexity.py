#!/usr/bin/env python3
"""Enforce Adapter SDK function-complexity and source-size limits."""

import argparse
import csv
import io
from pathlib import Path
import subprocess
import sys


MAX_CCN = 15
MAX_FUNCTION_NLOC = 80
MAX_CPP_LINES = 500
ANALYZED_PATH = "core/mujoco_sim_adapter/mujoco_sim_adapter_sdk/src"


def parse_failures(contents: str) -> tuple[int, list[str]]:
    function_count = 0
    failures = []
    for row in csv.reader(io.StringIO(contents)):
        if len(row) < 10:
            continue
        function_count += 1
        nloc = int(row[0])
        ccn = int(row[1])
        if nloc > MAX_FUNCTION_NLOC or ccn > MAX_CCN:
            failures.append(
                f"{row[6]}: {row[8]}: NLOC={nloc}, CCN={ccn} exceeds "
                f"NLOC<={MAX_FUNCTION_NLOC}, CCN<={MAX_CCN}"
            )
    return function_count, failures


def find_source_size_failures(root: Path) -> list[str]:
    failures = []
    for path in sorted((root / ANALYZED_PATH).rglob("*.cpp")):
        line_count = len(path.read_text(encoding="utf-8").splitlines())
        if line_count > MAX_CPP_LINES:
            failures.append(
                f"{path.relative_to(root)}: {line_count} lines exceeds "
                f"{MAX_CPP_LINES}"
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lizard", default="lizard")
    parser.add_argument("--root", required=True, type=Path)
    args = parser.parse_args()

    completed = subprocess.run(
        [args.lizard, "--csv", ANALYZED_PATH],
        cwd=args.root,
        check=True,
        capture_output=True,
        text=True,
    )
    function_count, failures = parse_failures(completed.stdout)
    failures.extend(find_source_size_failures(args.root))
    if failures:
        print("Adapter SDK quality gate failed:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(
        f"Adapter SDK quality gate passed: {function_count} functions, "
        f"CCN<={MAX_CCN}, NLOC<={MAX_FUNCTION_NLOC}, "
        f".cpp lines<={MAX_CPP_LINES}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
