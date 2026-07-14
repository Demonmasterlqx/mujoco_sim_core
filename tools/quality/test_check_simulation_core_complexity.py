#!/usr/bin/env python3
"""Regression tests for the SimulationCore complexity gate."""

import unittest

from check_simulation_core_complexity import (
    find_function_failures,
    parse_report,
)


def report_row(
    *, nloc: int, ccn: int, path: str, name: str, signature: str, start: int
) -> str:
    return (
        f'{nloc},{ccn},0,0,{nloc},"{name}@{start}-{start + nloc}@{path}",'
        f'"{path}","{name}","{signature}",{start},{start + nloc}'
    )


class ComplexityGateTest(unittest.TestCase):
    def test_line_move_preserves_function_identity(self) -> None:
        baseline = parse_report(
            report_row(
                nloc=10,
                ccn=2,
                path="core/example.cpp",
                name="Example::Run",
                signature="Example::Run()",
                start=10,
            )
        )
        current = parse_report(
            report_row(
                nloc=10,
                ccn=2,
                path="core/example.cpp",
                name="Example::Run",
                signature="Example::Run()",
                start=100,
            )
        )

        self.assertEqual(baseline.keys(), current.keys())

    def test_regression_below_hard_limit_is_rejected(self) -> None:
        baseline = parse_report(
            report_row(
                nloc=10,
                ccn=2,
                path="core/example.cpp",
                name="Example::Run",
                signature="Example::Run()",
                start=10,
            )
        )
        current = parse_report(
            report_row(
                nloc=11,
                ccn=3,
                path="core/example.cpp",
                name="Example::Run",
                signature="Example::Run()",
                start=100,
            )
        )

        failures = find_function_failures(current, baseline)

        self.assertEqual(1, len(failures))
        self.assertIn("regressed", failures[0])

    def test_overloads_have_distinct_function_identity(self) -> None:
        report = "\n".join(
            (
                report_row(
                    nloc=10,
                    ccn=2,
                    path="core/example.cpp",
                    name="Example::Run",
                    signature="Example::Run(int)",
                    start=10,
                ),
                report_row(
                    nloc=12,
                    ccn=3,
                    path="core/example.cpp",
                    name="Example::Run",
                    signature="Example::Run(double)",
                    start=30,
                ),
            )
        )

        functions = parse_report(report)

        self.assertEqual(2, len(functions))


if __name__ == "__main__":
    unittest.main()
