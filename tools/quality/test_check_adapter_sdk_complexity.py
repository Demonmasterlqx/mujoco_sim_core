#!/usr/bin/env python3
"""Tests for the Adapter SDK complexity gate."""

from pathlib import Path
import tempfile
import unittest

from check_adapter_sdk_complexity import (
    ANALYZED_PATH,
    find_source_size_failures,
    parse_failures,
)


def report_row(*, nloc: int, ccn: int) -> str:
    return (
        f'{nloc},{ccn},0,0,{nloc},"Run@1-2@example.cpp",'
        f'"example.cpp","Run","Run()",1,2'
    )


class AdapterSdkComplexityGateTest(unittest.TestCase):
    def test_function_over_hard_limit_is_rejected(self) -> None:
        count, failures = parse_failures(report_row(nloc=81, ccn=16))
        self.assertEqual(count, 1)
        self.assertEqual(len(failures), 1)

    def test_source_over_line_limit_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / ANALYZED_PATH / "large.cpp"
            source.parent.mkdir(parents=True)
            source.write_text("line\n" * 501, encoding="utf-8")
            failures = find_source_size_failures(root)
        self.assertEqual(len(failures), 1)


if __name__ == "__main__":
    unittest.main()
