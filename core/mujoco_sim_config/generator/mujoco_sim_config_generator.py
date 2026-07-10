#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True

from mujoco_sim_config_codegen import SchemaError, load_schema, render


def _write_if_changed(path: Path, content: str) -> None:
    content = content.rstrip() + "\n"
    if path.exists() and path.read_text(encoding="utf-8") == content:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a C++ configuration parser from a YAML schema"
    )
    parser.add_argument("--schema", required=True, help="input schema YAML")
    parser.add_argument("--header", required=True, help="generated C++ header")
    parser.add_argument("--source", required=True, help="generated C++ source")
    parser.add_argument(
        "--header-include",
        help="include spelling used by the generated source (defaults to header basename)",
    )
    parser.add_argument(
        "--template-dir",
        default=str(Path(__file__).resolve().parent / "templates"),
        help="directory containing config.hpp.in and config.cpp.in",
    )
    return parser.parse_args()


def main() -> int:
    arguments = _arguments()
    header = Path(arguments.header).resolve()
    source = Path(arguments.source).resolve()
    header_include = arguments.header_include or header.name
    try:
        schema = load_schema(arguments.schema)
        header_text, source_text = render(
            schema, header_include, Path(arguments.template_dir).resolve()
        )
        _write_if_changed(header, header_text)
        _write_if_changed(source, source_text)
    except (SchemaError, OSError, UnicodeError) as error:
        print(f"mujoco_sim_config_generator: error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
