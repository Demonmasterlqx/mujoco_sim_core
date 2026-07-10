from __future__ import annotations

import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_DIR = PACKAGE_ROOT / "generator"
sys.path.insert(0, str(GENERATOR_DIR))

from mujoco_sim_config_codegen import SchemaError, load_schema, render  # noqa: E402


class GeneratorTest(unittest.TestCase):
    def test_loads_every_supported_shape(self) -> None:
        schema = load_schema(PACKAGE_ROOT / "test/schema/runtime.yaml")
        types = {field.type for field in schema.fields}
        self.assertEqual(
            types,
            {
                "string",
                "path",
                "bool",
                "uint32",
                "uint64",
                "double",
                "vector<string>",
                "array",
                "enum",
                "object",
                "object_sequence",
            },
        )
        workers = next(field for field in schema.fields if field.name == "workers")
        self.assertEqual([field.name for field in workers.fields], ["id", "label", "axes", "policy"])

    def test_renders_public_api_and_strict_runtime_calls(self) -> None:
        schema = load_schema(PACKAGE_ROOT / "test/schema/runtime.yaml")
        header, source = render(
            schema,
            "runtime_config.hpp",
            GENERATOR_DIR / "templates",
        )
        self.assertIn("struct RuntimeConfig", header)
        self.assertIn("bool Load(", header)
        self.assertIn("bool Parse(", header)
        self.assertIn("std::string Usage()", header)
        self.assertIn("runtime::LoadSingleMapping", source)
        self.assertIn("runtime::RegisterKey", source)
        self.assertIn("CLI::MultiOptionPolicy::Throw", source)
        self.assertIn("Validate_RuntimeConfig", source)

    def test_command_writes_both_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            header = Path(directory) / "generated.hpp"
            source = Path(directory) / "generated.cpp"
            result = subprocess.run(
                [
                    sys.executable,
                    str(GENERATOR_DIR / "mujoco_sim_config_generator.py"),
                    "--schema",
                    str(PACKAGE_ROOT / "test/schema/runtime.yaml"),
                    "--header",
                    str(header),
                    "--source",
                    str(source),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(header.is_file())
            self.assertTrue(source.is_file())
            self.assertIn('#include "generated.hpp"', source.read_text())

    def test_rejects_duplicate_unknown_and_invalid_defaults(self) -> None:
        invalid_schemas = [
            (
                "duplicate.yaml",
                """
                version: 1
                version: 1
                namespace: example
                name: Config
                program: app
                fields: {value: {type: string}}
                """,
                "duplicate key: version",
            ),
            (
                "unknown.yaml",
                """
                version: 1
                namespace: example
                name: Config
                program: app
                fields:
                  value:
                    type: string
                    typo: true
                """,
                "unknown field key: typo",
            ),
            (
                "overflow.yaml",
                """
                version: 1
                namespace: example
                name: Config
                program: app
                fields:
                  value: {type: uint32, default: 4294967296}
                """,
                "invalid default for type uint32",
            ),
            (
                "flags.yaml",
                """
                version: 1
                namespace: example
                name: Config
                program: app
                fields:
                  first: {type: string, cli: --value}
                  second: {type: string, cli: --value}
                """,
                "CLI flag --value is used by both",
            ),
        ]
        with tempfile.TemporaryDirectory() as directory:
            for filename, contents, expected in invalid_schemas:
                with self.subTest(filename=filename):
                    path = Path(directory) / filename
                    path.write_text(textwrap.dedent(contents).lstrip(), encoding="utf-8")
                    with self.assertRaisesRegex(SchemaError, expected):
                        load_schema(path)


if __name__ == "__main__":
    unittest.main()
