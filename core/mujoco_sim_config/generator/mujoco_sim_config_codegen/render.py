from __future__ import annotations

import math
import re
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path
from typing import Any

from .schema import Field, MISSING, Schema, SchemaError


CPP_TYPES = {
    "string": "std::string",
    "path": "std::filesystem::path",
    "bool": "bool",
    "uint32": "std::uint32_t",
    "uint64": "std::uint64_t",
    "double": "double",
    "vector<string>": "std::vector<std::string>",
}


def _pascal(value: str) -> str:
    parts = [part for part in re.split(r"[^A-Za-z0-9]+", value) if part]
    result = "".join(part[0].upper() + part[1:] for part in parts)
    return result or "Value"


def _enum_identifiers(values: tuple[str, ...], source: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    used: set[str] = set()
    for value in values:
        identifier = "k" + _pascal(value)
        if identifier in used:
            raise SchemaError(
                f"{source}: enum values produce the same C++ identifier: {identifier}"
            )
        used.add(identifier)
        result[value] = identifier
    return result


def _cpp_string(value: str) -> str:
    escaped: list[str] = ['"']
    for character in value:
        code = ord(character)
        if character == "\\":
            escaped.append("\\\\")
        elif character == '"':
            escaped.append('\\"')
        elif character == "\n":
            escaped.append("\\n")
        elif character == "\r":
            escaped.append("\\r")
        elif character == "\t":
            escaped.append("\\t")
        elif code < 32 or code == 127:
            escaped.append(f"\\x{code:02x}")
        else:
            escaped.append(character)
    escaped.append('"')
    return "".join(escaped)


def _cpp_number(value: int | float) -> str:
    if isinstance(value, int):
        if value > 0x7FFFFFFFFFFFFFFF:
            return f"{value}ULL"
        return str(value)
    if math.isnan(value):
        return "std::numeric_limits<double>::quiet_NaN()"
    if math.isinf(value):
        prefix = "-" if value < 0 else ""
        return f"{prefix}std::numeric_limits<double>::infinity()"
    result = repr(value)
    if "e" not in result.lower() and "." not in result:
        result += ".0"
    return result


def _message_number(value: int | float) -> str:
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


@dataclass
class ObjectModel:
    fields: list[Field]
    cpp_type: str
    function_id: str
    path: tuple[str, ...]
    children: dict[str, "ObjectModel"] = dataclass_field(default_factory=dict)

    @property
    def presence_type(self) -> str:
        return f"Presence_{self.function_id}"


@dataclass(frozen=True)
class CliField:
    model: ObjectModel
    field: Field
    ancestors: tuple[Field, ...]

    @property
    def variable(self) -> str:
        return "cli_" + "__".join(self.field.path)


class Renderer:
    def __init__(self, schema: Schema, header_include: str):
        self.schema = schema
        self.header_include = header_include
        self.models: list[ObjectModel] = []
        self.root = self._build_model(
            schema.fields, schema.name, schema.name, (), append=True
        )
        self.cli_fields: list[CliField] = []
        self._collect_cli(self.root, ())
        self._validate_enums()

    def _build_model(
        self,
        fields: list[Field],
        cpp_type: str,
        function_id: str,
        path: tuple[str, ...],
        append: bool,
    ) -> ObjectModel:
        model = ObjectModel(fields, cpp_type, function_id, path)
        if append:
            self.models.append(model)
        for field_value in fields:
            if field_value.type == "object":
                local = _pascal(field_value.name)
                child = self._build_model(
                    field_value.fields,
                    f"{cpp_type}::{local}",
                    f"{function_id}_{local}",
                    field_value.path,
                    append=True,
                )
                model.children[field_value.name] = child
            elif field_value.type == "object_sequence":
                local = _pascal(field_value.name) + "Item"
                child = self._build_model(
                    field_value.fields,
                    f"{cpp_type}::{local}",
                    f"{function_id}_{local}",
                    field_value.path + ("item",),
                    append=True,
                )
                model.children[field_value.name] = child
        return model

    def _collect_cli(
        self, model: ObjectModel, ancestors: tuple[Field, ...]
    ) -> None:
        for field_value in model.fields:
            if field_value.type == "object":
                self._collect_cli(
                    model.children[field_value.name], ancestors + (field_value,)
                )
            elif field_value.type == "object_sequence":
                nested_cli = [
                    nested
                    for nested in self._walk_fields(field_value.fields)
                    if nested.cli.flags or nested.cli.inverse_flags
                ]
                if nested_cli:
                    raise SchemaError(
                        f"{self.schema.source}: fields inside object_sequence cannot have CLI flags"
                    )
            elif field_value.cli.flags or field_value.cli.inverse_flags:
                self.cli_fields.append(CliField(model, field_value, ancestors))

    @staticmethod
    def _walk_fields(fields: list[Field]):
        for field_value in fields:
            yield field_value
            yield from Renderer._walk_fields(field_value.fields)

    def _validate_enums(self) -> None:
        for field_value in self._walk_fields(self.schema.fields):
            if field_value.type == "enum":
                _enum_identifiers(field_value.enum_values, self.schema.source)
            if field_value.type == "array" and field_value.item_type == "enum":
                _enum_identifiers(field_value.item_enum_values, self.schema.source)

    def _local_type(self, model: ObjectModel, field_value: Field) -> str:
        if field_value.type in CPP_TYPES:
            return CPP_TYPES[field_value.type]
        if field_value.type == "enum":
            return _pascal(field_value.name)
        if field_value.type == "object":
            return _pascal(field_value.name)
        if field_value.type == "object_sequence":
            return f"std::vector<{_pascal(field_value.name)}Item>"
        if field_value.type == "array":
            item_type = (
                _pascal(field_value.name) + "Item"
                if field_value.item_type == "enum"
                else CPP_TYPES[field_value.item_type or ""]
            )
            return f"std::array<{item_type}, {field_value.size}>"
        raise AssertionError(field_value.type)

    def _qualified_type(self, model: ObjectModel, field_value: Field) -> str:
        if field_value.type in CPP_TYPES:
            return CPP_TYPES[field_value.type]
        if field_value.type == "enum":
            return f"{model.cpp_type}::{_pascal(field_value.name)}"
        if field_value.type == "object":
            return model.children[field_value.name].cpp_type
        if field_value.type == "object_sequence":
            return f"std::vector<{model.children[field_value.name].cpp_type}>"
        if field_value.type == "array":
            item_type = (
                f"{model.cpp_type}::{_pascal(field_value.name)}Item"
                if field_value.item_type == "enum"
                else CPP_TYPES[field_value.item_type or ""]
            )
            return f"std::array<{item_type}, {field_value.size}>"
        raise AssertionError(field_value.type)

    def _enum_type(
        self, model: ObjectModel, field_value: Field, qualified: bool, item: bool
    ) -> str:
        suffix = _pascal(field_value.name) + ("Item" if item else "")
        return f"{model.cpp_type}::{suffix}" if qualified else suffix

    def _scalar_expression(
        self,
        scalar_type: str,
        value: Any,
        enum_values: tuple[str, ...],
        enum_type: str | None,
    ) -> str:
        if scalar_type == "string":
            return _cpp_string(value)
        if scalar_type == "path":
            return f"std::filesystem::path{{{_cpp_string(value)}}}"
        if scalar_type == "bool":
            return "true" if value else "false"
        if scalar_type == "uint32":
            return f"std::uint32_t{{{value}U}}"
        if scalar_type == "uint64":
            return f"std::uint64_t{{{value}ULL}}"
        if scalar_type == "double":
            return _cpp_number(value)
        if scalar_type == "enum":
            identifiers = _enum_identifiers(enum_values, self.schema.source)
            return f"{enum_type}::{identifiers[value]}"
        raise AssertionError(scalar_type)

    def _default_expression(
        self,
        model: ObjectModel,
        field_value: Field,
        value: Any,
        qualified: bool,
    ) -> str:
        field_type = (
            self._qualified_type(model, field_value)
            if qualified
            else self._local_type(model, field_value)
        )
        if value is MISSING:
            return f"{field_type}{{}}"
        if field_value.type in {
            "string",
            "path",
            "bool",
            "uint32",
            "uint64",
            "double",
            "enum",
        }:
            enum_type = (
                self._enum_type(model, field_value, qualified, False)
                if field_value.type == "enum"
                else None
            )
            return self._scalar_expression(
                field_value.type, value, field_value.enum_values, enum_type
            )
        if field_value.type == "vector<string>":
            return f"{field_type}{{{', '.join(_cpp_string(item) for item in value)}}}"
        if field_value.type == "array":
            enum_type = (
                self._enum_type(model, field_value, qualified, True)
                if field_value.item_type == "enum"
                else None
            )
            items = [
                self._scalar_expression(
                    field_value.item_type or "",
                    item,
                    field_value.item_enum_values,
                    enum_type,
                )
                for item in value
            ]
            return f"{field_type}{{{{{', '.join(items)}}}}}"
        if field_value.type == "object":
            child_model = model.children[field_value.name]
            items = []
            for child in field_value.fields:
                child_value = value.get(child.name, child.default)
                items.append(
                    self._default_expression(
                        child_model, child, child_value, qualified
                    )
                )
            nested_type = (
                child_model.cpp_type if qualified else _pascal(field_value.name)
            )
            return f"{nested_type}{{{', '.join(items)}}}"
        if field_value.type == "object_sequence":
            item_model = model.children[field_value.name]
            item_type = (
                item_model.cpp_type
                if qualified
                else _pascal(field_value.name) + "Item"
            )
            items = []
            for item in value:
                child_values = []
                for child in field_value.fields:
                    child_value = item.get(child.name, child.default)
                    child_values.append(
                        self._default_expression(
                            item_model, child, child_value, qualified
                        )
                    )
                items.append(f"{item_type}{{{', '.join(child_values)}}}")
            return f"{field_type}{{{', '.join(items)}}}"
        raise AssertionError(field_value.type)

    def _render_enum(
        self, name: str, values: tuple[str, ...], indentation: str
    ) -> list[str]:
        identifiers = _enum_identifiers(values, self.schema.source)
        lines = [f"{indentation}enum class {name} {{"]
        for value in values:
            lines.append(f"{indentation}  {identifiers[value]},")
        lines.append(f"{indentation}}};")
        return lines

    def _render_struct(self, model: ObjectModel, local_name: str, level: int) -> list[str]:
        indent = "  " * level
        lines = [f"{indent}struct {local_name} {{"]
        nested: list[str] = []
        for field_value in model.fields:
            if field_value.type == "enum":
                nested.extend(
                    self._render_enum(
                        _pascal(field_value.name),
                        field_value.enum_values,
                        indent + "  ",
                    )
                )
            elif field_value.type == "array" and field_value.item_type == "enum":
                nested.extend(
                    self._render_enum(
                        _pascal(field_value.name) + "Item",
                        field_value.item_enum_values,
                        indent + "  ",
                    )
                )
            elif field_value.type in {"object", "object_sequence"}:
                child = model.children[field_value.name]
                child_name = _pascal(field_value.name) + (
                    "Item" if field_value.type == "object_sequence" else ""
                )
                nested.extend(self._render_struct(child, child_name, level + 1))
        if nested:
            lines.extend(nested)
            lines.append("")
        for field_value in model.fields:
            field_type = self._local_type(model, field_value)
            if field_value.default is MISSING:
                lines.append(f"{indent}  {field_type} {field_value.name}{{}};")
            else:
                default = self._default_expression(
                    model, field_value, field_value.default, qualified=False
                )
                lines.append(f"{indent}  {field_type} {field_value.name} = {default};")
        lines.append(f"{indent}}};")
        return lines

    def render_header_declarations(self) -> str:
        lines = self._render_struct(self.root, self.schema.name, 0)
        lines.extend(
            [
                "",
                f"bool Load(const std::filesystem::path& path, {self.schema.name}* config,",
                "          std::string* error_message);",
                f"bool Parse(int argc, const char* const argv[], {self.schema.name}* config,",
                "           std::string* error_message);",
                "std::string Usage();",
            ]
        )
        return "\n".join(lines)

    def _presence_definitions(self) -> str:
        lines = [
            "struct Origin {",
            "  bool provided = false;",
            "  YAML::Mark mark = YAML::Mark::null_mark();",
            "};",
            "",
        ]
        for model in reversed(self.models):
            lines.append(f"struct {model.presence_type} {{")
            lines.append("  Origin self;")
            for field_value in model.fields:
                lines.append(f"  Origin {field_value.name};")
                if field_value.type == "object":
                    child = model.children[field_value.name]
                    lines.append(
                        f"  {child.presence_type} {field_value.name}_values;"
                    )
                elif field_value.type == "object_sequence":
                    child = model.children[field_value.name]
                    lines.append(
                        f"  std::vector<{child.presence_type}> {field_value.name}_items;"
                    )
            lines.append("};")
            lines.append("")
        return "\n".join(lines).rstrip()

    @staticmethod
    def _origin_yaml(field_name: str, indentation: str = "      ") -> list[str]:
        return [
            f"{indentation}presence->{field_name}.provided = true;",
            f"{indentation}presence->{field_name}.mark = entry.second.Mark();",
        ]

    def _enum_parse_lines(
        self,
        model: ObjectModel,
        field_value: Field,
        node_expression: str,
        target_expression: str,
        field_path_expression: str,
        indentation: str,
        item: bool = False,
    ) -> list[str]:
        values = (
            field_value.item_enum_values if item else field_value.enum_values
        )
        enum_type = self._enum_type(model, field_value, True, item)
        identifiers = _enum_identifiers(values, self.schema.source)
        suffix = "_item" if item else ""
        temporary = f"parsed_{field_value.name}{suffix}"
        lines = [f"{indentation}std::string {temporary};"]
        lines.extend(
            [
                f"{indentation}if (!runtime::ParseString(source, {node_expression},",
                f"{indentation}                          {field_path_expression}, &{temporary}, error_message)) {{",
                f"{indentation}  return false;",
                f"{indentation}}}",
            ]
        )
        for index, value in enumerate(values):
            keyword = "if" if index == 0 else "else if"
            lines.append(
                f"{indentation}{keyword} ({temporary} == {_cpp_string(value)}) {{"
            )
            lines.append(
                f"{indentation}  {target_expression} = {enum_type}::{identifiers[value]};"
            )
            lines.append(f"{indentation}}}")
        allowed = ", ".join(values)
        lines.extend(
            [
                f"{indentation}else {{",
                f"{indentation}  return runtime::ErrorAt(",
                f"{indentation}      source, {node_expression}.Mark(),",
                f"{indentation}      std::string{{{field_path_expression}}} + {_cpp_string(' must be one of: ' + allowed)},",
                f"{indentation}      error_message);",
                f"{indentation}}}",
            ]
        )
        return lines

    def _parse_scalar_lines(
        self,
        model: ObjectModel,
        field_value: Field,
        scalar_type: str,
        node: str,
        target: str,
        label: str,
        indentation: str,
        enum_item: bool = False,
    ) -> list[str]:
        if scalar_type == "enum":
            return self._enum_parse_lines(
                model,
                field_value,
                node,
                target,
                label,
                indentation,
                item=enum_item,
            )
        function = {
            "string": "ParseString",
            "path": "ParsePath",
            "bool": "ParseBool",
            "uint32": "ParseUInt32",
            "uint64": "ParseUInt64",
            "double": "ParseDouble",
        }[scalar_type]
        lines = [
            f"{indentation}if (!runtime::{function}(source, {node}, {label},",
            f"{indentation}                    &{target}, error_message)) {{",
            f"{indentation}  return false;",
            f"{indentation}}}",
        ]
        if scalar_type == "path":
            lines.extend(
                [
                    f"{indentation}if (!{target}.empty() && {target}.is_relative()) {{",
                    f"{indentation}  {target} = std::filesystem::weakly_canonical(",
                    f"{indentation}      std::filesystem::absolute(source).parent_path() / {target});",
                    f"{indentation}}}",
                ]
            )
        return lines

    def _parse_field_body(self, model: ObjectModel, field_value: Field) -> list[str]:
        lines = self._origin_yaml(field_value.name)
        target = f"value->{field_value.name}"
        if field_value.type in {
            "string",
            "path",
            "bool",
            "uint32",
            "uint64",
            "double",
            "enum",
        }:
            lines.extend(
                self._parse_scalar_lines(
                    model,
                    field_value,
                    field_value.type,
                    "entry.second",
                    target,
                    "field_path",
                    "      ",
                )
            )
        elif field_value.type == "vector<string>":
            lines.extend(
                [
                    "      if (!runtime::RequireSequence(source, entry.second, field_path,",
                    "                                    error_message)) {",
                    "        return false;",
                    "      }",
                    "      std::vector<std::string> parsed;",
                    "      parsed.reserve(entry.second.size());",
                    "      for (std::size_t index = 0; index < entry.second.size(); ++index) {",
                    "        std::string item;",
                    "        const std::string item_path =",
                    "            field_path + \"[\" + std::to_string(index) + \"]\";",
                    "        if (!runtime::ParseString(source, entry.second[index], item_path,",
                    "                                  &item, error_message)) {",
                    "          return false;",
                    "        }",
                    "        parsed.push_back(std::move(item));",
                    "      }",
                    f"      {target} = std::move(parsed);",
                ]
            )
        elif field_value.type == "array":
            lines.extend(
                [
                    "      if (!runtime::RequireSequence(source, entry.second, field_path,",
                    "                                    error_message)) {",
                    "        return false;",
                    "      }",
                    f"      if (entry.second.size() != {field_value.size}) {{",
                    "        return runtime::ErrorAt(",
                    "            source, entry.second.Mark(),",
                    f"            field_path + {_cpp_string(f' must contain exactly {field_value.size} items')},",
                    "            error_message);",
                    "      }",
                    f"      for (std::size_t index = 0; index < {field_value.size}; ++index) {{",
                    "        const std::string item_path =",
                    "            field_path + \"[\" + std::to_string(index) + \"]\";",
                ]
            )
            lines.extend(
                self._parse_scalar_lines(
                    model,
                    field_value,
                    field_value.item_type or "",
                    "entry.second[index]",
                    f"{target}[index]",
                    "item_path",
                    "        ",
                    enum_item=True,
                )
            )
            lines.append("      }")
        elif field_value.type == "object":
            child = model.children[field_value.name]
            lines.extend(
                [
                    f"      if (!Parse_{child.function_id}(source, entry.second, &{target},",
                    f"                              &presence->{field_value.name}_values, field_path,",
                    "                              error_message)) {",
                    "        return false;",
                    "      }",
                ]
            )
        elif field_value.type == "object_sequence":
            child = model.children[field_value.name]
            item_type = child.cpp_type
            lines.extend(
                [
                    "      if (!runtime::RequireSequence(source, entry.second, field_path,",
                    "                                    error_message)) {",
                    "        return false;",
                    "      }",
                    f"      std::vector<{item_type}> parsed;",
                    f"      std::vector<{child.presence_type}> parsed_presence;",
                    "      parsed.reserve(entry.second.size());",
                    "      parsed_presence.reserve(entry.second.size());",
                    "      for (std::size_t index = 0; index < entry.second.size(); ++index) {",
                    f"        {item_type} item{{}};",
                    f"        {child.presence_type} item_presence;",
                    "        const std::string item_path =",
                    "            field_path + \"[\" + std::to_string(index) + \"]\";",
                    f"        if (!Parse_{child.function_id}(source, entry.second[index], &item,",
                    "                                &item_presence, item_path, error_message)) {",
                    "          return false;",
                    "        }",
                    "        parsed.push_back(std::move(item));",
                    "        parsed_presence.push_back(std::move(item_presence));",
                    "      }",
                    f"      {target} = std::move(parsed);",
                    f"      presence->{field_value.name}_items = std::move(parsed_presence);",
                ]
            )
        else:
            raise AssertionError(field_value.type)
        lines.append("      continue;")
        return lines

    def _parse_functions(self) -> str:
        blocks: list[str] = []
        for model in reversed(self.models):
            lines = [
                f"bool Parse_{model.function_id}(const std::filesystem::path& source,",
                f"{' ' * (12 + len(model.function_id))}const YAML::Node& node, {model.cpp_type}* value,",
                f"{' ' * (12 + len(model.function_id))}{model.presence_type}* presence,",
                f"{' ' * (12 + len(model.function_id))}const std::string& prefix,",
                f"{' ' * (12 + len(model.function_id))}std::string* error_message) {{",
                "  if (!runtime::RequireMapping(source, node,",
                "                               prefix.empty() ? \"configuration\" : prefix,",
                "                               error_message)) {",
                "    return false;",
                "  }",
                "  presence->self.provided = true;",
                "  presence->self.mark = node.Mark();",
                "  std::unordered_set<std::string> seen;",
                "  for (const auto& entry : node) {",
                "    std::string key;",
                "    if (!runtime::ReadMappingKey(source, entry.first, &key, error_message) ||",
                "        !runtime::RegisterKey(source, entry.first, key, &seen, error_message)) {",
                "      return false;",
                "    }",
                "    const std::string field_path =",
                "        prefix.empty() ? key : prefix + \".\" + key;",
            ]
            for field_value in model.fields:
                lines.append(f"    if (key == {_cpp_string(field_value.name)}) {{")
                lines.extend(self._parse_field_body(model, field_value))
                lines.append("    }")
            lines.extend(
                [
                    "    return runtime::ErrorAt(source, entry.first.Mark(),",
                    "                            \"unknown field: \" + field_path,",
                    "                            error_message);",
                    "  }",
                    "  return true;",
                    "}",
                ]
            )
            blocks.append("\n".join(lines))
        return "\n\n".join(blocks)

    @staticmethod
    def _constraint_error(
        origin: str, message_expression: str, indentation: str = "    "
    ) -> list[str]:
        return [
            f"{indentation}return ConstraintError(source, {origin},",
            f"{indentation}                       {message_expression}, error_message);",
        ]

    def _validation_constraints(
        self, field_value: Field, value: str, origin: str, field_path: str
    ) -> list[str]:
        lines: list[str] = []
        if field_value.non_empty:
            lines.append(f"  if ({value}.empty()) {{")
            lines.extend(
                self._constraint_error(
                    origin, f"{field_path} + {_cpp_string(' must be non-empty')}"
                )
            )
            lines.append("  }")
        if field_value.size is not None and field_value.type != "array":
            size_value = f"{value}.native()" if field_value.type == "path" else value
            lines.append(f"  if ({size_value}.size() != {field_value.size}) {{")
            lines.extend(
                self._constraint_error(
                    origin,
                    f"{field_path} + {_cpp_string(f' must contain exactly {field_value.size} items')}",
                )
            )
            lines.append("  }")

        numeric_array = field_value.type == "array"
        numeric_value = "item" if numeric_array else value
        if field_value.minimum is not None or field_value.maximum is not None or field_value.finite:
            if numeric_array:
                lines.append(f"  for (const auto& item : {value}) {{")
                indent = "  "
            else:
                indent = ""
            if field_value.finite:
                lines.append(f"  {indent}if (!std::isfinite({numeric_value})) {{")
                lines.extend(
                    self._constraint_error(
                        origin,
                        f"{field_path} + {_cpp_string(' must be finite')}",
                        "    " + indent,
                    )
                )
                lines.append(f"  {indent}}}")
            if field_value.minimum is not None:
                lines.append(
                    f"  {indent}if ({numeric_value} < {_cpp_number(field_value.minimum)}) {{"
                )
                lines.extend(
                    self._constraint_error(
                        origin,
                        f"{field_path} + {_cpp_string(f' must be >= {_message_number(field_value.minimum)}')}",
                        "    " + indent,
                    )
                )
                lines.append(f"  {indent}}}")
            if field_value.maximum is not None:
                lines.append(
                    f"  {indent}if ({numeric_value} > {_cpp_number(field_value.maximum)}) {{"
                )
                lines.extend(
                    self._constraint_error(
                        origin,
                        f"{field_path} + {_cpp_string(f' must be <= {_message_number(field_value.maximum)}')}",
                        "    " + indent,
                    )
                )
                lines.append(f"  {indent}}}")
            if numeric_array:
                lines.append("  }")
        if field_value.unique:
            lines.extend(
                [
                    f"  for (std::size_t left = 0; left < {value}.size(); ++left) {{",
                    f"    for (std::size_t right = left + 1; right < {value}.size(); ++right) {{",
                    f"      if ({value}[left] == {value}[right]) {{",
                ]
            )
            lines.extend(
                self._constraint_error(
                    origin,
                    f"{field_path} + {_cpp_string(' must contain unique items')}",
                    "        ",
                )
            )
            lines.extend(["      }", "    }", "  }"])
        return lines

    def _validation_functions(self) -> str:
        blocks: list[str] = []
        for model in reversed(self.models):
            lines = [
                f"bool Validate_{model.function_id}(const {model.cpp_type}& value,",
                f"{' ' * (14 + len(model.function_id))}const {model.presence_type}& presence,",
                f"{' ' * (14 + len(model.function_id))}const std::filesystem::path& source,",
                f"{' ' * (14 + len(model.function_id))}const std::string& prefix,",
                f"{' ' * (14 + len(model.function_id))}std::string* error_message) {{",
            ]
            for field_value in model.fields:
                path_var = f"field_path_{field_value.name}"
                lines.extend(
                    [
                        f"  const std::string {path_var} =",
                        f"      prefix.empty() ? {_cpp_string(field_value.name)} : prefix + {_cpp_string('.' + field_value.name)};",
                    ]
                )
                if field_value.required:
                    lines.append(f"  if (!presence.{field_value.name}.provided) {{")
                    lines.extend(
                        self._constraint_error(
                            "presence.self",
                            f"{path_var} + {_cpp_string(' is required')}",
                        )
                    )
                    lines.append("  }")
                lines.extend(
                    self._validation_constraints(
                        field_value,
                        f"value.{field_value.name}",
                        f"presence.{field_value.name}",
                        path_var,
                    )
                )
                if field_value.type == "object":
                    child = model.children[field_value.name]
                    lines.extend(
                        [
                            f"  if (presence.{field_value.name}.provided &&",
                            f"      !Validate_{child.function_id}(value.{field_value.name},",
                            f"                              presence.{field_value.name}_values, source,",
                            f"                              {path_var}, error_message)) {{",
                            "    return false;",
                            "  }",
                        ]
                    )
                elif field_value.type == "object_sequence":
                    child = model.children[field_value.name]
                    lines.extend(
                        [
                            f"  if (presence.{field_value.name}.provided) {{",
                            f"    for (std::size_t index = 0; index < value.{field_value.name}.size(); ++index) {{",
                            "      const std::string item_path =",
                            f"          {path_var} + \"[\" + std::to_string(index) + \"]\";",
                            f"      if (!Validate_{child.function_id}(value.{field_value.name}[index],",
                            f"                              presence.{field_value.name}_items[index], source,",
                            "                              item_path, error_message)) {",
                            "        return false;",
                            "      }",
                            "    }",
                            "  }",
                        ]
                    )
            lines.extend(["  return true;", "}"])
            blocks.append("\n".join(lines))
        return "\n\n".join(blocks)

    @staticmethod
    def _presence_parent_expression(ancestors: tuple[Field, ...]) -> tuple[str, str]:
        expression = "presence"
        separator = "->"
        for ancestor in ancestors:
            expression += f"{separator}{ancestor.name}_values"
            separator = "."
        return expression, separator

    @staticmethod
    def _value_expression(entry: CliField) -> str:
        parts = [ancestor.name for ancestor in entry.ancestors] + [entry.field.name]
        return "value->" + ".".join(parts)

    def _origin_expression(self, entry: CliField) -> str:
        parent, separator = self._presence_parent_expression(entry.ancestors)
        return f"{parent}{separator}{entry.field.name}"

    def _mark_cli_lines(self, entry: CliField, indentation: str) -> list[str]:
        lines: list[str] = []
        parent = "presence"
        separator = "->"
        for ancestor in entry.ancestors:
            origin = f"{parent}{separator}{ancestor.name}"
            lines.extend(
                [
                    f"{indentation}{origin}.provided = true;",
                    f"{indentation}{origin}.mark = YAML::Mark::null_mark();",
                ]
            )
            parent = f"{parent}{separator}{ancestor.name}_values"
            separator = "."
        origin = f"{parent}{separator}{entry.field.name}"
        lines.extend(
            [
                f"{indentation}{origin}.provided = true;",
                f"{indentation}{origin}.mark = YAML::Mark::null_mark();",
            ]
        )
        return lines

    def _overrides_definition(self) -> str:
        lines = ["struct Overrides {", "  std::optional<std::string> config_path;"]
        for entry in self.cli_fields:
            field_value = entry.field
            if field_value.type == "bool":
                if field_value.cli.flags:
                    lines.append(f"  bool {entry.variable}_true = false;")
                if field_value.cli.inverse_flags:
                    lines.append(f"  bool {entry.variable}_false = false;")
            elif field_value.type in {"vector<string>", "array"}:
                lines.append(f"  std::vector<std::string> {entry.variable};")
            else:
                lines.append(f"  std::optional<std::string> {entry.variable};")
        lines.append("};")
        return "\n".join(lines)

    @staticmethod
    def _flag_names(flags: tuple[str, ...]) -> str:
        return ",".join(flags)

    def _configure_cli(self) -> str:
        lines = [
            "void ConfigureCli(CLI::App* app, Overrides* overrides) {",
            "  app->set_help_flag(\"-h,--help\", \"Show this help message and exit\");",
            f"  app->add_option({_cpp_string(self._flag_names(self.schema.config.flags))},",
            "                  overrides->config_path,",
            f"                  {_cpp_string(self.schema.config.description)})",
            "      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);",
        ]
        for entry in self.cli_fields:
            field_value = entry.field
            variable = entry.variable
            description = _cpp_string(field_value.description)
            if field_value.type == "bool":
                option_names: list[str] = []
                if field_value.cli.flags:
                    option_name = f"option_{variable}_true"
                    lines.extend(
                        [
                            f"  CLI::Option* {option_name} =",
                            f"      app->add_flag({_cpp_string(self._flag_names(field_value.cli.flags))},",
                            f"                    overrides->{variable}_true, {description})",
                            "          ->multi_option_policy(CLI::MultiOptionPolicy::Throw);",
                        ]
                    )
                    option_names.append(option_name)
                if field_value.cli.inverse_flags:
                    option_name = f"option_{variable}_false"
                    lines.extend(
                        [
                            f"  CLI::Option* {option_name} =",
                            f"      app->add_flag({_cpp_string(self._flag_names(field_value.cli.inverse_flags))},",
                            f"                    overrides->{variable}_false, {description})",
                            "          ->multi_option_policy(CLI::MultiOptionPolicy::Throw);",
                        ]
                    )
                    option_names.append(option_name)
                if len(option_names) == 2:
                    lines.extend(
                        [
                            f"  {option_names[0]}->excludes({option_names[1]});",
                            f"  {option_names[1]}->excludes({option_names[0]});",
                        ]
                    )
                continue

            option_name = f"option_{variable}"
            lines.extend(
                [
                    f"  CLI::Option* {option_name} =",
                    f"      app->add_option({_cpp_string(self._flag_names(field_value.cli.flags))},",
                    f"                      overrides->{variable}, {description});",
                ]
            )
            if field_value.type == "vector<string>":
                lines.append(
                    f"  {option_name}->delimiter(',')->expected(-1)->multi_option_policy(CLI::MultiOptionPolicy::TakeAll);"
                )
            elif field_value.type == "array":
                lines.append(
                    f"  {option_name}->expected({field_value.size})->multi_option_policy(CLI::MultiOptionPolicy::Throw);"
                )
            else:
                lines.append(
                    f"  {option_name}->multi_option_policy(CLI::MultiOptionPolicy::Throw);"
                )
            if field_value.default is not MISSING:
                display = self._display_default(field_value.default)
                lines.append(f"  {option_name}->default_str({_cpp_string(display)});")
        lines.append("}")
        return "\n".join(lines)

    @staticmethod
    def _display_default(value: Any) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, list):
            return "[" + ", ".join(str(item) for item in value) + "]"
        return str(value)

    def _cli_enum_lines(
        self,
        entry: CliField,
        source_expression: str,
        target_expression: str,
        indentation: str,
        item: bool = False,
    ) -> list[str]:
        field_value = entry.field
        values = field_value.item_enum_values if item else field_value.enum_values
        enum_type = self._enum_type(entry.model, field_value, True, item)
        identifiers = _enum_identifiers(values, self.schema.source)
        lines: list[str] = []
        for index, value in enumerate(values):
            keyword = "if" if index == 0 else "else if"
            lines.append(
                f"{indentation}{keyword} ({source_expression} == {_cpp_string(value)}) {{"
            )
            lines.append(
                f"{indentation}  {target_expression} = {enum_type}::{identifiers[value]};"
            )
            lines.append(f"{indentation}}}")
        option = field_value.cli.flags[0]
        allowed = ", ".join(values)
        lines.extend(
            [
                f"{indentation}else {{",
                f"{indentation}  *error_message = {_cpp_string(option + ' must be one of: ' + allowed)};",
                f"{indentation}  return false;",
                f"{indentation}}}",
            ]
        )
        return lines

    def _cli_scalar_lines(
        self,
        entry: CliField,
        scalar_type: str,
        source: str,
        target: str,
        indentation: str,
        enum_item: bool = False,
    ) -> list[str]:
        option = _cpp_string(entry.field.cli.flags[0])
        if scalar_type == "string":
            return [f"{indentation}{target} = {source};"]
        if scalar_type == "path":
            return [f"{indentation}{target} = std::filesystem::path{{{source}}};"]
        if scalar_type == "enum":
            return self._cli_enum_lines(
                entry, source, target, indentation, item=enum_item
            )
        cpp_type = CPP_TYPES[scalar_type]
        function = {
            "bool": "ParseCliBool",
            "uint32": "ParseCliUInt32",
            "uint64": "ParseCliUInt64",
            "double": "ParseCliDouble",
        }[scalar_type]
        temporary = "parsed_item" if "[index]" in target else f"parsed_{entry.variable}"
        return [
            f"{indentation}{cpp_type} {temporary}{{}};",
            f"{indentation}if (!runtime::{function}({source}, {option}, &{temporary},",
            f"{indentation}                            error_message)) {{",
            f"{indentation}  return false;",
            f"{indentation}}}",
            f"{indentation}{target} = {temporary};",
        ]

    def _apply_cli(self) -> str:
        lines = [
            f"bool ApplyOverrides(const Overrides& overrides, {self.schema.name}* value,",
            f"                    {self.root.presence_type}* presence,",
            "                    std::string* error_message) {",
        ]
        for entry in self.cli_fields:
            field_value = entry.field
            target = self._value_expression(entry)
            variable = f"overrides.{entry.variable}"
            if field_value.type == "bool":
                conditions: list[tuple[str, str]] = []
                if field_value.cli.flags:
                    conditions.append((f"{variable}_true", "true"))
                if field_value.cli.inverse_flags:
                    conditions.append((f"{variable}_false", "false"))
                for index, (condition, bool_value) in enumerate(conditions):
                    keyword = "if" if index == 0 else "else if"
                    lines.append(f"  {keyword} ({condition}) {{")
                    lines.append(f"    {target} = {bool_value};")
                    lines.extend(self._mark_cli_lines(entry, "    "))
                    lines.append("  }")
                continue
            if field_value.type == "vector<string>":
                lines.append(f"  if (!{variable}.empty()) {{")
                lines.append(f"    {target} = {variable};")
                lines.extend(self._mark_cli_lines(entry, "    "))
                lines.append("  }")
                continue
            if field_value.type == "array":
                lines.append(f"  if (!{variable}.empty()) {{")
                lines.extend(
                    [
                        f"    if ({variable}.size() != {field_value.size}) {{",
                        f"      *error_message = {_cpp_string(field_value.cli.flags[0] + f' requires exactly {field_value.size} values')};",
                        "      return false;",
                        "    }",
                        f"    for (std::size_t index = 0; index < {field_value.size}; ++index) {{",
                    ]
                )
                lines.extend(
                    self._cli_scalar_lines(
                        entry,
                        field_value.item_type or "",
                        f"{variable}[index]",
                        f"{target}[index]",
                        "      ",
                        enum_item=True,
                    )
                )
                lines.append("    }")
                lines.extend(self._mark_cli_lines(entry, "    "))
                lines.append("  }")
                continue
            lines.append(f"  if ({variable}.has_value()) {{")
            lines.extend(
                self._cli_scalar_lines(
                    entry,
                    field_value.type,
                    f"*{variable}",
                    target,
                    "    ",
                )
            )
            lines.extend(self._mark_cli_lines(entry, "    "))
            lines.append("  }")
        lines.extend(["  return true;", "}"])
        return "\n".join(lines)

    def _helpers(self) -> str:
        return "\n\n".join(
            [
                self._presence_definitions(),
                """bool ConstraintError(const std::filesystem::path& source,
                     const Origin& origin, const std::string& message,
                     std::string* error_message) {
  if (origin.provided && !origin.mark.is_null()) {
    return runtime::ErrorAt(source, origin.mark, message, error_message);
  }
  *error_message = message;
  return false;
}""",
                self._parse_functions(),
                self._validation_functions(),
                self._overrides_definition(),
                self._configure_cli(),
                self._apply_cli(),
                f"""bool LoadInto(const std::filesystem::path& source,
              {self.schema.name}* value, {self.root.presence_type}* presence,
              std::string* error_message) {{
  YAML::Node root;
  if (!runtime::LoadSingleMapping(source, &root, error_message)) {{
    return false;
  }}
  return Parse_{self.root.function_id}(source, root, value, presence, "",
                                      error_message);
}}""",
            ]
        )

    def render_source_definitions(self) -> str:
        root_type = self.schema.name
        return f"""bool Load(const std::filesystem::path& path, {root_type}* config,
          std::string* error_message) {{
  if (config == nullptr || error_message == nullptr) {{
    if (error_message != nullptr) {{
      *error_message = "config and error_message must not be null";
    }}
    return false;
  }}
  error_message->clear();
  {root_type} parsed = *config;
  {self.root.presence_type} presence;
  if (!LoadInto(path, &parsed, &presence, error_message) ||
      !Validate_{self.root.function_id}(parsed, presence, path, "", error_message)) {{
    return false;
  }}
  *config = std::move(parsed);
  return true;
}}

bool Parse(int argc, const char* const argv[], {root_type}* config,
           std::string* error_message) {{
  if (config == nullptr || error_message == nullptr) {{
    if (error_message != nullptr) {{
      *error_message = "config and error_message must not be null";
    }}
    return false;
  }}
  error_message->clear();
  Overrides overrides;
  CLI::App app{{{_cpp_string(self.schema.description)}, {_cpp_string(self.schema.program)}}};
  ConfigureCli(&app, &overrides);
  try {{
    app.parse(argc, argv);
  }} catch (const CLI::CallForHelp&) {{
    *error_message = app.help();
    return false;
  }} catch (const CLI::ParseError& error) {{
    *error_message = runtime::FormatCliError(app, error);
    return false;
  }}

  {root_type} parsed = *config;
  {self.root.presence_type} presence;
  std::filesystem::path source;
  if (overrides.config_path.has_value()) {{
    source = std::filesystem::path{{*overrides.config_path}};
    if (!LoadInto(source, &parsed, &presence, error_message)) {{
      return false;
    }}
  }}
  if (!ApplyOverrides(overrides, &parsed, &presence, error_message) ||
      !Validate_{self.root.function_id}(parsed, presence, source, "", error_message)) {{
    return false;
  }}
  *config = std::move(parsed);
  return true;
}}

std::string Usage() {{
  Overrides overrides;
  CLI::App app{{{_cpp_string(self.schema.description)}, {_cpp_string(self.schema.program)}}};
  ConfigureCli(&app, &overrides);
  return app.help();
}}"""

    def render(self, template_dir: Path) -> tuple[str, str]:
        notice = (
            "// Generated by mujoco_sim_config. Do not edit.\n"
            f"// Schema: {self.schema.source.name}"
        )
        header_template = (template_dir / "config.hpp.in").read_text(encoding="utf-8")
        source_template = (template_dir / "config.cpp.in").read_text(encoding="utf-8")
        header = (
            header_template.replace("@NOTICE@", notice)
            .replace("@NAMESPACE@", self.schema.namespace)
            .replace("@DECLARATIONS@", self.render_header_declarations())
        )
        source = (
            source_template.replace("@NOTICE@", notice)
            .replace("@HEADER_INCLUDE@", self.header_include)
            .replace("@NAMESPACE@", self.schema.namespace)
            .replace("@HELPERS@", self._helpers())
            .replace("@DEFINITIONS@", self.render_source_definitions())
        )
        return header, source


def render(
    schema: Schema, header_include: str, template_dir: str | Path
) -> tuple[str, str]:
    return Renderer(schema, header_include).render(Path(template_dir))
