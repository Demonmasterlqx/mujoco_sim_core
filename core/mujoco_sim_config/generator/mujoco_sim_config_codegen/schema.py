from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


MISSING = object()
PRIMITIVE_TYPES = {"string", "path", "bool", "uint32", "uint64", "double"}
SCALAR_TYPES = PRIMITIVE_TYPES | {"enum"}
FIELD_TYPES = PRIMITIVE_TYPES | {
    "vector<string>",
    "array",
    "object",
    "object_sequence",
    "enum",
}
CPP_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
CPP_NAMESPACE = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:::[A-Za-z_][A-Za-z0-9_]*)*$"
)
CLI_FLAG = re.compile(r"^--?[A-Za-z0-9][A-Za-z0-9_-]*$")


class SchemaError(Exception):
    pass


class MarkedMap(dict):
    mark: yaml.Mark
    key_marks: dict[Any, yaml.Mark]


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader: UniqueKeyLoader, node: yaml.MappingNode) -> MarkedMap:
    loader.flatten_mapping(node)
    result = MarkedMap()
    result.mark = node.start_mark
    result.key_marks = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        try:
            duplicate = key in result
        except TypeError as error:
            raise SchemaError(
                f"{key_node.start_mark.name}:{key_node.start_mark.line + 1}:"
                f"{key_node.start_mark.column + 1}: mapping keys must be scalar"
            ) from error
        if duplicate:
            raise SchemaError(
                f"{key_node.start_mark.name}:{key_node.start_mark.line + 1}:"
                f"{key_node.start_mark.column + 1}: duplicate key: {key}"
            )
        result[key] = loader.construct_object(value_node, deep=True)
        result.key_marks[key] = key_node.start_mark
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


@dataclass(frozen=True)
class CliSpec:
    flags: tuple[str, ...] = ()
    inverse_flags: tuple[str, ...] = ()


@dataclass
class Field:
    name: str
    type: str
    description: str
    path: tuple[str, ...]
    default: Any = MISSING
    required: bool = False
    non_empty: bool = False
    minimum: int | float | None = None
    maximum: int | float | None = None
    finite: bool = False
    size: int | None = None
    unique: bool = False
    cli: CliSpec = field(default_factory=CliSpec)
    fields: list[Field] = field(default_factory=list)
    item_type: str | None = None
    enum_values: tuple[str, ...] = ()
    item_enum_values: tuple[str, ...] = ()

    @property
    def dotted_name(self) -> str:
        return ".".join(self.path)


@dataclass(frozen=True)
class ConfigOption:
    flags: tuple[str, ...]
    description: str


@dataclass
class Schema:
    source: Path
    namespace: str
    name: str
    program: str
    description: str
    config: ConfigOption
    fields: list[Field]


def _location(mapping: MarkedMap | None, key: Any = None) -> str:
    if mapping is None:
        return "schema"
    mark = mapping.key_marks.get(key, mapping.mark)
    return f"{mark.name}:{mark.line + 1}:{mark.column + 1}"


def _error(mapping: MarkedMap | None, key: Any, message: str) -> SchemaError:
    return SchemaError(f"{_location(mapping, key)}: {message}")


def _mapping(value: Any, parent: MarkedMap | None, key: Any, label: str) -> MarkedMap:
    if not isinstance(value, MarkedMap):
        raise _error(parent, key, f"{label} must be a mapping")
    return value


def _known_keys(mapping: MarkedMap, allowed: set[str], label: str) -> None:
    for key in mapping:
        if not isinstance(key, str):
            raise _error(mapping, key, f"{label} keys must be strings")
        if key not in allowed:
            raise _error(mapping, key, f"unknown {label} key: {key}")


def _required_string(mapping: MarkedMap, key: str, label: str) -> str:
    if key not in mapping:
        raise _error(mapping, None, f"missing required key: {key}")
    value = mapping[key]
    if not isinstance(value, str) or not value:
        raise _error(mapping, key, f"{label} must be a non-empty string")
    return value


def _optional_string(mapping: MarkedMap, key: str, default: str) -> str:
    if key not in mapping:
        return default
    value = mapping[key]
    if not isinstance(value, str):
        raise _error(mapping, key, f"{key} must be a string")
    return value


def _bool(mapping: MarkedMap, key: str, default: bool) -> bool:
    if key not in mapping:
        return default
    value = mapping[key]
    if type(value) is not bool:
        raise _error(mapping, key, f"{key} must be a boolean")
    return value


def _non_negative_int(mapping: MarkedMap, key: str) -> int | None:
    if key not in mapping:
        return None
    value = mapping[key]
    if type(value) is not int or value < 0:
        raise _error(mapping, key, f"{key} must be a non-negative integer")
    return value


def _flags(value: Any, mapping: MarkedMap, key: str, label: str) -> tuple[str, ...]:
    if isinstance(value, str):
        values = [value]
    elif isinstance(value, list):
        values = value
    else:
        raise _error(mapping, key, f"{label} must be a flag or a list of flags")
    if not values:
        raise _error(mapping, key, f"{label} must not be empty")
    result: list[str] = []
    for flag_value in values:
        if not isinstance(flag_value, str) or not CLI_FLAG.fullmatch(flag_value):
            raise _error(mapping, key, f"invalid CLI flag: {flag_value!r}")
        if flag_value in result:
            raise _error(mapping, key, f"duplicate CLI flag: {flag_value}")
        result.append(flag_value)
    return tuple(result)


def _parse_cli(mapping: MarkedMap, field_type: str) -> CliSpec:
    if "cli" not in mapping:
        return CliSpec()
    value = mapping["cli"]
    if isinstance(value, (str, list)):
        return CliSpec(flags=_flags(value, mapping, "cli", "cli"))
    cli = _mapping(value, mapping, "cli", "cli")
    _known_keys(cli, {"flags", "inverse_flags"}, "cli")
    flags = (
        _flags(cli["flags"], cli, "flags", "cli.flags")
        if "flags" in cli
        else ()
    )
    inverse = (
        _flags(cli["inverse_flags"], cli, "inverse_flags", "cli.inverse_flags")
        if "inverse_flags" in cli
        else ()
    )
    if not flags and not inverse:
        raise _error(mapping, "cli", "cli must declare flags")
    if inverse and field_type != "bool":
        raise _error(mapping, "cli", "inverse_flags are only valid for bool fields")
    return CliSpec(flags=flags, inverse_flags=inverse)


def _enum_values(value: Any, mapping: MarkedMap, key: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise _error(mapping, key, f"{key} must be a non-empty list")
    result: list[str] = []
    for item in value:
        if not isinstance(item, str) or not item:
            raise _error(mapping, key, f"{key} entries must be non-empty strings")
        if item in result:
            raise _error(mapping, key, f"duplicate enum value: {item}")
        result.append(item)
    return tuple(result)


def _parse_number_constraint(mapping: MarkedMap, key: str) -> int | float | None:
    if key not in mapping:
        return None
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error(mapping, key, f"{key} must be a number")
    if isinstance(value, float) and not math.isfinite(value):
        raise _error(mapping, key, f"{key} must be finite")
    return value


def _parse_item_spec(mapping: MarkedMap, path: tuple[str, ...]) -> tuple[str, tuple[str, ...]]:
    if "items" not in mapping:
        raise _error(mapping, None, "array fields require items")
    value = mapping["items"]
    if isinstance(value, str):
        item_type = value
        enum_values: tuple[str, ...] = ()
    else:
        item = _mapping(value, mapping, "items", "items")
        _known_keys(item, {"type", "values"}, "array item")
        item_type = _required_string(item, "type", "items.type")
        enum_values = (
            _enum_values(item.get("values"), item, "values")
            if item_type == "enum"
            else ()
        )
    if item_type not in SCALAR_TYPES:
        raise _error(mapping, "items", "array items must have a scalar type")
    if item_type == "enum" and not enum_values:
        raise _error(mapping, "items", "enum array items require values")
    return item_type, enum_values


def _parse_object_sequence_fields(
    mapping: MarkedMap, path: tuple[str, ...]
) -> list[Field]:
    if "items" not in mapping:
        raise _error(mapping, None, "object_sequence fields require items")
    item = _mapping(mapping["items"], mapping, "items", "items")
    _known_keys(item, {"type", "fields"}, "object_sequence item")
    if item.get("type", "object") != "object":
        raise _error(item, "type", "object_sequence items must have type: object")
    if "fields" not in item:
        raise _error(item, None, "object_sequence items require fields")
    return _parse_fields(item["fields"], item, "fields", path + ("item",))


def _validate_scalar_default(
    value: Any,
    field_type: str,
    enum_values: tuple[str, ...],
    mapping: MarkedMap,
) -> None:
    valid = False
    if field_type in {"string", "path"}:
        valid = isinstance(value, str)
    elif field_type == "bool":
        valid = type(value) is bool
    elif field_type == "uint32":
        valid = type(value) is int and 0 <= value <= 0xFFFFFFFF
    elif field_type == "uint64":
        valid = type(value) is int and 0 <= value <= 0xFFFFFFFFFFFFFFFF
    elif field_type == "double":
        valid = not isinstance(value, bool) and isinstance(value, (int, float))
    elif field_type == "enum":
        valid = isinstance(value, str) and value in enum_values
    if not valid:
        raise _error(mapping, "default", f"invalid default for type {field_type}")


def _validate_object_default(field_value: Field, value: Any, mapping: MarkedMap) -> None:
    if not isinstance(value, MarkedMap):
        raise _error(mapping, "default", "object default must be a mapping")
    children = {child.name: child for child in field_value.fields}
    for key, child_value in value.items():
        if key not in children:
            raise _error(value, key, f"unknown default field: {key}")
        _validate_default_value(children[key], child_value, value)


def _validate_default_value(field_value: Field, value: Any, mapping: MarkedMap) -> None:
    if field_value.type in SCALAR_TYPES:
        _validate_scalar_default(
            value, field_value.type, field_value.enum_values, mapping
        )
    elif field_value.type == "vector<string>":
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            raise _error(mapping, "default", "vector<string> default must be a string list")
    elif field_value.type == "array":
        if not isinstance(value, list) or len(value) != field_value.size:
            raise _error(
                mapping,
                "default",
                f"array default must contain exactly {field_value.size} items",
            )
        for item in value:
            _validate_scalar_default(
                item,
                field_value.item_type or "",
                field_value.item_enum_values,
                mapping,
            )
    elif field_value.type == "object":
        _validate_object_default(field_value, value, mapping)
    elif field_value.type == "object_sequence":
        if not isinstance(value, list):
            raise _error(mapping, "default", "object_sequence default must be a list")
        for item in value:
            _validate_object_default(field_value, item, mapping)


def _validate_constraints(field_value: Field, mapping: MarkedMap) -> None:
    numeric = field_value.type in {"uint32", "uint64", "double"} or (
        field_value.type == "array"
        and field_value.item_type in {"uint32", "uint64", "double"}
    )
    sizeable_types = {
        "string",
        "path",
        "vector<string>",
        "array",
        "object_sequence",
    }
    sizeable = field_value.type in sizeable_types
    non_empty_types = sizeable_types - {"array"}
    unique_types = {"vector<string>", "array"}

    if (field_value.required and field_value.default is not MISSING):
        raise _error(mapping, "required", "required fields cannot declare a default")
    if field_value.non_empty and field_value.type not in non_empty_types:
        raise _error(mapping, "non_empty", f"non_empty is not valid for {field_value.type}")
    if (field_value.minimum is not None or field_value.maximum is not None) and not numeric:
        raise _error(mapping, "min", f"min/max are not valid for {field_value.type}")
    if (
        field_value.minimum is not None
        and field_value.maximum is not None
        and field_value.minimum > field_value.maximum
    ):
        raise _error(mapping, "min", "min must not be greater than max")
    if field_value.finite and not (
        field_value.type == "double"
        or (field_value.type == "array" and field_value.item_type == "double")
    ):
        raise _error(mapping, "finite", f"finite is not valid for {field_value.type}")
    if field_value.size is not None and not sizeable:
        raise _error(mapping, "size", f"size is not valid for {field_value.type}")
    if field_value.type == "array" and field_value.size is None:
        raise _error(mapping, None, "array fields require size")
    if field_value.unique and field_value.type not in unique_types:
        raise _error(mapping, "unique", f"unique is not valid for {field_value.type}")
    if field_value.type in {"object", "object_sequence"} and field_value.cli.flags:
        raise _error(mapping, "cli", f"CLI flags are not valid for {field_value.type}")
    if field_value.type == "array" and field_value.size == 0 and field_value.cli.flags:
        raise _error(mapping, "cli", "zero-length arrays cannot have CLI flags")


def _validate_default_constraints(field_value: Field, mapping: MarkedMap) -> None:
    if field_value.default is MISSING:
        return
    value = field_value.default
    values = value if field_value.type == "array" else [value]
    if field_value.minimum is not None:
        if any(item < field_value.minimum for item in values):
            raise _error(mapping, "default", "default is below min")
    if field_value.maximum is not None:
        if any(item > field_value.maximum for item in values):
            raise _error(mapping, "default", "default is above max")
    if field_value.finite:
        if any(not math.isfinite(float(item)) for item in values):
            raise _error(mapping, "default", "default must be finite")
    if field_value.non_empty and len(value) == 0:
        raise _error(mapping, "default", "default must be non-empty")
    if field_value.size is not None and field_value.type != "array":
        if len(value) != field_value.size:
            raise _error(mapping, "default", f"default size must be {field_value.size}")
    if field_value.unique and len(set(value)) != len(value):
        raise _error(mapping, "default", "default items must be unique")


def _parse_field(name: str, raw: Any, parent: MarkedMap, path: tuple[str, ...]) -> Field:
    mapping = _mapping(raw, parent, name, f"field {name}")
    _known_keys(
        mapping,
        {
            "type",
            "description",
            "default",
            "required",
            "non_empty",
            "min",
            "max",
            "finite",
            "size",
            "unique",
            "cli",
            "fields",
            "items",
            "values",
        },
        "field",
    )
    field_type = _required_string(mapping, "type", f"field {name}.type")
    aliases = {"vector_string": "vector<string>", "sequence<object>": "object_sequence"}
    field_type = aliases.get(field_type, field_type)
    if field_type not in FIELD_TYPES:
        raise _error(mapping, "type", f"unsupported field type: {field_type}")

    field_path = path + (name,)
    nested_fields: list[Field] = []
    item_type: str | None = None
    enum_values: tuple[str, ...] = ()
    item_enum_values: tuple[str, ...] = ()
    if field_type == "object":
        if "fields" not in mapping:
            raise _error(mapping, None, "object fields require fields")
        nested_fields = _parse_fields(mapping["fields"], mapping, "fields", field_path)
    elif field_type == "object_sequence":
        nested_fields = _parse_object_sequence_fields(mapping, field_path)
    elif field_type == "array":
        item_type, item_enum_values = _parse_item_spec(mapping, field_path)
    elif field_type == "enum":
        if "values" not in mapping:
            raise _error(mapping, None, "enum fields require values")
        enum_values = _enum_values(mapping["values"], mapping, "values")

    result = Field(
        name=name,
        type=field_type,
        description=_optional_string(mapping, "description", name),
        path=field_path,
        default=mapping.get("default", MISSING),
        required=_bool(mapping, "required", False),
        non_empty=_bool(mapping, "non_empty", False),
        minimum=_parse_number_constraint(mapping, "min"),
        maximum=_parse_number_constraint(mapping, "max"),
        finite=_bool(mapping, "finite", False),
        size=_non_negative_int(mapping, "size"),
        unique=_bool(mapping, "unique", False),
        cli=_parse_cli(mapping, field_type),
        fields=nested_fields,
        item_type=item_type,
        enum_values=enum_values,
        item_enum_values=item_enum_values,
    )
    _validate_constraints(result, mapping)
    if result.default is not MISSING:
        _validate_default_value(result, result.default, mapping)
    _validate_default_constraints(result, mapping)
    return result


def _parse_fields(
    raw: Any, parent: MarkedMap, key: str, path: tuple[str, ...]
) -> list[Field]:
    mapping = _mapping(raw, parent, key, "fields")
    if not mapping:
        raise _error(parent, key, "fields must not be empty")
    result: list[Field] = []
    for name, value in mapping.items():
        if not isinstance(name, str) or not CPP_IDENTIFIER.fullmatch(name):
            raise _error(mapping, name, f"invalid C++ field name: {name!r}")
        result.append(_parse_field(name, value, mapping, path))
    return result


def _all_fields(fields: list[Field]):
    for field_value in fields:
        yield field_value
        yield from _all_fields(field_value.fields)


def _validate_flags(schema: Schema, root: MarkedMap) -> None:
    owners: dict[str, str] = {"-h": "help", "--help": "help"}
    for flag_value in schema.config.flags:
        if flag_value in owners:
            raise _error(root, "config", f"CLI flag {flag_value} conflicts with help")
        owners[flag_value] = "config"
    for field_value in _all_fields(schema.fields):
        for flag_value in field_value.cli.flags + field_value.cli.inverse_flags:
            owner = owners.get(flag_value)
            if owner is not None:
                raise _error(
                    root,
                    "fields",
                    f"CLI flag {flag_value} is used by both {owner} and {field_value.dotted_name}",
                )
            owners[flag_value] = field_value.dotted_name


def load_schema(path: str | Path) -> Schema:
    source = Path(path).resolve()
    try:
        with source.open("r", encoding="utf-8") as stream:
            raw = yaml.load(stream, Loader=UniqueKeyLoader)
    except OSError as error:
        raise SchemaError(f"{source}: {error.strerror}") from error
    except yaml.YAMLError as error:
        mark = getattr(error, "problem_mark", None)
        if mark is not None:
            raise SchemaError(
                f"{mark.name}:{mark.line + 1}:{mark.column + 1}: {error}"
            ) from error
        raise SchemaError(f"{source}: {error}") from error

    root = _mapping(raw, None, None, "schema root")
    _known_keys(
        root,
        {"version", "namespace", "name", "program", "description", "config", "fields"},
        "schema",
    )
    if root.get("version") != 1 or type(root.get("version")) is not int:
        raise _error(root, "version", "version must be 1")
    namespace = _required_string(root, "namespace", "namespace")
    if not CPP_NAMESPACE.fullmatch(namespace):
        raise _error(root, "namespace", f"invalid C++ namespace: {namespace}")
    name = _required_string(root, "name", "name")
    if not CPP_IDENTIFIER.fullmatch(name):
        raise _error(root, "name", f"invalid C++ type name: {name}")
    program = _required_string(root, "program", "program")
    description = _optional_string(root, "description", program)

    if "config" in root:
        config_mapping = _mapping(root["config"], root, "config", "config")
        _known_keys(config_mapping, {"flags", "description"}, "config")
        config_flags = _flags(
            config_mapping.get("flags", ["--config"]),
            config_mapping,
            "flags",
            "config.flags",
        )
        config_description = _optional_string(
            config_mapping, "description", "YAML configuration file"
        )
    else:
        config_flags = ("--config",)
        config_description = "YAML configuration file"

    if "fields" not in root:
        raise _error(root, None, "missing required key: fields")
    fields = _parse_fields(root["fields"], root, "fields", ())
    schema = Schema(
        source=source,
        namespace=namespace,
        name=name,
        program=program,
        description=description,
        config=ConfigOption(config_flags, config_description),
        fields=fields,
    )
    _validate_flags(schema, root)
    return schema
