"""Strict JSON conversion at the diagnostic output boundary only.

Invalid robot feedback deliberately contains NaN vectors and a -Infinity
timestamp.  Those values must remain invalid in the control loop, while the
WAIT event describing them must still be writable with ``allow_nan=False``.
This module makes a separate JSON value, replaces nonfinite numeric leaves
with null, and records their exact JSON Pointer paths.  Never pass its output
back to a controller, estimator, referee, or feedback freshness check.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


SERIALIZATION_FIELD = "telemetry_serialization"


def _pointer_child(parent: str, key: str) -> str:
    return parent + "/" + key.replace("~", "~0").replace("/", "~1")


def sanitize_record(record: Mapping[Any, Any]) -> dict[str, Any]:
    """Copy a diagnostic record into strict JSON types, preserving bad-value evidence.

    ``telemetry_serialization.valid`` reports whether all original numeric
    values were finite.  It does not certify sensor or motion validity.  A bad
    value becomes null only in this returned copy; its path and original kind
    (NaN, +Infinity or -Infinity) remain in the metadata.  Booleans, finite
    numbers, strings, None, mappings, lists/tuples, numpy arrays/scalars,
    dataclasses and Paths are supported.  Finite numeric/bool/None mapping keys
    use the same string conversion as JSON (mission bearing maps use integer
    target IDs).  Unknown types and nonfinite keys still raise TypeError;
    converted key collisions fail explicitly instead of losing a field.

    The metadata key is reserved: sanitize each record exactly once, at its
    final write boundary, and continue using json.dumps(..., allow_nan=False).
    """
    if not isinstance(record, Mapping):
        raise TypeError("telemetry record must be a mapping")
    if SERIALIZATION_FIELD in record:
        raise ValueError(f"{SERIALIZATION_FIELD} is reserved; sanitize a record only once")
    invalid: list[dict[str, str]] = []
    active_containers: set[int] = set()

    def json_key(key: Any, pointer: str) -> str:
        if isinstance(key, str):
            return str(key)
        if key is None:
            return "null"
        if isinstance(key, (bool, np.bool_)):
            return "true" if key else "false"
        if isinstance(key, (int, np.integer)):
            return str(int(key))
        if isinstance(key, (float, np.floating)) and bool(np.isfinite(key)):
            number = float(key)
            if math.isfinite(number):
                return json.dumps(number, allow_nan=False)
        raise TypeError(f"unsupported or nonfinite telemetry mapping key at {pointer}: {type(key)}")

    def visit(value: Any, pointer: str) -> Any:
        if value is None or isinstance(value, (str, bool)):
            return value
        if isinstance(value, np.bool_):
            return bool(value)
        if isinstance(value, (int, np.integer)):
            return int(value)
        if isinstance(value, (float, np.floating)):
            # Check the source before converting a numpy scalar.  In
            # particular, longdouble.item() can return another longdouble.
            if not bool(np.isfinite(value)):
                kind = "NaN" if bool(np.isnan(value)) else "+Infinity" if value > 0 else "-Infinity"
                invalid.append({"path": pointer, "value": kind})
                return None
            result = float(value)
            if not math.isfinite(result):
                raise OverflowError(f"finite numeric value outside JSON float range at {pointer}")
            return result
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, np.generic):
            converted = value.item()
            if type(converted) is type(value):
                raise TypeError(f"not JSON serializable at {pointer}: {type(value)}")
            return visit(converted, pointer)

        identity = id(value)
        if identity in active_containers:
            raise ValueError(f"circular telemetry value at {pointer}")
        active_containers.add(identity)
        try:
            if isinstance(value, np.ndarray):
                return visit(value.tolist(), pointer)
            if is_dataclass(value) and not isinstance(value, type):
                return {field.name: visit(getattr(value, field.name), _pointer_child(pointer, field.name))
                        for field in fields(value)}
            if isinstance(value, Mapping):
                result = {}
                for key, item in value.items():
                    converted_key = json_key(key, pointer)
                    if converted_key in result:
                        raise ValueError(f"telemetry key collision after JSON conversion at {pointer}: {converted_key}")
                    result[converted_key] = visit(item, _pointer_child(pointer, converted_key))
                return result
            if isinstance(value, (list, tuple)):
                return [visit(item, _pointer_child(pointer, str(index))) for index, item in enumerate(value)]
            raise TypeError(f"not JSON serializable at {pointer}: {type(value)}")
        finally:
            active_containers.remove(identity)

    result = visit(record, "")
    result[SERIALIZATION_FIELD] = {
        "schema": 1,
        "valid": not invalid,
        "nonfinite_paths": [item["path"] for item in invalid],
        "nonfinite_values": invalid,
    }
    return result
