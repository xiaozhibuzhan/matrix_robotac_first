"""Vectorized, validated XYZ decoding for ROS PointCloud2 without ROS imports."""

import numpy as np


def read_scalar_field(message, name: str, datatype: int = 7) -> np.ndarray:
    """Read one scalar FLOAT32/FLOAT64 PointCloud2 field with row padding."""
    fields = [field for field in message.fields if field.name == name]
    if len(fields) != 1 or fields[0].count != 1 or fields[0].datatype != datatype:
        raise ValueError(f"PointCloud2 is missing scalar {name!r} field")
    field = fields[0]
    size = 4 if datatype == 7 else 8
    if field.offset < 0 or field.offset + size > message.point_step:
        raise ValueError(f"PointCloud2 field {name!r} exceeds point_step")
    if message.row_step < message.width * message.point_step:
        raise ValueError("PointCloud2 row_step is too short")
    view = memoryview(message.data)
    if view.nbytes != message.height * message.row_step:
        raise ValueError("PointCloud2 payload length does not match row_step * height")
    endian = ">" if message.is_bigendian else "<"
    result = np.ndarray((message.height, message.width), dtype=endian + ("f4" if datatype == 7 else "f8"),
                        buffer=view, offset=field.offset,
                        strides=(message.row_step, message.point_step)).reshape(-1)
    return result.astype(np.float64, copy=True)


def read_xyz(message) -> np.ndarray:
    """Read XYZ respecting field offsets, endian flag and padded organized rows."""
    width, height = int(message.width), int(message.height)
    step, row_step = int(message.point_step), int(message.row_step)
    if min(width, height, step, row_step) < 0 or height < 1:
        raise ValueError("invalid PointCloud2 dimensions")
    fields = {}
    for field in message.fields:
        if field.name in ("x", "y", "z"):
            if field.name in fields or field.count != 1 or field.datatype not in (7, 8):
                raise ValueError("XYZ must be unique scalar FLOAT32 or FLOAT64 fields")
            size = 4 if field.datatype == 7 else 8
            if field.offset < 0 or field.offset + size > step:
                raise ValueError("XYZ field exceeds point_step")
            fields[field.name] = (field.offset, size)
    if len(fields) != 3:
        raise ValueError("PointCloud2 is missing XYZ fields")
    if step == 0 or row_step < width * step:
        raise ValueError("PointCloud2 row_step is too short")
    view = memoryview(message.data)
    if view.nbytes != height * row_step:
        raise ValueError("PointCloud2 payload length does not match row_step * height")
    if width == 0:
        return np.empty((0, 3), dtype=np.float32)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": [endian + "f" + str(fields[n][1]) for n in ("x", "y", "z")],
                      "offsets": [fields[n][0] for n in ("x", "y", "z")], "itemsize": step})
    records = np.ndarray((height, width), dtype=dtype, buffer=view, strides=(row_step, step))
    return np.column_stack([records[n].reshape(-1) for n in ("x", "y", "z")])


def read_field(message, name: str) -> np.ndarray:
    """Read one scalar numeric PointCloud2 field as float64."""
    width, height = int(message.width), int(message.height)
    step, row_step = int(message.point_step), int(message.row_step)
    field = next((item for item in message.fields if item.name == name), None)
    if field is None or field.count != 1 or field.datatype not in (2, 4, 6, 7, 8):
        raise ValueError(f"PointCloud2 is missing scalar field {name!r}")
    sizes = {2: 1, 4: 2, 6: 4, 7: 4, 8: 8}
    formats = {2: "u1", 4: "i2", 6: "i4", 7: "f4", 8: "f8"}
    size = sizes[field.datatype]
    if field.offset < 0 or field.offset + size > step or row_step < width * step:
        raise ValueError("field exceeds point_step")
    view = memoryview(message.data)
    if view.nbytes != height * row_step:
        raise ValueError("PointCloud2 payload length does not match row_step * height")
    if width == 0:
        return np.empty(0, dtype=np.float64)
    endian = ">" if message.is_bigendian and size > 1 else "<"
    dtype = np.dtype(endian + formats[field.datatype])
    records = np.ndarray((height, width), dtype=dtype, buffer=view,
                         offset=field.offset, strides=(row_step, step))
    return records.reshape(-1).astype(np.float64, copy=False)
