"""Real PointCloud2 byte layouts decoded without ROS or message-reader stubs."""

from array import array
import copy
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cloud_io import read_xyz


def field(name, offset, datatype=7, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def cloud(points, width=None, *, bigendian=False, fields=None, point_step=12, padding=0):
    """Pack individually addressed values, independently of NumPy's decoder."""
    points = list(points)
    width = len(points) if width is None else width
    height = len(points) // width if width else 1
    row_step = width * point_step + padding
    fields = fields if fields is not None else [field("x", 0), field("y", 4), field("z", 8)]
    payload = bytearray([0xA5] * (row_step * height))
    for index, point in enumerate(points):
        row, column = divmod(index, width)
        for item in fields:
            if item.name in ("x", "y", "z"):
                location = row * row_step + column * point_step + item.offset
                kind = "f" if item.datatype == 7 else "d"
                struct.pack_into((">" if bigendian else "<") + kind, payload, location,
                                 point[("x", "y", "z").index(item.name)])
    return SimpleNamespace(width=width, height=height, point_step=point_step, row_step=row_step,
                           is_bigendian=bigendian, fields=fields, data=bytes(payload))


class CloudIoTests(unittest.TestCase):
    def test_tightly_packed_xyz_float32_is_native_contiguous_and_independent(self):
        points = [(1.25, -2.5, 3.75), (-4.5, 0, 6.125)]
        message = cloud(points)
        message.data = array("B", message.data)  # ROS Python uses array('B').
        decoded = read_xyz(message)
        np.testing.assert_array_equal(decoded, points)
        self.assertEqual(decoded.dtype, np.dtype(np.float32))
        self.assertEqual(decoded.shape, (2, 3))
        self.assertTrue(decoded.flags.c_contiguous)
        message.data[:] = array("B", [0] * len(message.data))
        np.testing.assert_array_equal(decoded, points)

    def test_organized_rows_and_point_padding_skip_extra_fields(self):
        points = [(index + 0.25, -index - 0.5, 0.125 * index) for index in range(6)]
        fields = [field("ring", 0, 4), field("z", 20), field("intensity", 4),
                  field("x", 12), field("y", 16), field("timestamp", 24, 8)]
        for endian in (False, True):
            with self.subTest(bigendian=endian):
                message = cloud(points, width=3, bigendian=endian, fields=fields,
                                point_step=40, padding=17)
                decoded = read_xyz(message)
                np.testing.assert_array_equal(decoded, points)
                self.assertTrue(decoded.flags.c_contiguous)
                self.assertTrue(decoded.dtype.isnative)

    def test_float64_preserves_precision_for_both_byte_orders(self):
        points = [(12345678.123456789, -0.0000000012345, 4.56789123456789), (2.1, 3.2, -4.3)]
        for endian in (False, True):
            with self.subTest(bigendian=endian):
                message = cloud(points, width=1, bigendian=endian,
                                fields=[field("y", 0, 8), field("x", 8, 8), field("z", 16, 8)],
                                point_step=28, padding=7)
                decoded = read_xyz(message)
                np.testing.assert_array_equal(decoded, points)
                self.assertEqual(decoded.dtype, np.dtype(np.float64))

    def test_mixed_float_widths_and_unaligned_offsets(self):
        points = [(1.25, 12345678.123456789, -3.5)]
        for endian in (False, True):
            with self.subTest(bigendian=endian):
                message = cloud(points, bigendian=endian,
                                fields=[field("x", 1), field("y", 5, 8), field("z", 13)], point_step=19)
                np.testing.assert_array_equal(read_xyz(message), points)

    def test_nonfinite_values_are_preserved_for_later_point_filter(self):
        message = cloud([(float("nan"), 2, 3), (1, float("inf"), float("-inf"))])
        decoded = read_xyz(message)
        self.assertTrue(np.isnan(decoded[0, 0]))
        self.assertTrue(np.isposinf(decoded[1, 1]))
        self.assertTrue(np.isneginf(decoded[1, 2]))
        self.assertEqual(decoded.shape, (2, 3))

    def test_empty_cloud_with_valid_xyz_schema(self):
        for padding in (0, 8):
            with self.subTest(padding=padding):
                result = read_xyz(cloud([], padding=padding))
                self.assertEqual(result.shape, (0, 3))
                self.assertEqual(result.dtype, np.dtype(np.float32))

    def test_zero_height_and_negative_dimensions_are_rejected(self):
        message = cloud([(1, 2, 3)])
        for attribute, value in (("height", 0), ("height", -1), ("width", -1),
                                 ("point_step", -1), ("row_step", -1)):
            malformed = copy.deepcopy(message)
            setattr(malformed, attribute, value)
            with self.subTest(attribute=attribute, value=value), self.assertRaises(ValueError):
                read_xyz(malformed)

    def test_empty_cloud_still_requires_valid_xyz_schema_and_payload(self):
        for changes in ({"fields": []}, {"point_step": 0}, {"data": b"invalid"}):
            malformed = cloud([])
            for key, value in changes.items():
                setattr(malformed, key, value)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                read_xyz(malformed)

    def test_missing_duplicate_array_and_nonfloat_xyz_fields_are_rejected(self):
        invalid = [
            [field("x", 0), field("y", 4)],
            [field("x", 0), field("x", 0), field("y", 4), field("z", 8)],
            [field("x", 0, count=2), field("y", 4), field("z", 8)],
            [field("x", 0, count=0), field("y", 4), field("z", 8)],
            [field("x", 0, datatype=6), field("y", 4), field("z", 8)],
            [field("x", 0, datatype=9), field("y", 4), field("z", 8)],
        ]
        for fields in invalid:
            malformed = cloud([(1, 2, 3)])
            malformed.fields = fields
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                read_xyz(malformed)

    def test_xyz_offsets_cannot_extend_outside_point_stride(self):
        for changed in (field("x", -1), field("x", 9), field("x", 5, 8)):
            malformed = cloud([(1, 2, 3)])
            malformed.fields[0] = changed
            with self.subTest(field=changed), self.assertRaises(ValueError):
                read_xyz(malformed)

    def test_short_row_stride_and_wrong_payload_length_are_rejected(self):
        original = cloud([(1, 2, 3), (4, 5, 6)], width=1, padding=3)
        for changes in ({"row_step": 11}, {"data": original.data[:-1]},
                        {"data": original.data + b"\x00"}, {"point_step": 0}):
            malformed = copy.deepcopy(original)
            for key, value in changes.items():
                setattr(malformed, key, value)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                read_xyz(malformed)


if __name__ == "__main__":
    unittest.main()
