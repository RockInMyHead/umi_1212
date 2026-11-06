#!/usr/bin/env python3
"""Host-only regression tests. Synthetic fixtures verify fitting, not sensor accuracy."""
import math
import struct
import unittest
import zlib
from fit_calibration import fit_accel, fit_gyro_temperature, cpp_header, FACES
from decode_telemetry import decode


class CalibrationTests(unittest.TestCase):
    def face_data(self):
        rows = []
        self.scales = [1.02, .97, 1.01]
        self.offsets = [.12, -.08, .05]
        for face in FACES:
            expected = [0., 0., 0.]
            expected["xyz".index(face[1])] = 9.80665 if face[0] == "+" else -9.80665
            for _ in range(200):
                row = dict(face=face, temp_c=25., gx=.001, gy=-.002, gz=.0005)
                for i, column in enumerate(("ax", "ay", "az")):
                    row[column] = expected[i]/self.scales[i]+self.offsets[i]
                rows.append(row)
        return rows

    def test_six_face_recovers_known_scale_offset(self):
        result = fit_accel(self.face_data())
        for actual, expected in zip(result["scale"], self.scales):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(result["offset"], self.offsets):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertFalse(result["issues"])
        self.assertIn("measuredCalibrationProvided", cpp_header({"accelerometer": result}))

    def test_missing_face_is_rejected(self):
        with self.assertRaises(ValueError):
            fit_accel([r for r in self.face_data() if r["face"] != "-z"])

    def test_motion_screening(self):
        rows = self.face_data()
        for i, row in enumerate(rows):
            row["ax"] += .8 * (1 if i % 2 else -1)
        self.assertTrue(fit_accel(rows)["issues"])

    def test_gyro_temperature_recovers_linear_model(self):
        rows = []
        for i in range(900):
            t = 15. + 30.*i/899
            rows.append(dict(temp_c=t, ax=0., ay=0., az=9.80665,
                             gx=.001+.0002*(t-30), gy=-.002-.0001*(t-30), gz=.003+.0003*(t-30)))
        result = fit_gyro_temperature(rows)
        for actual, expected in zip(result["slope"], [.0002, -.0001, .0003]):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertFalse(result["issues"])
        for r in rows:
            r["temp_c"] = 25.
        with self.assertRaises(ValueError):
            fit_gyro_temperature(rows)


class ProtocolTests(unittest.TestCase):
    def test_st_version3_extension(self):
        packet=bytearray(148)
        packet[:4]=b"IN\x03\x94"
        struct.pack_into("<8I",packet,108,1200,700,13000,41000,600,500,0,0)
        struct.pack_into("<ffI",packet,132,1013.25,4.0,2)
        struct.pack_into("<I",packet,144,zlib.crc32(packet[:144]))
        result=decode(packet)
        self.assertEqual(result["version"],3)
        self.assertEqual(result["mag_age_us"],13000)
        self.assertEqual(result["pressure_hpa"],1013.25)
        self.assertEqual(result["barometer_bias_variance_m2"],4.)
        self.assertEqual(result["auxiliary_overruns"],2)

    def test_packet_decode_and_crc_rejection(self):
        packet = bytearray(112)
        packet[:4] = b"IN\x02\x70"
        struct.pack_into("<IIQII", packet, 4, 77, 4, 28_800_000_000, 1, 0)
        struct.pack_into("<3f", packet, 28, 1.25, -2.5, 9.75)
        struct.pack_into("<I", packet, 108, zlib.crc32(packet[:108]))
        result = decode(packet)
        self.assertEqual(result["filter_time_us"], 28_800_000_000)
        self.assertEqual(result["position_m"], (1.25, -2.5, 9.75))
        packet[44] ^= 1
        with self.assertRaises(ValueError):
            decode(packet)


if __name__ == "__main__":
    unittest.main()
