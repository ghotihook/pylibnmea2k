"""
Cross-validate pylibnmea2k against the nmea2000 reference library.

Each test encodes known values into a synthetic YDWG line, decodes with
both libraries, and asserts they agree within quantisation tolerance.
Sentinel tests verify that "not available" bit patterns return None.
"""

import math
import struct
import unittest
from datetime import date as _date

from nmea2000.consts import PhysicalQuantities
from nmea2000.decoder import NMEA2000Decoder

import pylibnmea2k

# ── Shared library decoder (initialised once for the module) ──────────────────

_lib = NMEA2000Decoder(
    include_pgns=[
        pylibnmea2k.PGN_RUDDER,    pylibnmea2k.PGN_HEADING,  pylibnmea2k.PGN_ROT,
        pylibnmea2k.PGN_ATTITUDE,  pylibnmea2k.PGN_FLUID,    pylibnmea2k.PGN_BATTERY,
        pylibnmea2k.PGN_SPEED,     pylibnmea2k.PGN_DEPTH,    pylibnmea2k.PGN_POS_RAPID,
        pylibnmea2k.PGN_COG_SOG,   pylibnmea2k.PGN_DATETIME, pylibnmea2k.PGN_XTE,
        pylibnmea2k.PGN_WIND,      pylibnmea2k.PGN_ENV,      pylibnmea2k.PGN_ENV_PARAMS,
    ],
    preferred_units={
        PhysicalQuantities.ANGLE:            "deg",
        PhysicalQuantities.SPEED:            "kts",
        PhysicalQuantities.TEMPERATURE:      "K",
        PhysicalQuantities.PRESSURE:         "Pa",
        PhysicalQuantities.LENGTH:           "m",
        PhysicalQuantities.ANGULAR_VELOCITY: "rad/s",
    },
)

# ── Helpers ───────────────────────────────────────────────────────────────────

def _ydwg(can_id: int, data: bytes) -> str:
    return f"12:00:00.000 R {can_id:08X} {' '.join(f'{b:02X}' for b in data)}"


def _field(msg, field_id):
    for f in msg.fields:
        if f.id == field_id:
            return f.value
    raise KeyError(field_id)


# CAN IDs: priority=2, source=1
_CAN = {
    pylibnmea2k.PGN_RUDDER:     0x09F10D01,
    pylibnmea2k.PGN_HEADING:    0x09F11201,
    pylibnmea2k.PGN_ROT:        0x09F11301,
    pylibnmea2k.PGN_ATTITUDE:   0x09F11901,
    pylibnmea2k.PGN_FLUID:      0x09F21101,
    pylibnmea2k.PGN_BATTERY:    0x09F21401,
    pylibnmea2k.PGN_SPEED:      0x09F50301,
    pylibnmea2k.PGN_DEPTH:      0x09F50B01,
    pylibnmea2k.PGN_POS_RAPID:  0x09F80101,
    pylibnmea2k.PGN_COG_SOG:    0x09F80201,
    pylibnmea2k.PGN_DATETIME:   0x09F80901,
    pylibnmea2k.PGN_XTE:        0x09F90301,
    pylibnmea2k.PGN_WIND:       0x09FD0201,
    pylibnmea2k.PGN_ENV:        0x09FD0601,
    pylibnmea2k.PGN_ENV_PARAMS: 0x09FD0701,
}

_MS_TO_KN = 1.94384


# ── PGN 129025 — Position Rapid Update ───────────────────────────────────────

class TestPosRapid(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_POS_RAPID],
                 struct.pack("<ii", int(-33.847 * 1e7), int(151.219 * 1e7)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.PosRapid)
        self.assertAlmostEqual(msg.lat, -33.847, delta=1e-6)
        self.assertAlmostEqual(msg.lon, 151.219, delta=1e-6)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.lat, _field(ref, "latitude"),  delta=1e-6)
        self.assertAlmostEqual(msg.lon, _field(ref, "longitude"), delta=1e-6)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_POS_RAPID],
                     struct.pack("<ii", 0x7FFFFFFF, 0x7FFFFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 129026 — COG & SOG ────────────────────────────────────────────────────

class TestCogSog(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_COG_SOG],
                 struct.pack("<BBHH", 0, 0,
                             int(math.radians(90.0) / 1e-4),
                             int((5.0 / _MS_TO_KN) / 0.01)) + b"\x00\x00")

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.CogSog)
        self.assertAlmostEqual(msg.cog_deg, 90.0, delta=0.01)
        self.assertAlmostEqual(msg.sog_kn,   5.0, delta=0.02)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.cog_deg, _field(ref, "cog"), delta=0.01)
        self.assertAlmostEqual(msg.sog_kn,  _field(ref, "sog"), delta=0.02)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_COG_SOG],
                     struct.pack("<BBHH", 0, 0, 0xFFFF, 0xFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 127250 — Vessel Heading ───────────────────────────────────────────────

class TestHeading(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_HEADING],
                 struct.pack("<BH", 0, int(math.radians(45.0) / 1e-4)) + b"\x00" * 5)

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Heading)
        self.assertAlmostEqual(msg.hdg_deg, 45.0, delta=0.01)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.hdg_deg, _field(ref, "heading"), delta=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_HEADING],
                     struct.pack("<BH", 0, 0xFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 127251 — Rate of Turn ─────────────────────────────────────────────────

class TestRot(unittest.TestCase):
    ROT_RAD_S = math.radians(10.0)
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ROT],
                 struct.pack("<Bi", 0, int(ROT_RAD_S / 3.125e-8)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Rot)
        self.assertAlmostEqual(msg.rot_rad_s, self.ROT_RAD_S, delta=1e-6)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.rot_rad_s, _field(ref, "rate"), delta=1e-6)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ROT],
                     struct.pack("<Bi", 0, 0x7FFFFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 127257 — Attitude ─────────────────────────────────────────────────────

class TestAttitude(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ATTITUDE],
                 struct.pack("<Bhhh",
                             0,
                             int(math.radians( 5.0) / 1e-4),
                             int(math.radians(-2.0) / 1e-4),
                             int(math.radians( 3.0) / 1e-4)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Attitude)
        self.assertAlmostEqual(msg.yaw_deg,    5.0, delta=0.01)
        self.assertAlmostEqual(msg.pitch_deg, -2.0, delta=0.01)
        self.assertAlmostEqual(msg.roll_deg,   3.0, delta=0.01)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.yaw_deg,   _field(ref, "yaw"),   delta=0.01)
        self.assertAlmostEqual(msg.pitch_deg, _field(ref, "pitch"), delta=0.01)
        self.assertAlmostEqual(msg.roll_deg,  _field(ref, "roll"),  delta=0.01)

    def test_partial_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ATTITUDE],
                     struct.pack("<Bhhh", 0, 0x7FFF,
                                 int(math.radians(-2.0) / 1e-4), 0x7FFF))
        msg = pylibnmea2k.decode(line)
        self.assertIsNotNone(msg)
        self.assertIsNone(msg.yaw_deg)
        self.assertAlmostEqual(msg.pitch_deg, -2.0, delta=0.01)
        self.assertIsNone(msg.roll_deg)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ATTITUDE],
                     struct.pack("<Bhhh", 0, 0x7FFF, 0x7FFF, 0x7FFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 128259 — Speed Through Water ─────────────────────────────────────────

class TestSpeed(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_SPEED],
                 struct.pack("<BH", 0, int((6.0 / _MS_TO_KN) / 0.01)) + b"\x00" * 5)

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Speed)
        self.assertAlmostEqual(msg.stw_kn, 6.0, delta=0.02)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.stw_kn, _field(ref, "speedWaterReferenced"), delta=0.02)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_SPEED],
                     struct.pack("<BH", 0, 0xFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 128267 — Water Depth ──────────────────────────────────────────────────

class TestDepth(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_DEPTH],
                 struct.pack("<BIh", 0, int(10.5 / 0.01), int(0.5 / 0.001)) + b"\x00")

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Depth)
        self.assertAlmostEqual(msg.depth_m,  10.5, delta=0.01)
        self.assertAlmostEqual(msg.offset_m,  0.5, delta=0.002)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.depth_m,  _field(ref, "depth"),  delta=0.01)
        self.assertAlmostEqual(msg.offset_m, _field(ref, "offset"), delta=0.002)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DEPTH],
                     struct.pack("<BIh", 0, 0xFFFFFFFF, 0))
        self.assertIsNone(pylibnmea2k.decode(line))

    def test_offset_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DEPTH],
                     struct.pack("<BIh", 0, int(5.0 / 0.01), 0x7FFF) + b"\x00")
        msg = pylibnmea2k.decode(line)
        self.assertIsNotNone(msg)
        self.assertIsNone(msg.offset_m)


# ── PGN 127245 — Rudder ───────────────────────────────────────────────────────

class TestRudder(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_RUDDER],
                 struct.pack("<BBhh", 0, 0, 0, int(math.radians(5.0) / 1e-4)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Rudder)
        self.assertAlmostEqual(msg.position_deg, 5.0, delta=0.01)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.position_deg, _field(ref, "position"), delta=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_RUDDER],
                     struct.pack("<BBhh", 0, 0, 0, 0x7FFF))
        msg = pylibnmea2k.decode(line)
        self.assertIsNotNone(msg)
        self.assertIsNone(msg.position_deg)


# ── PGN 130306 — Wind ─────────────────────────────────────────────────────────

class TestWind(unittest.TestCase):
    SPEED_MS = 5.0
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_WIND],
                 struct.pack("<BHHB", 0,
                             int(SPEED_MS / 0.01),
                             int(math.radians(45.0) / 1e-4),
                             2))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Wind)
        self.assertAlmostEqual(msg.speed_ms,  self.SPEED_MS, delta=0.01)
        self.assertAlmostEqual(msg.angle_deg, 45.0,          delta=0.01)
        self.assertEqual(msg.reference, 2)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.speed_ms * _MS_TO_KN, _field(ref, "windSpeed"), delta=0.02)
        self.assertAlmostEqual(msg.angle_deg,             _field(ref, "windAngle"), delta=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_WIND],
                     struct.pack("<BHHB", 0, 0xFFFF, 0xFFFF, 0))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 129033 — Date / Time ──────────────────────────────────────────────────

class TestDateTime(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_DATETIME],
                 struct.pack("<HIh", 19869, int(43200.0 / 1e-4), 0))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.DateTime)
        self.assertEqual(msg.date_days, 19869)
        self.assertAlmostEqual(msg.time_s, 43200.0, delta=0.001)
        self.assertEqual(msg.local_offset_min, 0)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        lib_date = _field(ref, "date")
        lib_time = _field(ref, "time")
        self.assertEqual(lib_date, _date(2024, 5, 26))
        self.assertEqual((lib_date - _date(1970, 1, 1)).days, msg.date_days)
        lib_time_s = lib_time.hour * 3600 + lib_time.minute * 60 + lib_time.second
        self.assertAlmostEqual(lib_time_s, msg.time_s, delta=1.0)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DATETIME],
                     struct.pack("<HIh", 0xFFFF, 0xFFFFFFFF, 0))
        self.assertIsNone(pylibnmea2k.decode(line))

    def test_local_offset_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DATETIME],
                     struct.pack("<HIh", 19869, int(43200.0 / 1e-4), 0x7FFF))
        msg = pylibnmea2k.decode(line)
        self.assertIsNotNone(msg)
        self.assertIsNone(msg.local_offset_min)


# ── PGN 129283 — Cross Track Error ───────────────────────────────────────────

class TestXte(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_XTE],
                 struct.pack("<BBi", 0, 0, int(15.0 / 0.01)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Xte)
        self.assertAlmostEqual(msg.xte_m, 15.0, delta=0.01)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.xte_m, _field(ref, "xte"), delta=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_XTE],
                     struct.pack("<BBi", 0, 0, 0x7FFFFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 130310 — Outside Environmental Parameters ────────────────────────────

class TestEnv(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ENV],
                 struct.pack("<BHHH", 0,
                             int(288.15 / 0.01),
                             int(293.15 / 0.01),
                             1013))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Env)
        self.assertAlmostEqual(msg.water_temp_k, 288.15, delta=0.02)
        self.assertAlmostEqual(msg.air_temp_k,   293.15, delta=0.02)
        self.assertAlmostEqual(msg.pressure_hpa, 1013.0, delta=0.5)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.water_temp_k,       _field(ref, "waterTemperature"),             delta=0.02)
        self.assertAlmostEqual(msg.air_temp_k,         _field(ref, "outsideAmbientAirTemperature"), delta=0.02)
        self.assertAlmostEqual(msg.pressure_hpa * 100, _field(ref, "atmosphericPressure"),          delta=50)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ENV],
                     struct.pack("<BHHH", 0, 0xFFFF, 0xFFFF, 0xFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 130311 — Environmental Parameters ────────────────────────────────────

class TestEnvParams(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ENV_PARAMS],
                 struct.pack("<BBHHH", 0, 0,
                             int(288.15 / 0.01),
                             int(60.0   / 0.004),
                             1013))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.EnvParams)
        self.assertAlmostEqual(msg.temp_k,       288.15, delta=0.02)
        self.assertAlmostEqual(msg.humidity_pct,  60.0,  delta=0.01)
        self.assertAlmostEqual(msg.pressure_hpa, 1013.0, delta=0.5)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.temp_k,             _field(ref, "temperature"),         delta=0.02)
        self.assertAlmostEqual(msg.humidity_pct,       _field(ref, "humidity"),            delta=0.01)
        self.assertAlmostEqual(msg.pressure_hpa * 100, _field(ref, "atmosphericPressure"), delta=50)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ENV_PARAMS],
                     struct.pack("<BBHHH", 0, 0, 0xFFFF, 0xFFFF, 0xFFFF))
        self.assertIsNone(pylibnmea2k.decode(line))


# ── PGN 127505 — Fluid Level ──────────────────────────────────────────────────

class TestFluidLevel(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_FLUID],
                 struct.pack("<BHI",
                             (1 << 4) | 0,        # instance=0, type=1 (Fresh Water)
                             int(75.0  / 0.004),
                             int(200.0 / 0.1)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.FluidLevel)
        self.assertEqual(msg.instance,   0)
        self.assertEqual(msg.fluid_type, 1)
        self.assertAlmostEqual(msg.level_pct,  75.0,  delta=0.01)
        self.assertAlmostEqual(msg.capacity_l, 200.0, delta=0.1)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.level_pct,  _field(ref, "level"),    delta=0.01)
        self.assertAlmostEqual(msg.capacity_l, _field(ref, "capacity"), delta=0.1)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_FLUID],
                     struct.pack("<BHI", 0, 0xFFFF, 0))
        self.assertIsNone(pylibnmea2k.decode(line))

    def test_capacity_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_FLUID],
                     struct.pack("<BHI", 0, int(50.0 / 0.004), 0xFFFFFFFF))
        msg = pylibnmea2k.decode(line)
        self.assertIsNotNone(msg)
        self.assertIsNone(msg.capacity_l)


# ── PGN 127508 — Battery Status ───────────────────────────────────────────────

class TestBattery(unittest.TestCase):
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_BATTERY],
                 struct.pack("<BHhH",
                             2,
                             int(12.6   / 0.01),
                             int(5.0    / 0.1),
                             int(298.15 / 0.01)) + b"\x00")

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        self.assertIsInstance(msg, pylibnmea2k.Battery)
        self.assertEqual(msg.instance, 2)
        self.assertAlmostEqual(msg.voltage_v, 12.6,   delta=0.01)
        self.assertAlmostEqual(msg.current_a,  5.0,   delta=0.1)
        self.assertAlmostEqual(msg.temp_k,    298.15, delta=0.02)

    def test_vs_library(self):
        ref = _lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        self.assertAlmostEqual(msg.voltage_v, _field(ref, "voltage"),     delta=0.01)
        self.assertAlmostEqual(msg.current_a, _field(ref, "current"),     delta=0.1)
        self.assertAlmostEqual(msg.temp_k,    _field(ref, "temperature"), delta=0.02)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_BATTERY],
                     struct.pack("<BHhH", 0, 0xFFFF, 0x7FFF, 0xFFFF) + b"\x00")
        self.assertIsNone(pylibnmea2k.decode(line))

    def test_partial_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_BATTERY],
                     struct.pack("<BHhH", 0, int(12.6 / 0.01), 0x7FFF, 0xFFFF) + b"\x00")
        msg = pylibnmea2k.decode(line)
        self.assertIsNotNone(msg)
        self.assertAlmostEqual(msg.voltage_v, 12.6, delta=0.01)
        self.assertIsNone(msg.current_a)
        self.assertIsNone(msg.temp_k)


# ── Malformed / unrecognised input ────────────────────────────────────────────

class TestMalformed(unittest.TestCase):
    def test_empty_string(self):
        self.assertIsNone(pylibnmea2k.decode(""))

    def test_garbage(self):
        self.assertIsNone(pylibnmea2k.decode("not a valid line"))

    def test_unknown_pgn(self):
        line = _ydwg(0x09FC1001, b"\x01\x02\x03\x04\x05\x06\x07\x08")
        self.assertIsNone(pylibnmea2k.decode(line))

    def test_truncated_data(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_POS_RAPID],
                     struct.pack("<i", int(-33.847 * 1e7)))
        self.assertIsNone(pylibnmea2k.decode(line))


if __name__ == "__main__":
    unittest.main()
