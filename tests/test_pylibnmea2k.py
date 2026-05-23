"""
Cross-validate pylibnmea2k against the nmea2000 reference library.

Each test encodes known values into a synthetic YDWG line, decodes with
both libraries, and asserts they agree within quantisation tolerance.
Sentinel tests verify that "not available" bit patterns return None.
"""

import math
import struct
from datetime import date as _date, time as _dtime

import pytest
from nmea2000.consts import PhysicalQuantities
from nmea2000.decoder import NMEA2000Decoder

import pylibnmea2k

# ── Helpers ───────────────────────────────────────────────────────────────────

def _ydwg(can_id: int, data: bytes) -> str:
    return f"12:00:00.000 R {can_id:08X} {' '.join(f'{b:02X}' for b in data)}"


def _field(msg, field_id):
    for f in msg.fields:
        if f.id == field_id:
            return f.value
    raise KeyError(field_id)


# CAN IDs: priority=2, source=1, per-PGN dp/pf/ps from NMEA 2000 spec.
# Formula: (2<<26)|(dp<<24)|(pf<<16)|(ps<<8)|source
_CAN = {
    pylibnmea2k.PGN_RUDDER:    0x09F10D01,  # dp=1 pf=241 ps=13
    pylibnmea2k.PGN_HEADING:   0x09F11201,  # dp=1 pf=241 ps=18
    pylibnmea2k.PGN_ROT:       0x09F11301,  # dp=1 pf=241 ps=19
    pylibnmea2k.PGN_ATTITUDE:  0x09F11901,  # dp=1 pf=241 ps=25
    pylibnmea2k.PGN_FLUID:     0x09F21101,  # dp=1 pf=242 ps=17
    pylibnmea2k.PGN_BATTERY:   0x09F21401,  # dp=1 pf=242 ps=20
    pylibnmea2k.PGN_SPEED:     0x09F50301,  # dp=1 pf=245 ps=3
    pylibnmea2k.PGN_DEPTH:     0x09F50B01,  # dp=1 pf=245 ps=11
    pylibnmea2k.PGN_POS_RAPID: 0x09F80101,  # dp=1 pf=248 ps=1
    pylibnmea2k.PGN_COG_SOG:   0x09F80201,  # dp=1 pf=248 ps=2
    pylibnmea2k.PGN_DATETIME:  0x09F80901,  # dp=1 pf=248 ps=9
    pylibnmea2k.PGN_XTE:       0x09F90301,  # dp=1 pf=249 ps=3
    pylibnmea2k.PGN_WIND:      0x09FD0201,  # dp=1 pf=253 ps=2
    pylibnmea2k.PGN_ENV:       0x09FD0601,  # dp=1 pf=253 ps=6
    pylibnmea2k.PGN_ENV_PARAMS:0x09FD0701,  # dp=1 pf=253 ps=7
}

_MS_TO_KN = 1.94384


@pytest.fixture(scope="module")
def lib():
    return NMEA2000Decoder(
        include_pgns=list(_CAN.keys()),
        preferred_units={
            PhysicalQuantities.ANGLE:            "deg",
            PhysicalQuantities.SPEED:            "kts",
            PhysicalQuantities.TEMPERATURE:      "K",
            PhysicalQuantities.PRESSURE:         "Pa",
            PhysicalQuantities.LENGTH:           "m",
            PhysicalQuantities.ANGULAR_VELOCITY: "rad/s",
        },
    )


# ── PGN 129025 — Position Rapid Update ───────────────────────────────────────

class TestPosRapid:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_POS_RAPID],
                 struct.pack("<ii", int(-33.847 * 1e7), int(151.219 * 1e7)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.PosRapid)
        assert msg.lat == pytest.approx(-33.847, abs=1e-6)
        assert msg.lon == pytest.approx(151.219, abs=1e-6)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.lat == pytest.approx(_field(ref, "latitude"),  abs=1e-6)
        assert msg.lon == pytest.approx(_field(ref, "longitude"), abs=1e-6)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_POS_RAPID],
                     struct.pack("<ii", 0x7FFFFFFF, 0x7FFFFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 129026 — COG & SOG ────────────────────────────────────────────────────

class TestCogSog:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_COG_SOG],
                 struct.pack("<BBHH", 0, 0,
                             int(math.radians(90.0) / 1e-4),
                             int((5.0 / _MS_TO_KN) / 0.01)) + b"\x00\x00")

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.CogSog)
        assert msg.cog_deg == pytest.approx(90.0, abs=0.01)
        assert msg.sog_kn  == pytest.approx(5.0,  abs=0.02)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.cog_deg == pytest.approx(_field(ref, "cog"), abs=0.01)
        assert msg.sog_kn  == pytest.approx(_field(ref, "sog"), abs=0.02)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_COG_SOG],
                     struct.pack("<BBHH", 0, 0, 0xFFFF, 0xFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 127250 — Vessel Heading ───────────────────────────────────────────────

class TestHeading:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_HEADING],
                 struct.pack("<BH", 0, int(math.radians(45.0) / 1e-4)) + b"\x00" * 5)

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Heading)
        assert msg.hdg_deg == pytest.approx(45.0, abs=0.01)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.hdg_deg == pytest.approx(_field(ref, "heading"), abs=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_HEADING],
                     struct.pack("<BH", 0, 0xFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 127251 — Rate of Turn ─────────────────────────────────────────────────

class TestRot:
    ROT_RAD_S = math.radians(10.0)
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ROT],
                 struct.pack("<Bi", 0, int(ROT_RAD_S / 3.125e-8)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Rot)
        assert msg.rot_rad_s == pytest.approx(self.ROT_RAD_S, rel=1e-4)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.rot_rad_s == pytest.approx(_field(ref, "rate"), rel=1e-4)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ROT],
                     struct.pack("<Bi", 0, 0x7FFFFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 127257 — Attitude ─────────────────────────────────────────────────────

class TestAttitude:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ATTITUDE],
                 struct.pack("<Bhhh",
                             0,
                             int(math.radians(5.0)  / 1e-4),
                             int(math.radians(-2.0) / 1e-4),
                             int(math.radians(3.0)  / 1e-4)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Attitude)
        assert msg.yaw_deg   == pytest.approx(5.0,  abs=0.01)
        assert msg.pitch_deg == pytest.approx(-2.0, abs=0.01)
        assert msg.roll_deg  == pytest.approx(3.0,  abs=0.01)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.yaw_deg   == pytest.approx(_field(ref, "yaw"),   abs=0.01)
        assert msg.pitch_deg == pytest.approx(_field(ref, "pitch"), abs=0.01)
        assert msg.roll_deg  == pytest.approx(_field(ref, "roll"),  abs=0.01)

    def test_partial_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ATTITUDE],
                     struct.pack("<Bhhh", 0, 0x7FFF, int(math.radians(-2.0) / 1e-4), 0x7FFF))
        msg = pylibnmea2k.decode(line)
        assert msg is not None
        assert msg.yaw_deg  is None
        assert msg.pitch_deg == pytest.approx(-2.0, abs=0.01)
        assert msg.roll_deg  is None

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ATTITUDE],
                     struct.pack("<Bhhh", 0, 0x7FFF, 0x7FFF, 0x7FFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 128259 — Speed Through Water ─────────────────────────────────────────

class TestSpeed:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_SPEED],
                 struct.pack("<BH", 0, int((6.0 / _MS_TO_KN) / 0.01)) + b"\x00" * 5)

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Speed)
        assert msg.stw_kn == pytest.approx(6.0, abs=0.02)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.stw_kn == pytest.approx(_field(ref, "speedWaterReferenced"), abs=0.02)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_SPEED],
                     struct.pack("<BH", 0, 0xFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 128267 — Water Depth ──────────────────────────────────────────────────

class TestDepth:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_DEPTH],
                 struct.pack("<BIh", 0, int(10.5 / 0.01), int(0.5 / 0.001)) + b"\x00")

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Depth)
        assert msg.depth_m  == pytest.approx(10.5, abs=0.01)
        assert msg.offset_m == pytest.approx(0.5,  abs=0.002)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.depth_m  == pytest.approx(_field(ref, "depth"),  abs=0.01)
        assert msg.offset_m == pytest.approx(_field(ref, "offset"), abs=0.002)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DEPTH],
                     struct.pack("<BIh", 0, 0xFFFFFFFF, 0))
        assert pylibnmea2k.decode(line) is None

    def test_offset_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DEPTH],
                     struct.pack("<BIh", 0, int(5.0 / 0.01), 0x7FFF) + b"\x00")
        msg = pylibnmea2k.decode(line)
        assert msg is not None
        assert msg.offset_m is None


# ── PGN 127245 — Rudder ───────────────────────────────────────────────────────

class TestRudder:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_RUDDER],
                 struct.pack("<BBhh", 0, 0, 0, int(math.radians(5.0) / 1e-4)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Rudder)
        assert msg.position_deg == pytest.approx(5.0, abs=0.01)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.position_deg == pytest.approx(_field(ref, "position"), abs=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_RUDDER],
                     struct.pack("<BBhh", 0, 0, 0, 0x7FFF))
        msg = pylibnmea2k.decode(line)
        assert msg is not None
        assert msg.position_deg is None


# ── PGN 130306 — Wind ─────────────────────────────────────────────────────────

class TestWind:
    SPEED_MS = 5.0
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_WIND],
                 struct.pack("<BHHB",
                             0,
                             int(SPEED_MS / 0.01),
                             int(math.radians(45.0) / 1e-4),
                             2))  # reference = Apparent

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Wind)
        assert msg.speed_ms  == pytest.approx(self.SPEED_MS, abs=0.01)
        assert msg.angle_deg == pytest.approx(45.0,          abs=0.01)
        assert msg.reference == 2

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        # library returns speed in kts
        assert msg.speed_ms * _MS_TO_KN == pytest.approx(_field(ref, "windSpeed"), abs=0.02)
        assert msg.angle_deg             == pytest.approx(_field(ref, "windAngle"), abs=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_WIND],
                     struct.pack("<BHHB", 0, 0xFFFF, 0xFFFF, 0))
        assert pylibnmea2k.decode(line) is None


# ── PGN 129033 — Date / Time ──────────────────────────────────────────────────

class TestDateTime:
    # 19869 days = 2024-05-26; 43200 s = 12:00:00 UTC
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_DATETIME],
                 struct.pack("<HIh", 19869, int(43200.0 / 1e-4), 0))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.DateTime)
        assert msg.date_days == 19869
        assert msg.time_s    == pytest.approx(43200.0, abs=0.001)
        assert msg.local_offset_min == 0

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        lib_date = _field(ref, "date")
        lib_time = _field(ref, "time")
        # library returns datetime.date / datetime.time objects
        assert lib_date == _date(2024, 5, 26)
        assert lib_time == _dtime(12, 0, 0)
        # cross-check our values match
        from datetime import date as _d
        epoch = _d(1970, 1, 1)
        assert (lib_date - epoch).days == msg.date_days
        assert lib_time.hour * 3600 + lib_time.minute * 60 + lib_time.second == pytest.approx(msg.time_s, abs=1.0)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DATETIME],
                     struct.pack("<HIh", 0xFFFF, 0xFFFFFFFF, 0))
        assert pylibnmea2k.decode(line) is None

    def test_local_offset_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_DATETIME],
                     struct.pack("<HIh", 19869, int(43200.0 / 1e-4), 0x7FFF))
        msg = pylibnmea2k.decode(line)
        assert msg is not None
        assert msg.local_offset_min is None


# ── PGN 129283 — Cross Track Error ───────────────────────────────────────────

class TestXte:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_XTE],
                 struct.pack("<BBi", 0, 0, int(15.0 / 0.01)))

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Xte)
        assert msg.xte_m == pytest.approx(15.0, abs=0.01)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.xte_m == pytest.approx(_field(ref, "xte"), abs=0.01)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_XTE],
                     struct.pack("<BBi", 0, 0, 0x7FFFFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 130310 — Outside Environmental Parameters ────────────────────────────

class TestEnv:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ENV],
                 struct.pack("<BHHH",
                             0,
                             int(288.15 / 0.01),  # water temp
                             int(293.15 / 0.01),  # air temp
                             1013))               # pressure hPa (100 Pa/bit)

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Env)
        assert msg.water_temp_k == pytest.approx(288.15, abs=0.02)
        assert msg.air_temp_k   == pytest.approx(293.15, abs=0.02)
        assert msg.pressure_hpa == pytest.approx(1013.0, abs=0.5)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        # library returns pressure in Pa; ours in hPa (100 Pa/bit → value = hPa)
        assert msg.water_temp_k       == pytest.approx(_field(ref, "waterTemperature"),             abs=0.02)
        assert msg.air_temp_k         == pytest.approx(_field(ref, "outsideAmbientAirTemperature"), abs=0.02)
        assert msg.pressure_hpa * 100 == pytest.approx(_field(ref, "atmosphericPressure"),          abs=50)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ENV],
                     struct.pack("<BHHH", 0, 0xFFFF, 0xFFFF, 0xFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 130311 — Environmental Parameters ────────────────────────────────────

class TestEnvParams:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_ENV_PARAMS],
                 struct.pack("<BBHHH",
                             0,
                             0,                   # temp_source=Sea, humidity_source=Inside
                             int(288.15 / 0.01),  # temp
                             int(60.0  / 0.004),  # humidity
                             1013))               # pressure hPa

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.EnvParams)
        assert msg.temp_k        == pytest.approx(288.15, abs=0.02)
        assert msg.humidity_pct  == pytest.approx(60.0,   abs=0.01)
        assert msg.pressure_hpa  == pytest.approx(1013.0, abs=0.5)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.temp_k              == pytest.approx(_field(ref, "temperature"),         abs=0.02)
        assert msg.humidity_pct        == pytest.approx(_field(ref, "humidity"),            abs=0.01)
        assert msg.pressure_hpa * 100  == pytest.approx(_field(ref, "atmosphericPressure"), abs=50)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_ENV_PARAMS],
                     struct.pack("<BBHHH", 0, 0, 0xFFFF, 0xFFFF, 0xFFFF))
        assert pylibnmea2k.decode(line) is None


# ── PGN 127505 — Fluid Level ──────────────────────────────────────────────────

class TestFluidLevel:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_FLUID],
                 struct.pack("<BHI",
                             (1 << 4) | 0,       # instance=0, type=1 (Fresh Water)
                             int(75.0 / 0.004),  # 75%
                             int(200.0 / 0.1)))   # 200 L

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.FluidLevel)
        assert msg.instance   == 0
        assert msg.fluid_type == 1
        assert msg.level_pct  == pytest.approx(75.0,  abs=0.01)
        assert msg.capacity_l == pytest.approx(200.0, abs=0.1)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.level_pct  == pytest.approx(_field(ref, "level"),    abs=0.01)
        assert msg.capacity_l == pytest.approx(_field(ref, "capacity"), abs=0.1)

    def test_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_FLUID],
                     struct.pack("<BHI", 0, 0xFFFF, 0))
        assert pylibnmea2k.decode(line) is None

    def test_capacity_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_FLUID],
                     struct.pack("<BHI", 0, int(50.0 / 0.004), 0xFFFFFFFF))
        msg = pylibnmea2k.decode(line)
        assert msg is not None
        assert msg.capacity_l is None


# ── PGN 127508 — Battery Status ───────────────────────────────────────────────

class TestBattery:
    LINE = _ydwg(_CAN[pylibnmea2k.PGN_BATTERY],
                 struct.pack("<BHhH",
                             2,                   # instance=2
                             int(12.6 / 0.01),   # voltage
                             int(5.0  / 0.1),    # current
                             int(298.15 / 0.01)) # temp
                 + b"\x00")

    def test_decode(self):
        msg = pylibnmea2k.decode(self.LINE)
        assert isinstance(msg, pylibnmea2k.Battery)
        assert msg.instance  == 2
        assert msg.voltage_v == pytest.approx(12.6,   abs=0.01)
        assert msg.current_a == pytest.approx(5.0,    abs=0.1)
        assert msg.temp_k    == pytest.approx(298.15, abs=0.02)

    def test_vs_library(self, lib):
        ref = lib.decode(self.LINE)
        msg = pylibnmea2k.decode(self.LINE)
        assert msg.voltage_v == pytest.approx(_field(ref, "voltage"),     abs=0.01)
        assert msg.current_a == pytest.approx(_field(ref, "current"),     abs=0.1)
        assert msg.temp_k    == pytest.approx(_field(ref, "temperature"), abs=0.02)

    def test_all_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_BATTERY],
                     struct.pack("<BHhH", 0, 0xFFFF, 0x7FFF, 0xFFFF) + b"\x00")
        assert pylibnmea2k.decode(line) is None

    def test_partial_not_available(self):
        line = _ydwg(_CAN[pylibnmea2k.PGN_BATTERY],
                     struct.pack("<BHhH", 0, int(12.6 / 0.01), 0x7FFF, 0xFFFF) + b"\x00")
        msg = pylibnmea2k.decode(line)
        assert msg is not None
        assert msg.voltage_v == pytest.approx(12.6, abs=0.01)
        assert msg.current_a is None
        assert msg.temp_k    is None


# ── Malformed / unrecognised input ────────────────────────────────────────────

class TestMalformed:
    def test_empty_string(self):
        assert pylibnmea2k.decode("") is None

    def test_garbage(self):
        assert pylibnmea2k.decode("not a valid line") is None

    def test_unknown_pgn(self):
        # PGN 130000 — not in our decoder
        line = _ydwg(0x09FC1001, b"\x01\x02\x03\x04\x05\x06\x07\x08")
        assert pylibnmea2k.decode(line) is None

    def test_truncated_data(self):
        # Valid PGN 129025 but only 4 bytes of data instead of 8
        line = _ydwg(_CAN[pylibnmea2k.PGN_POS_RAPID],
                     struct.pack("<i", int(-33.847 * 1e7)))
        assert pylibnmea2k.decode(line) is None
