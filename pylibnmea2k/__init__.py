"""
pylibnmea2k — minimal NMEA 2000 decoder for marine sensor PGNs.

Decodes Yacht Devices YDWG-02 text format:
    'HH:MM:SS.mmm R 0xCANID DD DD DD DD DD DD DD DD'

Single-frame PGNs supported:
    127245  Rudder
    127250  Vessel Heading
    127251  Rate of Turn
    127257  Attitude (Roll / Pitch / Yaw)
    127258  Magnetic Variation
    127505  Fluid Level
    127508  Battery Status
    128259  Speed Through Water
    128267  Water Depth
    129025  Position Rapid Update
    129026  COG & SOG Rapid Update
    129033  Date / Time
    129283  Cross Track Error
    129291  Set & Drift Rapid Update
    130306  Wind (speed + angle + reference)
    130310  Outside Environmental Parameters
    130311  Environmental Parameters

Not supported (fast-packet / multi-frame):
    128275  Distance Log
    127506  DC Status
    129029  GNSS Position

None-return policy
    decode() returns None when:
      - the line is malformed or the PGN is unsupported
      - the primary measurement field(s) carry the "not available" sentinel
    It returns a dataclass with None fields when:
      - the frame carries a valid instance/secondary identifier but individual
        measurement fields are unavailable (Rudder, Depth, DateTime, FluidLevel,
        Battery, Attitude, Env, EnvParams, MagVariation)

Usage:
    import pylibnmea2k

    msg = pylibnmea2k.decode(line)          # stateless
    decoder = pylibnmea2k.Decoder()
    msg = decoder.decode(line)              # stateful wrapper (reserved for future fast-packet support)
"""

import math
import struct
from dataclasses import dataclass

__version__ = "0.2.4"
__all__ = [
    "Decoder", "decode",
    "PGN_RUDDER", "PGN_HEADING", "PGN_ROT", "PGN_ATTITUDE", "PGN_MAG_VAR",
    "PGN_FLUID", "PGN_BATTERY", "PGN_SPEED", "PGN_DEPTH",
    "PGN_POS_RAPID", "PGN_COG_SOG", "PGN_DATETIME", "PGN_XTE", "PGN_SET_DRIFT",
    "PGN_WIND", "PGN_ENV", "PGN_ENV_PARAMS",
    "Rudder", "Heading", "Rot", "Attitude", "MagVariation",
    "FluidLevel", "Battery", "Speed", "Depth",
    "PosRapid", "CogSog", "DateTime", "Xte", "SetDrift",
    "Wind", "Env", "EnvParams",
]

# ── PGN constants ─────────────────────────────────────────────────────────────

PGN_RUDDER    = 127245   # Rudder                           (single frame)
PGN_HEADING   = 127250   # Vessel Heading                   (single frame)
PGN_ROT       = 127251   # Rate of Turn                     (single frame)
PGN_ATTITUDE  = 127257   # Attitude — Roll / Pitch / Yaw   (single frame)
PGN_MAG_VAR   = 127258   # Magnetic Variation               (single frame)
PGN_FLUID     = 127505   # Fluid Level                      (single frame)
PGN_BATTERY   = 127508   # Battery Status                   (single frame)
PGN_SPEED     = 128259   # Speed Through Water              (single frame)
PGN_DEPTH     = 128267   # Water Depth                      (single frame)
PGN_POS_RAPID = 129025   # Position Rapid Update            (single frame)
PGN_COG_SOG   = 129026   # COG & SOG Rapid Update          (single frame)
PGN_DATETIME  = 129033   # Date / Time                      (single frame)
PGN_XTE       = 129283   # Cross Track Error                (single frame)
PGN_SET_DRIFT = 129291   # Set & Drift Rapid Update         (single frame)
PGN_WIND      = 130306   # Wind Data                        (single frame)
PGN_ENV       = 130310   # Outside Environmental Params     (single frame)
PGN_ENV_PARAMS= 130311   # Environmental Parameters         (single frame)

_RAD_TO_DEG = 180.0 / math.pi
_MS_TO_KN   = 1.94384    # m/s → knots


# ── Result types ──────────────────────────────────────────────────────────────

@dataclass(slots=True)
class Rudder:
    pgn:          int
    priority:     int
    source:       int
    instance:     int
    position_deg: float | None   # degrees, + = starboard; None = not available


@dataclass(slots=True)
class Heading:
    pgn:      int
    priority: int
    source:   int
    hdg_deg:  float        # degrees true, 0–360


@dataclass(slots=True)
class Rot:
    pgn:       int
    priority:  int
    source:    int
    rot_rad_s: float       # rad/s, + = turning clockwise (starboard)


@dataclass(slots=True)
class Attitude:
    pgn:       int
    priority:  int
    source:    int
    yaw_deg:   float | None
    pitch_deg: float | None   # + = bow up
    roll_deg:  float | None   # + = starboard down


@dataclass(slots=True)
class MagVariation:
    pgn:           int
    priority:      int
    source:        int
    var_source:    int         # 0=Manual 1=Chart 2=Table 3=Calculated 4=WMM2000 5=WMM2005 6=WMM2010 7=WMM2015 8=WMM2020
    variation_deg: float | None  # degrees, + = East; None = not available


@dataclass(slots=True)
class SetDrift:
    pgn:       int
    priority:  int
    source:    int
    set_deg:   float   # direction current flows toward, degrees, 0–360
    drift_kn:  float   # current speed, knots
    reference: int     # 0=True 1=Magnetic


@dataclass(slots=True)
class FluidLevel:
    pgn:        int
    priority:   int
    source:     int
    instance:   int
    fluid_type: int        # 0=Fuel 1=Fresh 2=Waste 3=LiveWell 4=Oil 5=BlackWater
    level_pct:  float      # 0–100 %
    capacity_l: float | None


@dataclass(slots=True)
class Battery:
    pgn:       int
    priority:  int
    source:    int
    instance:  int
    voltage_v: float | None
    current_a: float | None
    temp_k:    float | None


@dataclass(slots=True)
class Speed:
    pgn:      int
    priority: int
    source:   int
    stw_kn:   float        # speed through water, knots


@dataclass(slots=True)
class Depth:
    pgn:      int
    priority: int
    source:   int
    depth_m:  float
    offset_m: float | None   # transducer offset; None = not available


@dataclass(slots=True)
class PosRapid:
    pgn:      int
    priority: int
    source:   int
    lat:      float        # decimal degrees, + = N
    lon:      float        # decimal degrees, + = E


@dataclass(slots=True)
class CogSog:
    pgn:      int
    priority: int
    source:   int
    cog_deg:  float        # degrees true, 0–360
    sog_kn:   float        # knots


@dataclass(slots=True)
class DateTime:
    pgn:              int
    priority:         int
    source:           int
    date_days:        int        # days since 1970-01-01
    time_s:           float      # seconds since midnight UTC
    local_offset_min: int | None # UTC offset in minutes; None = not available


@dataclass(slots=True)
class Xte:
    pgn:      int
    priority: int
    source:   int
    xte_m:    float        # metres, + = right of course


@dataclass(slots=True)
class Wind:
    pgn:       int
    priority:  int
    source:    int
    speed_ms:  float       # m/s
    angle_deg: float       # 0–360
    reference: int         # 0=True(N) 1=Magnetic 2=Apparent 3=True(boat) 4=True(water)


@dataclass(slots=True)
class Env:
    pgn:          int
    priority:     int
    source:       int
    water_temp_k: float | None   # Kelvin
    air_temp_k:   float | None   # Kelvin
    pressure_hpa: float | None   # hPa


@dataclass(slots=True)
class EnvParams:
    pgn:             int
    priority:        int
    source:          int
    temp_source:     int           # bits 0–5 of byte 1
    humidity_source: int           # bits 6–7 of byte 1
    temp_k:          float | None  # Kelvin
    humidity_pct:    float | None  # %
    pressure_hpa:    float | None  # hPa


# ── Decoder class ─────────────────────────────────────────────────────────────

class Decoder:
    """Decode YDWG lines, optionally restricted to a set of PGNs and/or sources.

    Decoder()
        — decodes all supported PGNs from all sources
    Decoder(include_pgns={PGN_WIND})
        — only decodes Wind frames; all other PGNs return None after a fast dict miss
    Decoder(max_source_addr=200)
        — discards frames from source addresses >= 200 (e.g. gateway echo addresses)
    """

    def __init__(self, include_pgns=None, max_source_addr=None):
        self._decoders = (
            _DECODERS if include_pgns is None
            else {pgn: _DECODERS[pgn] for pgn in include_pgns if pgn in _DECODERS}
        )
        self._max_source_addr = max_source_addr

    def decode(self, line: str):
        """Decode one line. Returns a typed dataclass or None."""
        parsed = _parse_line(line)
        if parsed is None:
            return None
        pgn, priority, source, parts = parsed
        if self._max_source_addr is not None and source >= self._max_source_addr:
            return None
        handler = self._decoders.get(pgn)
        if handler:
            return handler(priority, source, parts)
        return None


# ── Module-level decode ───────────────────────────────────────────────────────

def decode(line: str):
    """Decode one YDWG text line. Returns a typed dataclass or None."""
    parsed = _parse_line(line)
    if parsed is None:
        return None
    pgn, priority, source, parts = parsed
    handler = _DECODERS.get(pgn)
    if handler:
        return handler(priority, source, parts)
    return None


# ── Line / CAN-ID parser ─────────────────────────────────────────────────────

def _parse_line(line: str):
    """Parse CAN ID fields. Returns (pgn, priority, source, parts) or None."""
    try:
        parts  = line.split(None, 3)
        can_id = int(parts[2], 16)
    except (IndexError, ValueError):
        return None
    dp       = (can_id >> 24) & 0x01
    pf       = (can_id >> 16) & 0xFF
    ps       = (can_id >>  8) & 0xFF
    pgn      = (dp << 16) | (pf << 8) | (ps if pf >= 240 else 0)
    priority = (can_id >> 26) & 0x07
    source   = can_id & 0xFF
    return pgn, priority, source, parts


# ── Data byte parser ──────────────────────────────────────────────────────────

def _data(parts) -> bytes | None:
    try:
        return bytes.fromhex(parts[3].replace(" ", ""))
    except (IndexError, ValueError):
        return None


# ── PGN decoders ─────────────────────────────────────────────────────────────

def _rudder(priority, source, parts):
    # Byte 0:    Instance
    # Byte 1:    Direction Order (2 bits) | reserved
    # Bytes 2–3: Angle Order  int16LE  1e-4 rad/bit
    # Bytes 4–5: Position     int16LE  1e-4 rad/bit  (0x7FFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 6:
        return None
    instance = d[0]
    pos_raw  = struct.unpack_from("<h", d, 4)[0]
    if pos_raw == 0x7FFF:
        return Rudder(PGN_RUDDER, priority, source, instance, None)
    return Rudder(PGN_RUDDER, priority, source, instance,
                  pos_raw * 1e-4 * _RAD_TO_DEG)


def _heading(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–2: Heading  uint16LE  1e-4 rad/bit
    d = _data(parts)
    if d is None or len(d) < 3:
        return None
    _, hdg_raw = struct.unpack_from("<BH", d, 0)
    if hdg_raw == 0xFFFF:
        return None
    return Heading(PGN_HEADING, priority, source,
                   hdg_raw * 1e-4 * _RAD_TO_DEG % 360.0)


def _rot(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–4: Rate of Turn  int32LE  3.125e-8 rad/s/bit  (0x7FFFFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 5:
        return None
    _, rot_raw = struct.unpack_from("<Bi", d, 0)
    if rot_raw == 0x7FFFFFFF:
        return None
    return Rot(PGN_ROT, priority, source, rot_raw * 3.125e-8)


def _attitude(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–2: Yaw    int16LE  1e-4 rad/bit  (0x7FFF = n/a)
    # Bytes 3–4: Pitch  int16LE  1e-4 rad/bit  (0x7FFF = n/a)
    # Bytes 5–6: Roll   int16LE  1e-4 rad/bit  (0x7FFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 7:
        return None
    _, yaw_raw, pitch_raw, roll_raw = struct.unpack_from("<Bhhh", d, 0)
    yaw   = None if yaw_raw   == 0x7FFF else yaw_raw   * 1e-4 * _RAD_TO_DEG
    pitch = None if pitch_raw == 0x7FFF else pitch_raw * 1e-4 * _RAD_TO_DEG
    roll  = None if roll_raw  == 0x7FFF else roll_raw  * 1e-4 * _RAD_TO_DEG
    if yaw is None and pitch is None and roll is None:
        return None
    return Attitude(PGN_ATTITUDE, priority, source, yaw, pitch, roll)


def _fluid(priority, source, parts):
    # Byte 0:    Instance (bits 0–3) | Type (bits 4–7)
    # Bytes 1–2: Level     uint16LE  0.004 %/bit  (0xFFFF = n/a)
    # Bytes 3–6: Capacity  uint32LE  0.1 L/bit    (0xFFFFFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 7:
        return None
    b0, lvl_raw, cap_raw = struct.unpack_from("<BHI", d, 0)
    if lvl_raw == 0xFFFF:
        return None
    cap = None if cap_raw == 0xFFFFFFFF else cap_raw * 0.1
    return FluidLevel(PGN_FLUID, priority, source,
                      b0 & 0x0F, (b0 >> 4) & 0x0F,
                      lvl_raw * 0.004, cap)


def _battery(priority, source, parts):
    # Byte 0:    Instance
    # Bytes 1–2: Voltage  uint16LE  0.01 V/bit   (0xFFFF = n/a)
    # Bytes 3–4: Current  int16LE   0.1 A/bit    (0x7FFF = n/a)
    # Bytes 5–6: Temp     uint16LE  0.01 K/bit   (0xFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 7:
        return None
    instance, v_raw, c_raw, t_raw = struct.unpack_from("<BHhH", d, 0)
    v = None if v_raw == 0xFFFF else v_raw * 0.01
    c = None if c_raw == 0x7FFF else c_raw * 0.1
    t = None if t_raw == 0xFFFF else t_raw * 0.01
    if v is None and c is None and t is None:
        return None
    return Battery(PGN_BATTERY, priority, source, instance, v, c, t)


def _speed(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–2: Speed Water Referenced  uint16LE  0.01 m/s per bit
    d = _data(parts)
    if d is None or len(d) < 3:
        return None
    _, spd_raw = struct.unpack_from("<BH", d, 0)
    if spd_raw == 0xFFFF:
        return None
    return Speed(PGN_SPEED, priority, source, spd_raw * 0.01 * _MS_TO_KN)


def _depth(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–4: Depth   uint32LE  0.01 m/bit    (0xFFFFFFFF = n/a)
    # Bytes 5–6: Offset  int16LE   0.001 m/bit   (0x7FFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 7:
        return None
    _, depth_raw, offset_raw = struct.unpack_from("<BIh", d, 0)
    if depth_raw == 0xFFFFFFFF:
        return None
    offset = None if offset_raw == 0x7FFF else offset_raw * 0.001
    return Depth(PGN_DEPTH, priority, source, depth_raw * 0.01, offset)


def _pos_rapid(priority, source, parts):
    # Bytes 0–3: latitude  int32LE  1e-7 deg/bit
    # Bytes 4–7: longitude int32LE  1e-7 deg/bit
    d = _data(parts)
    if d is None or len(d) < 8:
        return None
    lat_raw, lon_raw = struct.unpack_from("<ii", d, 0)
    if lat_raw == 0x7FFFFFFF or lon_raw == 0x7FFFFFFF:
        return None
    return PosRapid(PGN_POS_RAPID, priority, source,
                    lat_raw * 1e-7, lon_raw * 1e-7)


def _cog_sog(priority, source, parts):
    # Byte 0:    SID
    # Byte 1:    COG reference (2 bits) | reserved
    # Bytes 2–3: COG  uint16LE  1e-4 rad/bit
    # Bytes 4–5: SOG  uint16LE  0.01 m/s per bit
    d = _data(parts)
    if d is None or len(d) < 6:
        return None
    _, _, cog_raw, sog_raw = struct.unpack_from("<BBHH", d, 0)
    if cog_raw == 0xFFFF or sog_raw == 0xFFFF:
        return None
    return CogSog(PGN_COG_SOG, priority, source,
                  cog_raw * 1e-4 * _RAD_TO_DEG % 360.0,
                  sog_raw * 0.01 * _MS_TO_KN)


def _datetime(priority, source, parts):
    # Bytes 0–1: Date          uint16LE  days since 1970-01-01  (0xFFFF = n/a)
    # Bytes 2–5: Time          uint32LE  0.0001 s/bit           (0xFFFFFFFF = n/a)
    # Bytes 6–7: Local Offset  int16LE   minutes                (0x7FFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 8:
        return None
    date_raw, time_raw, off_raw = struct.unpack_from("<HIh", d, 0)
    if date_raw == 0xFFFF or time_raw == 0xFFFFFFFF:
        return None
    offset = None if off_raw == 0x7FFF else int(off_raw)
    return DateTime(PGN_DATETIME, priority, source,
                    int(date_raw), time_raw * 1e-4, offset)


def _xte(priority, source, parts):
    # Byte 0:    SID
    # Byte 1:    XTE Mode (bits 0–3) | Nav Terminated (bit 4)
    # Bytes 2–5: XTE  int32LE  0.01 m/bit  (0x7FFFFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 6:
        return None
    _, _, xte_raw = struct.unpack_from("<BBi", d, 0)
    if xte_raw == 0x7FFFFFFF:
        return None
    return Xte(PGN_XTE, priority, source, xte_raw * 0.01)


def _wind(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–2: Wind Speed  uint16LE  0.01 m/s/bit   (0xFFFF = n/a)
    # Bytes 3–4: Wind Angle  uint16LE  1e-4 rad/bit   (0xFFFF = n/a)
    # Byte 5:    Reference   bits 0–2  (2 = apparent)
    d = _data(parts)
    if d is None or len(d) < 6:
        return None
    _, spd_raw, ang_raw, ref_byte = struct.unpack_from("<BHHB", d, 0)
    if spd_raw == 0xFFFF or ang_raw == 0xFFFF:
        return None
    return Wind(PGN_WIND, priority, source,
                spd_raw * 0.01,
                ang_raw * 1e-4 * _RAD_TO_DEG % 360.0,
                ref_byte & 0x07)


def _env(priority, source, parts):
    # Byte 0:    SID
    # Bytes 1–2: Water Temp  uint16LE  0.01 K/bit   (0xFFFF = n/a)
    # Bytes 3–4: Air Temp    uint16LE  0.01 K/bit   (0xFFFF = n/a)
    # Bytes 5–6: Pressure    uint16LE  100 Pa/bit → hPa  (0xFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 7:
        return None
    _, wt_raw, at_raw, pr_raw = struct.unpack_from("<BHHH", d, 0)
    wt = None if wt_raw == 0xFFFF else wt_raw * 0.01
    at = None if at_raw == 0xFFFF else at_raw * 0.01
    pr = None if pr_raw == 0xFFFF else float(pr_raw)
    if wt is None and at is None and pr is None:
        return None
    return Env(PGN_ENV, priority, source, wt, at, pr)


def _mag_variation(priority, source, parts):
    # Byte 0:    SID
    # Byte 1:    Variation Source (bits 0–3) | reserved
    # Bytes 2–3: Age of Service  uint16LE  days since 1970-01-01
    # Bytes 4–5: Variation       int16LE   1e-4 rad/bit  (0x7FFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 6:
        return None
    _, src_byte, _, var_raw = struct.unpack_from("<BBHh", d, 0)
    var = None if var_raw == 0x7FFF else var_raw * 1e-4 * _RAD_TO_DEG
    return MagVariation(PGN_MAG_VAR, priority, source, src_byte & 0x0F, var)


def _set_drift(priority, source, parts):
    # Byte 0:    SID
    # Byte 1:    Set Reference (bits 0–1) | reserved
    # Bytes 2–3: Set    uint16LE  1e-4 rad/bit   (0xFFFF = n/a)
    # Bytes 4–5: Drift  uint16LE  0.01 m/s/bit   (0xFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 6:
        return None
    _, ref_byte, set_raw, drift_raw = struct.unpack_from("<BBHH", d, 0)
    if set_raw == 0xFFFF or drift_raw == 0xFFFF:
        return None
    return SetDrift(PGN_SET_DRIFT, priority, source,
                    set_raw * 1e-4 * _RAD_TO_DEG % 360.0,
                    drift_raw * 0.01 * _MS_TO_KN,
                    ref_byte & 0x03)


def _env_params(priority, source, parts):
    # Byte 0:    SID
    # Byte 1:    Temp Source (bits 0–5) | Humidity Source (bits 6–7)
    # Bytes 2–3: Temperature  uint16LE  0.01 K/bit    (0xFFFF = n/a)
    # Bytes 4–5: Humidity     uint16LE  0.004 %/bit   (0xFFFF = n/a)
    # Bytes 6–7: Pressure     uint16LE  100 Pa/bit → hPa  (0xFFFF = n/a)
    d = _data(parts)
    if d is None or len(d) < 8:
        return None
    _, src_byte, t_raw, h_raw, p_raw = struct.unpack_from("<BBHHH", d, 0)
    t = None if t_raw == 0xFFFF else t_raw * 0.01
    h = None if h_raw == 0xFFFF else h_raw * 0.004
    p = None if p_raw == 0xFFFF else float(p_raw)
    if t is None and h is None and p is None:
        return None
    return EnvParams(PGN_ENV_PARAMS, priority, source,
                     src_byte & 0x3F, (src_byte >> 6) & 0x03, t, h, p)


# ── PGN dispatch table ────────────────────────────────────────────────────────

_DECODERS = {
    PGN_RUDDER:     _rudder,
    PGN_HEADING:    _heading,
    PGN_ROT:        _rot,
    PGN_ATTITUDE:   _attitude,
    PGN_MAG_VAR:    _mag_variation,
    PGN_FLUID:      _fluid,
    PGN_BATTERY:    _battery,
    PGN_SPEED:      _speed,
    PGN_DEPTH:      _depth,
    PGN_POS_RAPID:  _pos_rapid,
    PGN_COG_SOG:    _cog_sog,
    PGN_DATETIME:   _datetime,
    PGN_XTE:        _xte,
    PGN_SET_DRIFT:  _set_drift,
    PGN_WIND:       _wind,
    PGN_ENV:        _env,
    PGN_ENV_PARAMS: _env_params,
}
