# pylibnmea2k

Minimal, fast NMEA 2000 single-frame PGN decoder for Python.

Decodes **Yacht Devices YDWG-02** text format:
```
HH:MM:SS.mmm R 0xCANID DD DD DD DD DD DD DD DD
```

Returns typed dataclasses. No dependencies beyond the standard library.

## Install

```bash
pip install pylibnmea2k
```

## Usage

```python
import pylibnmea2k

# Stateless
msg = pylibnmea2k.decode(line)

# Stateful wrapper (same interface)
decoder = pylibnmea2k.Decoder()
msg = decoder.decode(line)

# Filter to specific PGNs — non-matching frames cost ~0.5 µs and return None
decoder = pylibnmea2k.Decoder(include_pgns={pylibnmea2k.PGN_WIND, pylibnmea2k.PGN_COG_SOG})
msg = decoder.decode(line)

# Discard frames from source addresses >= threshold (e.g. gateway/bridge echo addresses)
decoder = pylibnmea2k.Decoder(max_source_addr=200)
msg = decoder.decode(line)
```

`decode()` returns a typed dataclass or `None` if the PGN is unsupported or fields are not available.

## Supported PGNs

| PGN    | Name                            | Key fields |
|--------|---------------------------------|------------|
| 127245 | Rudder                          | `position_deg` |
| 127250 | Vessel Heading                  | `hdg_deg` |
| 127251 | Rate of Turn                    | `rot_rad_s` |
| 127257 | Attitude                        | `yaw_deg`, `pitch_deg`, `roll_deg` |
| 127258 | Magnetic Variation              | `variation_deg`, `var_source` |
| 127505 | Fluid Level                     | `level_pct`, `capacity_l`, `fluid_type` |
| 127508 | Battery Status                  | `voltage_v`, `current_a`, `temp_k` |
| 128259 | Speed Through Water             | `stw_kn` |
| 128267 | Water Depth                     | `depth_m`, `offset_m` |
| 129025 | Position Rapid Update           | `lat`, `lon` |
| 129026 | COG & SOG Rapid Update          | `cog_deg`, `sog_kn` |
| 129033 | Date / Time                     | `date_days`, `time_s`, `local_offset_min` |
| 129283 | Cross Track Error               | `xte_m` |
| 129291 | Set & Drift Rapid Update        | `set_deg`, `drift_kn`, `reference` |
| 130306 | Wind                            | `speed_ms`, `angle_deg`, `reference` |
| 130310 | Outside Environmental Params    | `water_temp_k`, `air_temp_k`, `pressure_hpa` |
| 130311 | Environmental Parameters        | `temp_source`, `humidity_source`, `temp_k`, `humidity_pct`, `pressure_hpa` |

Fast-packet (multi-frame) PGNs are not currently supported.

## Result types

All dataclasses include `pgn`, `priority`, and `source` fields. Optional fields are `float | None` — `None` means the device reported "not available".

```python
msg = pylibnmea2k.decode(line)
if isinstance(msg, pylibnmea2k.CogSog):
    print(f"COG {msg.cog_deg:.1f}°  SOG {msg.sog_kn:.1f} kn")

if isinstance(msg, pylibnmea2k.Wind):
    ref = ["True(N)", "Magnetic", "Apparent", "True(boat)", "True(water)"][msg.reference]
    print(f"Wind {msg.speed_ms:.1f} m/s at {msg.angle_deg:.1f}° ({ref})")
```

## Performance

Struct-based decoding with no library dependencies. On a typical ARM Cortex-A7 at 650 MHz, decoding a matching PGN takes ~30–45 µs; discarding a non-matching frame takes ~0.5 µs.

## License

MIT
