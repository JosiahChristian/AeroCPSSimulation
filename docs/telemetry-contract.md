# AeroCPS Telemetry Contract

## Version

`aerocps.telemetry.v1`

Every data row carries the schema version so downstream consumers can reject incompatible input explicitly instead of silently misreading reordered or renamed fields.

## CSV Fields

| Field | Type | Unit | Meaning |
|---|---:|---|---|
| `schema_version` | string | — | Contract identifier |
| `step` | integer | — | One-based integration step |
| `time_s` | number | seconds | Simulated elapsed time |
| `altitude_m` | number | meters | Current vertical position |
| `velocity_mps` | number | meters/second | Current vertical velocity |
| `target_altitude_m` | number | meters | Configured altitude command |
| `gravity_mps2` | number | meters/second² | Configured signed gravitational acceleration |

## Compatibility Policy

- Field meanings and units do not change within `v1`.
- New optional fields require a new documented schema version if their placement changes the CSV layout.
- Consumers such as AeroCPSTelemetry should verify `schema_version` before parsing numeric fields.
- Non-finite numeric values are invalid.

## Fixed-Wing Six-Degree-of-Freedom Contract

The `run_fixed_wing` executable emits `aerocps.fixed-wing.v1`. Each row records:

- scenario step and elapsed time
- world-frame position and velocity in SI units
- the normalized body-to-world quaternion in `w, x, y, z` order
- body-frame roll, pitch, and yaw rates in radians per second
- wind-relative airspeed, angle of attack, and sideslip angle

The exact header is defined by `FixedWingTelemetryContract.hpp`. The same
compatibility rule applies: additive fields are permitted, but removing,
renaming, reordering, or changing a required field requires a new schema
version.
