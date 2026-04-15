# sa.rpi-sensehat — SA Engine extension for the Raspberry Pi Sense HAT v1

A streaming SA Engine extension that exposes the Sense HAT's IMU and
barometer as named OSQL flows. A background C thread reads the LSM9DS1
FIFO (and optionally the LPS25H) over I²C and pushes tagged samples
into a [`sa_datapump`](https://www.streamanalyze.com/) circular buffer.
SA Engine consumers subscribe to one flow or merge several into a single
timestamped stream — well suited to time-based windows (`twinagg`),
group-by-topic aggregations, and edge ML pipelines.

## Sensors

| Chip | Signals | Default rate |
| --- | --- | --- |
| LSM9DS1 | accelerometer (±16 g), gyroscope (±2000 dps) | configurable, 14.9 Hz – 952 Hz |
| LPS25H | barometric pressure (hPa), temperature (°C) | configurable polling, 0 = disabled |

## Files

| File | Purpose |
| --- | --- |
| [sa.rpi-sensehat.c](sa.rpi-sensehat.c) | C extension: I²C drivers, reader thread, multi-flow datapump, foreign functions |
| [sa.rpi-sensehat.osql](sa.rpi-sensehat.osql) | OSQL bindings: stream functions, payload accessors |
| [test-sa.rpi-sensehat.osql](test-sa.rpi-sensehat.osql) | Validation tests: error handling, per-flow checks, multi-ODR rate validation |
| [Makefile](Makefile) | Build, install, and test targets |

## Prerequisites

- Raspberry Pi with a [Sense HAT v1](https://www.raspberrypi.com/products/sense-hat/) seated on the GPIO header.
- Raspberry Pi OS or other Debian-based Linux with `/dev/i2c-1` available.
  Enable I²C via `sudo raspi-config` → *Interface Options* → *I2C*.
- A C toolchain (`gcc`, `make`).
- [SA Engine](https://www.streamanalyze.com/) installed, with
  `SA_ENGINE_HOME` exported and pointing at the SA Engine repo root
  (the directory that contains `C/sa_core.h` and `bin/sa.engine`).

## Build & install

```sh
export SA_ENGINE_HOME=/path/to/sa.engine
make rpi-sensehat.so      # compile the shared library
make install              # copy rpi-sensehat.so to $SA_ENGINE_HOME/bin/
```

`make install` is what `load_extension("rpi-sensehat")` in OSQL relies on —
SA Engine searches `$SA_ENGINE_HOME/bin/` for extension `.so` files. The
`.so` drops the `sa.` prefix that the source files carry because
`load_extension` treats any dot in the name as a filename suffix and
skips appending `.so` — dot-free extension names load cleanly.

## Run the tests

The tests require a real Sense HAT; they read live sensor data and
validate plausible physics (accelerometer magnitude near 1 g when
stationary, pressure in 900–1100 hPa, temperature in 0–80 °C) and
sample-rate accuracy at multiple ODR settings.

```sh
make test                 # runs test-sa.rpi-sensehat.osql via sa.engine -O
```

## Quick start

```sql
load_osql('sa.rpi-sensehat.osql');

-- Register the pump: ODR 5 (476 Hz), 1024-item buffer,
-- 1 s pressure/temp polling, gyro disabled.
register_sensehat_pump(5, 1024, 1000, 0);

-- One flow at a time:
select sensehat:xyz(v)
  from Timeval v
 where v in extract(first_n(sensehat:flow("accel"), 10));

-- Multiple flows merged into one timestamped stream:
select sensehat:topic(v), value(v)
  from Timeval v
 where v in extract(first_n(
         sensehat:signal_flows(["accel","pressure","temp"]), 30));
```

## API reference

### Foreign functions (defined in `sa.rpi-sensehat.c`)

#### `register_sensehat_pump(odr, bufsize, env_ms, enable_gyro) -> Charstring`

Registers the `sensehat` datapump with the given settings. The reader
thread starts lazily on the first `subscribe`. Returns the pump name
(`"sensehat"`).

| Argument | Meaning |
| --- | --- |
| `odr` | LSM9DS1 output data rate, **1–6** (see table below). |
| `bufsize` | Circular-buffer capacity in items. 1024–4096 is typical. |
| `env_ms` | Pressure/temperature polling interval in ms. **0 disables** the LPS25H entirely. |
| `enable_gyro` | `0` = accel only (saves I²C bandwidth); `1` = accel + gyro. |

Raises `"sensehat: ODR must be 1-6"` if `odr` is out of range.

#### `sensehat:stop() -> Integer`

Tears down the running pump and reader thread so
`register_sensehat_pump` can be called again with new settings. Returns
`1` if a pump was stopped, `0` if nothing was running.

### OSQL helpers (defined in `sa.rpi-sensehat.osql`)

| Function | Purpose |
| --- | --- |
| `sensehat:flow(name)` | Subscribe to a single flow (`"accel"`, `"gyro"`, `"pressure"`, `"temp"`). |
| `sensehat:signal_flows(names)` | Subscribe to a vector of flows merged into one stream. |
| `sensehat:topic(tv)` | Flow name on a sample. |
| `sensehat:xyz(tv)` | 3-vector payload (accel/gyro). |
| `sensehat:scalar(tv)` | Scalar payload (pressure/temp). |
| `sensehat:accel_mag(tv)` | Accelerometer magnitude in g. |

### Flow topics

| Topic | Payload | Rate |
| --- | --- | --- |
| `sensehat:accel` | `Timeval([ax, ay, az])` in g | IMU ODR (always on) |
| `sensehat:gyro` | `Timeval([gx, gy, gz])` in dps | IMU ODR (if `enable_gyro = 1`) |
| `sensehat:pressure` | `Timeval(hPa)` | every `env_ms` (if > 0) |
| `sensehat:temp` | `Timeval(°C)` | every `env_ms` (if > 0) |

Each item is wrapped as `Timeval([flow_name, payload])` so that merged
streams from `sensehat:signal_flows` remain self-describing — the
typed accessors above unpack them.

### LSM9DS1 ODR table

| `odr` | Rate |
| --- | --- |
| 1 | 14.9 Hz |
| 2 | 59.5 Hz |
| 3 | 119 Hz |
| 4 | 238 Hz |
| 5 | 476 Hz |
| 6 | 952 Hz |

## Notes

- **Cold-start ramp-up.** After `register_sensehat_pump` and the first
  `subscribe`, the pump takes ~200–400 ms to reach the configured ODR
  (sensor reset delays, I²C config writes, FIFO settling). The tests
  measure several windows and average only the steady-state tail —
  see `tail_avg` in [test-sa.rpi-sensehat.osql](test-sa.rpi-sensehat.osql).
- **I²C bandwidth.** At 952 Hz with both accel and gyro enabled, the
  reader thread spends most of its time draining the FIFO over I²C.
  Skipping gyro (`enable_gyro = 0`) roughly halves bus traffic.
- **Sense HAT v1 only.** The Sense HAT v2 uses different sensor chips
  (ICM-20948 + LPS22HB) and is not supported by this extension.

## License

MIT — see [LICENSE](LICENSE).
