# sa.rpi-sensehat — SA Engine extension for the Raspberry Pi Sense HAT (v1 and v2)

A streaming SA Engine extension that exposes all Sense HAT sensors and
the joystick as named OSQL flows, plus functions for the 8×8 LED matrix.
A single background C thread reads the LSM9DS1 FIFO, polls the other
sensors over I²C and reads joystick events, and pushes tagged samples
into a [`sa_datapump`](https://www.streamanalyze.com/) circular buffer.
SA Engine consumers subscribe to one flow or merge several into a single
timestamped stream — well suited to time-based windows (`twinagg`),
group-by-topic aggregations, and edge ML pipelines.

## Sensors

| Chip | Signals | Default rate |
| --- | --- | --- |
| LSM9DS1 | accelerometer (±8 g), gyroscope (±2000 dps) | configurable, 14.9 Hz – 952 Hz |
| LPS25H | barometric pressure (hPa), temperature (°C) | configurable polling, 0 = disabled |
| HTS221 | relative humidity (%rH), temperature (°C) | polled with the pressure sensor (`env_ms`) |
| LSM9DS1 magnetometer | magnetic field (µT, ±4 gauss) | `mag_ms` polling, 0 = disabled |
| TCS34725 (v2 only) | colour/light, raw R, G, B, clear counts (gain 16, 154 ms integration) | `colour_ms` polling, 0 = disabled |
| Joystick | up / down / left / right / middle; press / release / hold | on events |
| LED matrix | 8×8 RGB565 framebuffer `RPi-Sense FB` | written on demand |

Humidity, magnetometer, colour and joystick are optional: when a device
is missing (the v1 HAT has no colour sensor), only that flow receives an
error and every other flow keeps streaming.

## Files

| File | Purpose |
| --- | --- |
| [sa.rpi-sensehat.c](sa.rpi-sensehat.c) | C extension: I²C drivers, reader thread, multi-flow datapump, foreign functions |
| [sa.rpi-sensehat.osql](sa.rpi-sensehat.osql) | OSQL bindings: stream functions, payload accessors |
| [test-sa.rpi-sensehat.osql](test-sa.rpi-sensehat.osql) | Validation tests: error handling, per-flow checks, multi-ODR rate validation |
| [Makefile](Makefile) | Build, install, and test targets |

## Prerequisites

- Raspberry Pi with a [Sense HAT](https://www.raspberrypi.com/products/sense-hat/) (v1 or v2) seated on the GPIO header. Tested on a v2 (LSM9DS1 0x6a/0x1c, LPS25H 0x5c, HTS221 0x5f, TCS34725 0x29) with Debian 13 / kernel 6.18.
- The user needs access to `/dev/i2c-1` (group `i2c`), the joystick event device (group `input`) and the LED framebuffer `/dev/fb*` (group `video`).
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

Registering while a pump is running stops the running pump first, so
re-running a setup script is safe.

#### `register_sensehat_pump_ex(odr, bufsize, env_ms, enable_gyro, mag_ms, colour_ms, joystick) -> Charstring`

Like `register_sensehat_pump`, plus:

| Argument | Meaning |
| --- | --- |
| `mag_ms` | Magnetometer polling interval in ms (0 = disabled). The sensor runs at 80 Hz. |
| `colour_ms` | Colour sensor polling interval in ms (0 = disabled). One integration takes 154 ms. |
| `joystick` | `1` = stream joystick events, `0` = off. |

```sql
register_sensehat_pump_ex(2, 4096, 1000, 1, 50, 200, 1);  -- mag 20 Hz, colour 5 Hz, joystick on
```

#### LED matrix functions

Independent of the pump. Colours are 0–255 and are stored as RGB565, so
reading back quantises red/blue to multiples of 8 and green to multiples
of 4. `x` is the column and `y` the row, both 0–7.

| Function | Effect |
| --- | --- |
| `sensehat:led_clear(r, g, b) -> Integer` | Fill all 64 pixels. |
| `sensehat:led_pixel(x, y, r, g, b) -> Integer` | Set one pixel. |
| `sensehat:led_pixels(Vector px) -> Integer` | Set all pixels from 64 `[r, g, b]` vectors, row-major (`index = y*8 + x`). |
| `sensehat:led_get_pixels() -> Vector` | Read the 64 pixels back as `[r, g, b]`. |
| `sensehat:led_low_light(enable) -> Integer` | `1` = dim, `0` = normal brightness (driver gamma table). |

#### `sensehat:stop() -> Integer`

Tears down the running pump and reader thread so
`register_sensehat_pump` can be called again with new settings. Returns
`1` if a pump was stopped, `0` if nothing was running.

### OSQL helpers (defined in `sa.rpi-sensehat.osql`)

| Function | Purpose |
| --- | --- |
| `sensehat:flow(name)` | Subscribe to a single flow (`"accel"`, `"gyro"`, `"pressure"`, `"temp"`, `"humidity"`, `"humidity_temp"`, `"mag"`, `"colour"`, `"joystick"`). |
| `sensehat:signal_flows(names)` | Subscribe to a vector of flows merged into one stream. |
| `sensehat:topic(tv)` | Flow name on a sample. |
| `sensehat:xyz(tv)` | 3-vector payload (accel/gyro/mag). |
| `sensehat:scalar(tv)` | Scalar payload (pressure/temp/humidity/humidity_temp). |
| `sensehat:rgbc(tv)` | Colour payload `[r, g, b, clear]` (raw counts). |
| `sensehat:key(tv)` | Joystick payload `[direction, action]`. |
| `sensehat:accel_mag(tv)` | Accelerometer magnitude in g. |

### Flow topics

| Topic | Payload | Rate |
| --- | --- | --- |
| `sensehat:accel` | `Timeval([ax, ay, az])` in g | IMU ODR (always on) |
| `sensehat:gyro` | `Timeval([gx, gy, gz])` in dps | IMU ODR (if `enable_gyro = 1`) |
| `sensehat:pressure` | `Timeval(hPa)` | every `env_ms` (if > 0) |
| `sensehat:temp` | `Timeval(°C)` | every `env_ms` (if > 0) |
| `sensehat:humidity` | `Timeval(%rH)` | every `env_ms` (if > 0) |
| `sensehat:humidity_temp` | `Timeval(°C)` from the HTS221 | every `env_ms` (if > 0) |
| `sensehat:mag` | `Timeval([mx, my, mz])` in µT, raw LSM9DS1-M frame, uncalibrated | every `mag_ms` (if > 0) |
| `sensehat:colour` | `Timeval([r, g, b, c])` raw counts | every `colour_ms` (if > 0) |
| `sensehat:joystick` | `Timeval([direction, action])`, e.g. `["up", "press"]` | on events (if `joystick = 1`) |

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
- **Sense HAT v1 and v2.** Both use the LSM9DS1, LPS25H and HTS221;
  the v2 adds the TCS34725 colour sensor.
- **One driver at a time.** The pump reconfigures the sensors; don't run
  the Python `sense_hat` / RTIMULib stack against the same HAT at the
  same time.
- **Magnetometer frame and offset.** `sensehat:mag` is the raw LSM9DS1
  magnetometer frame (RTIMULib flips x and z to align it with the
  accelerometer). Readings include the board's hard-iron offset, which
  can be several hundred µT; calibrate (per-axis min/max while rotating
  the board) before computing a heading.
- **Joystick and the console.** The joystick is also a keyboard
  (arrow keys + Enter) for the Linux console; reading it here does not
  grab it exclusively.

## License

MIT — see [LICENSE](LICENSE).
