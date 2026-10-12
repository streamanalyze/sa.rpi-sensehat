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
| HTS221 | relative humidity (%rH), temperature (°C) | polled every `humidity_ms` (`env_ms` in the 4-argument register) |
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
| [test-sa.rpi-sensehat.osql](test-sa.rpi-sensehat.osql) | Validation tests (v1 and v2): error handling, per-flow checks, multi-ODR rate validation, LED matrix |
| [test-sa.rpi-sensehat-v2.osql](test-sa.rpi-sensehat-v2.osql) | Validation tests for v2-only hardware (colour sensor) |
| [sensehat-streams.osql](sensehat-streams.osql) | Optional helpers: typed sensor streams, motion summary, recorder |
| [sensehat-compass.osql](sensehat-compass.osql) | Optional helpers: magnetometer calibration, tilt-compensated heading |
| [sensehat-display.osql](sensehat-display.osql) | Optional helpers: scrolling text, characters, numbers, emojis on the LED matrix |
| [sensehat-font.osql](sensehat-font.osql), [sensehat-emoji.osql](sensehat-emoji.osql) | Data for the display helpers: 91-character font (from python-sense-hat, BSD-3-Clause), 31 colour emojis |
| [examples/sensehat-demos.osql](examples/sensehat-demos.osql) | Demos: tilt level, joystick dot, font and emoji slideshows |
| [test-sensehat-helpers.osql](test-sensehat-helpers.osql) | Tests for the helper files (no Sense HAT needed) |
| [Makefile](Makefile) | Build, install, and test targets |

## Prerequisites

- Raspberry Pi with a [Sense HAT](https://www.raspberrypi.com/products/sense-hat/) (v1 or v2) seated on the GPIO header. Tested on a v2 (LSM9DS1 0x6a/0x1c, LPS25H 0x5c, HTS221 0x5f, TCS34725 0x29) with Debian 13 / kernel 6.18.
- The user needs access to `/dev/i2c-1` (group `i2c`), the joystick event device (group `input`) and the LED framebuffer `/dev/fb*` (group `video`).
- Raspberry Pi OS or other Debian-based Linux with `/dev/i2c-1` available.
- **A fixed VPU core clock** for the higher ODRs: add `core_freq_min=500` to `/boot/firmware/config.txt` and reboot. The I²C clock is divided from the core clock. With the default `ondemand` governor the core drops to 300 MHz a few seconds after start, the bus slows to 60 %, and ODR 6 delivers ~500 instead of ~970 samples/s (measured on a Pi 4).
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
make test-v2              # Sense HAT v2 only: colour sensor tests
make test-helpers         # OSQL helper files; no Sense HAT needed
```

The LED tests briefly change the display, and the joystick test only
checks that the flow can be subscribed (presses need a human).

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

-- All sensors: ODR 2 (59.5 Hz), 4096-item buffer, accel + gyro,
-- pressure and humidity every 1 s, mag every 50 ms, colour every 200 ms,
-- joystick on.
register_sensehat_pump(2, 4096, 1, 1, 1000, 1000, 50, 200, 1);

select sensehat:topic(v), value(v)
  from Timeval v
 where v in extract(first_n(
         sensehat:signal_flows(["mag","humidity","colour"]), 20));

-- Light the top-left pixel red, then clear the matrix:
sensehat:led_pixel(0, 0, 255, 0, 0);
sensehat:led_clear(0, 0, 0);
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
| `env_ms` | Pressure/temperature/humidity polling interval in ms. **0 disables** the LPS25H and HTS221 entirely. |
| `enable_gyro` | `0` = accel only (saves I²C bandwidth); `1` = accel + gyro. |

Raises `"sensehat: ODR must be 1-6"` if `odr` is out of range.

Registering while a pump is running stops the running pump first, so
re-running a setup script is safe.

#### `register_sensehat_pump(odr, bufsize, accel, gyro, pressure_ms, humidity_ms, mag_ms, colour_ms, joystick) -> Charstring`

The same function with every sensor named.

| Argument | Meaning |
| --- | --- |
| `odr`, `bufsize` | As above. |
| `accel` | `1` = stream the accelerometer, `0` = off. |
| `gyro` | `1` = stream the gyroscope, `0` = off. Needs `accel = 1` (the LSM9DS1 FIFO pairs them). With accel and gyro both off the IMU is not read at all. |
| `pressure_ms` | LPS25H pressure + temperature interval in ms (0 = off). |
| `humidity_ms` | HTS221 humidity + temperature interval in ms (0 = off). |
| `mag_ms` | Magnetometer interval in ms (0 = off). The sensor runs at 80 Hz. |
| `colour_ms` | Colour sensor interval in ms (0 = off). One integration takes 154 ms. |
| `joystick` | `1` = stream joystick events, `0` = off. |

```sql
register_sensehat_pump(2, 4096, 1, 1, 1000, 1000, 50, 200, 1);  -- mag 20 Hz, colour 5 Hz, joystick on
register_sensehat_pump(6, 2048, 1, 0, 0, 0, 0, 0, 0);           -- accel only at ~970 Hz
```

#### The I²C bus budget

Each sample is its own I²C transaction, a register write plus a read of 4–8 bytes. At the Pi's default 100 kHz bus clock that is about 0.9 ms, so the bus serves roughly 1100 transactions per second. That figure was measured as 2 × 547 Hz with accel and gyro at ODR 6, with the core clock fixed. Both register functions add up what a request needs before starting the pump:
- one transaction per accel sample and one per gyro sample (the ODR rate each);
- one per pressure, humidity, mag and colour poll (1000 / interval each).

If that exceeds the budget, 1050 per second at 100 kHz and scaled to the configured `i2c_arm_baudrate`, the call raises `"sensehat: request exceeds the I2C bus budget (lower odr, turn gyro off or poll slower)"`, with the needed transactions per second as the error value. It no longer silently delivers fewer samples. Examples at 100 kHz:

| Request | Needs | |
| --- | --- | --- |
| accel at ODR 6 | 952/s | fits (~970 Hz measured) |
| accel at ODR 6 + pressure, humidity 1 s + mag 50 ms + colour 200 ms | 979/s | fits |
| accel + gyro at ODR 5 | 952/s | fits |
| accel + gyro at ODR 6 | 1904/s | **error** |

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
| `sensehat:flows_after(names, since)` | Like `signal_flows`, but drops items stamped before `since` (e.g. left over from a previous pump). |
| `sensehat:flow_topics(names)` | The topic names (`"sensehat:" + name`) for a vector of flow names. |
| `sensehat:topic(tv)` | Flow name on a sample. |
| `sensehat:payload(tv)` | Raw payload of a sample (untyped). |
| `sensehat:xyz(tv)` | 3-vector payload (accel/gyro/mag). |
| `sensehat:scalar(tv)` | Scalar payload (pressure/temp/humidity/humidity_temp). |
| `sensehat:rgbc(tv)` | Colour payload `[r, g, b, clear]` (raw counts). |
| `sensehat:key(tv)` | Joystick payload `[direction, action]`. |
| `sensehat:accel_mag(tv)` | Accelerometer magnitude in g. |

### Optional OSQL helper files

Higher-level functions in separate files. Load `sa.rpi-sensehat.osql`
first, then the files you need, from the plugin folder:

```sql
load_osql('sa.rpi-sensehat.osql');
load_osql('sensehat-streams.osql');
load_osql('sensehat-compass.osql');   -- needs sensehat-streams.osql
load_osql('sensehat-display.osql');   -- loads the font and emoji files
register_sensehat_pump(2, 4096, 1, 1, 1000, 1000, 50, 200, 1);

sensehat:show_message("Hello");
sensehat:show_emoji("smile");
first_n(sensehat:env(), 3);
```

**`sensehat-streams.osql`**

| Function | Purpose |
| --- | --- |
| `sensehat:accel()`, `sensehat:gyro()`, `sensehat:mag_raw()` | Streams of `[x, y, z]` (g, dps, µT). A stream of numeric vectors plots as one line per element. |
| `sensehat:colour()` | Stream of `[r, g, b, clear]` raw counts. |
| `sensehat:joystick()` | Stream of `[direction, action]`. |
| `sensehat:env()` | Stream of `["pressure", hPa]`, `["temp", °C]`, `["humidity", %rH]`. |
| `sensehat:motion(win)` | Per tumbling window of `win` s: `[local time, samples, avg \|a\|, max \|a\|, stdev \|a\|]` (stdev near 0 = still). |
| `sensehat:record(flows, secs, file)` | Record flows for `secs` s to a JSON file, one `{"sa_time", "sa_value": [flow, payload]}` per line; `json:file_stream(file)` reads it back. Items cached from before the start are dropped. |

**`sensehat-compass.osql`**

| Function | Purpose |
| --- | --- |
| `sensehat:mag_calibrate(secs)` | Hard-iron calibration: rotate the board through all orientations for `secs` s; stores the offset (per-axis midpoint of min and max) and returns `[offset, range]`. |
| `sensehat:mag()` | Magnetometer `[x, y, z]` in µT with the offset removed. |
| `sensehat:mag_offset()`, `sensehat:set_mag_offset(v)` | Read or restore the stored offset (e.g. saved with `json:stringify`). |
| `sensehat:heading()` | Tilt-compensated compass heading in degrees, ~10 Hz: 0 = the accelerometer +x axis (the Pi's USB/Ethernet end) points to magnetic north, rising clockwise. Add the local declination for true north. |
| `sensehat:heading_of(a, m)` | Heading from one accel and one calibrated mag vector. |

**`sensehat-display.osql`**

The `*_frame` functions build a 64-pixel frame; the `show_*` functions
write it to the matrix.

| Function | Purpose |
| --- | --- |
| `sensehat:show_message(text)` | Scroll text once in white (python-sense-hat font and padding); blocks until done. |
| `sensehat:scroll_text(text, rgb, secs)` | Scroll text in a colour, `secs` per column; streams the frame number. |
| `sensehat:show_char(ch, rgb)` / `char_frame` | One centred character. |
| `sensehat:show_number(n, rgb)` / `number_frame` | 0-99: one font digit, or two compact 3x5 digits. |
| `sensehat:show_emoji(name)` / `emoji_frame` | One of 31 colour emojis; `sensehat:emoji_names()` lists them. |
| `sensehat:logo_message(emoji, text, rgb, hold, secs)` | Show an emoji for `hold` s, then scroll it off followed by text. |
| `sensehat:show_dot(x, y, rgb)` / `dot_frame`, `sensehat:led_off()` | One dot; all off. |
| `sensehat:text_columns(text)`, `sensehat:text_frame(cols, k, fg, bg)` | Building blocks for custom scrolling. |

Emojis are 8 rows of palette letters in `sensehat-emoji.osql`, easy to
edit by eye. The demos in `examples/sensehat-demos.osql` show how the
pieces combine (`sensehat:level_demo()`, `sensehat:joy_demo()`).

### Flow topics

| Topic | Payload | Rate |
| --- | --- | --- |
| `sensehat:accel` | `Timeval([ax, ay, az])` in g | IMU ODR (if `accel = 1`; always with the 4-argument register) |
| `sensehat:gyro` | `Timeval([gx, gy, gz])` in dps | IMU ODR (if `gyro` / `enable_gyro = 1`) |
| `sensehat:pressure` | `Timeval(hPa)` | every `pressure_ms` / `env_ms` (if > 0) |
| `sensehat:temp` | `Timeval(°C)` | every `pressure_ms` / `env_ms` (if > 0) |
| `sensehat:humidity` | `Timeval(%rH)` | every `humidity_ms` / `env_ms` (if > 0) |
| `sensehat:humidity_temp` | `Timeval(°C)` from the HTS221 | every `humidity_ms` / `env_ms` (if > 0) |
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
- **I²C bandwidth.** See "The I²C bus budget" above. Accel
  at ODR 6 alone uses ~90 % of a 100 kHz bus. It holds ~970 Hz only with
  the core clock fixed (`core_freq_min=500`, see Prerequisites).
- **Sense HAT v1 and v2.** Both use the LSM9DS1, LPS25H and HTS221;
  the v2 adds the TCS34725 colour sensor.
- **One driver at a time.** The pump reconfigures the sensors; don't run
  the Python `sense_hat` / RTIMULib stack against the same HAT at the
  same time.
- **Magnetometer frame and offset.** The `mag` flow is the raw LSM9DS1
  magnetometer frame. Relative to the raw accelerometer frame its x axis
  is flipped (measured: over a tumble recording, `[-x, y, z]` is the only
  axis mapping that keeps the angle between field and gravity constant);
  `sensehat:heading_of` applies this. Readings include the board's
  hard-iron offset, which changed between ~40 and ~330 µT on our board
  depending on what was nearby. Calibrate with `sensehat:mag_calibrate`
  where the board will be used. At high latitudes the horizontal field is
  only ~13-20 µT, so expect ±15-20° from the heading.
- **Joystick and the console.** The joystick is also a keyboard
  (arrow keys + Enter) for the Linux console; reading it here does not
  grab it exclusively.

## License

MIT — see [LICENSE](LICENSE).

The font data in [sensehat-font.osql](sensehat-font.osql) is derived from
[python-sense-hat](https://github.com/RPi-Distro/python-sense-hat)
(Copyright (c) 2015 Raspberry Pi Foundation, BSD-3-Clause); its notice
is in the file header.
