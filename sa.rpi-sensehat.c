/*
 * sa.rpi-sensehat.c — SA Engine datapump extension for Raspberry Pi Sense HAT
 *                     (v1 and v2)
 *
 * Uses the sa_datapump infrastructure with per-signal flow routing.
 * A single background thread reads the LSM9DS1 IMU FIFO and polls the
 * other sensors (LPS25H barometer, HTS221 humidity, LSM9DS1 magnetometer,
 * TCS34725 colour sensor, joystick), pushing tagged items into one
 * circular buffer (the datapump is single-producer).
 * SA Engine consumers select desired signals via subscribe:merge.
 *
 * Flow names (topics under the "sensehat" pump):
 *   sensehat:accel         — Timeval([ax,ay,az]) g       at IMU ODR (always on)
 *   sensehat:gyro          — Timeval([gx,gy,gz]) dps     at IMU ODR (if enabled)
 *   sensehat:pressure      — Timeval(hPa)                at env_ms  (if > 0)
 *   sensehat:temp          — Timeval(°C, LPS25H)         at env_ms  (if > 0)
 *   sensehat:humidity      — Timeval(%rH)                at env_ms  (if > 0)
 *   sensehat:humidity_temp — Timeval(°C, HTS221)         at env_ms  (if > 0)
 *   sensehat:mag           — Timeval([mx,my,mz]) µT      at mag_ms  (if > 0)
 *   sensehat:colour        — Timeval([r,g,b,c]) counts   at colour_ms (if > 0, v2 only)
 *   sensehat:joystick      — Timeval([dir, action])      on events  (if enabled)
 *       dir    = "up" | "down" | "left" | "right" | "middle"
 *       action = "press" | "release" | "hold"
 *
 * Humidity, magnetometer, colour and joystick are optional: if a device
 * is missing (e.g. no colour sensor on a v1 HAT) only that flow gets an
 * error; the rest of the pump keeps running.
 *
 * Exposed OSQL foreign functions (see sa.rpi-sensehat.osql for full bindings):
 *
 *   register_sensehat_pump(Integer odr, Integer bufsize,
 *                          Integer env_ms, Integer enable_gyro)
 *     -> Charstring
 *
 *     odr         — LSM9DS1 output data rate 1-6 (see table below)
 *     bufsize     — circular-buffer capacity in items
 *     env_ms      — pressure/temp/humidity polling interval in ms (0 = disabled)
 *     enable_gyro — 0 = accel only, 1 = accel + gyro
 *
 *   register_sensehat_pump_ex(Integer odr, Integer bufsize, Integer env_ms,
 *                             Integer enable_gyro, Integer mag_ms,
 *                             Integer colour_ms, Integer joystick)
 *     -> Charstring
 *
 *     As above, plus magnetometer / colour polling intervals in ms
 *     (0 = disabled) and joystick events (0/1).
 *
 *   Registering while a pump is running first stops the running pump.
 *
 *   sensehat:stop() -> Integer
 *
 *     Tears down the running pump so register_sensehat_pump can be
 *     called again with new settings.  Returns 1 if a pump was
 *     stopped, 0 if nothing was running.
 *
 *   LED matrix (8x8, RGB565 framebuffer "RPi-Sense FB"; independent of
 *   the pump):
 *     sensehat:led_clear(Integer r, Integer g, Integer b) -> Integer
 *     sensehat:led_pixel(Integer x, Integer y,
 *                        Integer r, Integer g, Integer b) -> Integer
 *     sensehat:led_pixels(Vector px) -> Integer   (64 x [r,g,b], row-major)
 *     sensehat:led_get_pixels() -> Vector         (64 x [r,g,b])
 *     sensehat:led_low_light(Integer on) -> Integer
 *
 * sa.rpi-sensehat.osql additionally defines pure-OSQL helpers on top of
 * these, notably sensehat:signal_flows(Vector of Charstring) which
 * subscribes to a merged stream of the named flows.
 *
 * ODR table (LSM9DS1):
 *   1 =  14.9 Hz    4 = 238 Hz
 *   2 =  59.5 Hz    5 = 476 Hz
 *   3 = 119   Hz    6 = 952 Hz
 *
 * Build: see Makefile
 * Load:  load_extension("rpi-sensehat");
 */

#include "sa_core.h"
#include "sa_datapump.h"
#include "sa_lisp.h"
#include "sa_syscalls.h"
#include "sa_threads.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* a_timestamp is EXPORT in sa.engine/system/C/a_time.c but not in sa_core.h */
extern ohandle a_timestamp(time_t seconds, int useconds, ohandle o);

/* ---- I2C addresses ---- */
#define LSM_AG_ADDR 0x6A   /* LSM9DS1 accel/gyro   */
#define LSM_M_ADDR  0x1C   /* LSM9DS1 magnetometer */
#define LPS_ADDR    0x5C   /* LPS25H pressure      */
#define HTS_ADDR    0x5F   /* HTS221 humidity      */
#define TCS_ADDR    0x29   /* TCS34725 colour (v2) */

/* ---- LSM9DS1 registers ---- */
#define LSM_WHO_AM_I     0x0F
#define LSM_CTRL_REG1_G  0x10
#define LSM_CTRL_REG4    0x1E
#define LSM_CTRL_REG5_XL 0x1F
#define LSM_CTRL_REG6_XL 0x20
#define LSM_CTRL_REG8    0x22
#define LSM_CTRL_REG9    0x23
#define LSM_FIFO_CTRL    0x2E
#define LSM_FIFO_SRC     0x2F
#define LSM_OUT_X_L_G    0x18
#define LSM_OUT_X_L_XL   0x28

/* ---- LPS25H registers ---- */
#define LPS_WHO_AM_I     0x0F
#define LPS_CTRL_REG1    0x20
#define LPS_CTRL_REG2    0x21
#define LPS_PRESS_OUT_XL 0x28

/* ---- LSM9DS1 magnetometer registers ---- */
#define MAG_WHO_AM_I     0x0F           /* = 0x3D */
#define MAG_CTRL_REG1    0x20
#define MAG_CTRL_REG2    0x21
#define MAG_CTRL_REG3    0x22
#define MAG_CTRL_REG4    0x23
#define MAG_OUT_X_L      0x28

/* ---- HTS221 registers ---- */
#define HTS_WHO_AM_I     0x0F           /* = 0xBC */
#define HTS_CTRL_REG1    0x20
#define HTS_HUMIDITY_OUT 0x28
#define HTS_CALIB_0      0x30

/* ---- TCS34725 registers (always OR'ed with a command byte) ---- */
#define TCS_CMD          0x80           /* command, repeated byte  */
#define TCS_CMD_AUTOINC  0xA0           /* command, auto-increment */
#define TCS_ENABLE       0x00
#define TCS_ATIME        0x01
#define TCS_CONTROL      0x0F
#define TCS_ID           0x12           /* = 0x44 (TCS34725) / 0x4D (TCS34727) */
#define TCS_CDATAL       0x14

/* ---- Joystick ---- */
/* Device path on a Pi 4 (BCM2711 I2C at fe804000); on other boards
   joy_open() falls back to scanning /dev/input/event* for JOY_NAME. */
#define JOY_BY_PATH \
    "/dev/input/by-path/platform-fe804000.i2c-platform-fe804000.i2c:sensehat@46:joystick-event"
#define JOY_NAME    "Raspberry Pi Sense HAT Joystick"
#define JOY_POLL_S  0.01                /* check for key events every 10 ms */

/* ---- Conversion factors ---- */
#define ACCEL_SCALE (0.000244)          /* ±8g: 0.244 mg/LSB per LSM9DS1 datasheet */
#define GYRO_SCALE  (2000.0 / 32768.0) /* ±2000 dps */
#define MAG_SCALE   (0.014)            /* ±4 gauss: 0.14 mgauss/LSB = 0.014 µT/LSB */

/* ---- Accelerometer calibration loaded from /etc/RTIMULib.ini ---- */
/* 6-position gravity calibration; corrected = 2*(raw-min)/(max-min) - 1  */
/* Falls back to identity (min=-1, max=1) when file is absent or invalid.  */

#define RTIMULIB_INI "/etc/RTIMULib.ini"

typedef struct {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
} accel_cal_t;

static accel_cal_t accel_cal = { -1.0, 1.0, -1.0, 1.0, -1.0, 1.0 };

static void load_accel_cal(void) {
    FILE *f = fopen(RTIMULIB_INI, "r");
    if (!f) return;

    accel_cal_t c = { -1.0, 1.0, -1.0, 1.0, -1.0, 1.0 };
    int valid = 0;
    char line[256], key[64], sval[64];

    while (fgets(line, sizeof(line), f)) {
        if (sscanf(line, "%63[^=]=%63s", key, sval) != 2) continue;
        if      (strcmp(key, "AccelCalValid") == 0) valid = strcmp(sval, "true") == 0;
        else if (strcmp(key, "AccelCalMinX")  == 0) sscanf(sval, "%lf", &c.min_x);
        else if (strcmp(key, "AccelCalMaxX")  == 0) sscanf(sval, "%lf", &c.max_x);
        else if (strcmp(key, "AccelCalMinY")  == 0) sscanf(sval, "%lf", &c.min_y);
        else if (strcmp(key, "AccelCalMaxY")  == 0) sscanf(sval, "%lf", &c.max_y);
        else if (strcmp(key, "AccelCalMinZ")  == 0) sscanf(sval, "%lf", &c.min_z);
        else if (strcmp(key, "AccelCalMaxZ")  == 0) sscanf(sval, "%lf", &c.max_z);
    }
    fclose(f);
    if (valid) accel_cal = c;
}

/* ---- SA Engine error codes (registered in a_initialize_extension) ---- */
static int SENSEHAT_INVALID_ODR;

/* ================================================================
 * Flow types and tagged item struct
 *
 * A single struct covers all flow types via a tagged union.
 * The datapump memcpy's sizeof(sensehat_item_t) for every push,
 * so the struct is kept as small as practical.
 * ================================================================ */

typedef enum {
    FLOW_ACCEL = 0,
    FLOW_GYRO,
    FLOW_PRESSURE,
    FLOW_TEMP,
    FLOW_HUMIDITY,
    FLOW_HUMIDITY_TEMP,
    FLOW_MAG,
    FLOW_COLOUR,
    FLOW_JOYSTICK
} sensehat_flow_t;

static const char *flow_names[] = {"accel", "gyro", "pressure", "temp",
                                   "humidity", "humidity_temp", "mag",
                                   "colour", "joystick"};

typedef struct {
    struct timespec tspec;
    sensehat_flow_t flow;
    union {
        struct { double x, y, z; } vec3;   /* accel, gyro or mag          */
        double scalar;                      /* pressure, temp, humidity    */
        uint16_t rgbc[4];                   /* colour: r, g, b, clear      */
        struct { int code, value; } key;    /* joystick: key code, 0/1/2   */
    } d;
} sensehat_item_t;

/* HTS221 factory calibration, read once at init */
typedef struct {
    double h0_rh, h1_rh;      /* %rH at the two calibration points    */
    double h0_out, h1_out;    /* raw humidity output at those points  */
    double t0_c, t1_c;        /* °C at the two calibration points     */
    double t0_out, t1_out;    /* raw temperature output at those      */
} hts_cal_t;

/* ================================================================
 * Pump context — carried in pump->context
 * ================================================================ */

typedef struct {
    volatile int running;
    volatile int done;
    int  odr_bits;
    int  gyro_enabled;
    int  env_ms;           /* 0 = pressure/temp/humidity disabled */
    int  mag_ms;           /* 0 = magnetometer disabled           */
    int  colour_ms;        /* 0 = colour sensor disabled          */
    int  joystick;         /* 0 = joystick disabled               */
    int  i2c_fd;
    int  joy_fd;
    int  hts_ok, mag_ok, tcs_ok;
    hts_cal_t hts_cal;
} sensehat_ctx_t;

/* ================================================================
 * Registration arguments — heap-allocated, read once by starter
 * ================================================================ */

typedef struct {
    int odr_bits;
    int buffer_size;
    int env_ms;
    int gyro_enabled;
    int mag_ms;
    int colour_ms;
    int joystick;
} sensehat_reg_args_t;

/* ================================================================
 * I2C helpers — parameterised on file descriptor
 * ================================================================ */

static int i2c_set_addr(int fd, uint8_t addr) {
    return ioctl(fd, I2C_SLAVE, addr) < 0 ? -1 : 0;
}

static int i2c_write_byte(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return write(fd, buf, 2) == 2 ? 0 : -1;
}

static int i2c_read_byte(int fd, uint8_t reg) {
    if (write(fd, &reg, 1) != 1) return -1;
    uint8_t val;
    if (read(fd, &val, 1) != 1) return -1;
    return val;
}

static int i2c_read_bytes(int fd, uint8_t reg, uint8_t *buf, int len) {
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, len) != len) return -1;
    return 0;
}

static int16_t to_i16(uint8_t lo, uint8_t hi) {
    return (int16_t)((hi << 8) | lo);
}

/* ================================================================
 * LSM9DS1 — init / FIFO count / read
 * ================================================================ */

static int lsm_init(int fd, int odr_bits, int gyro_enabled) {
    i2c_set_addr(fd, LSM_AG_ADDR);

    if (i2c_read_byte(fd, LSM_WHO_AM_I) != 0x68)
        return -1;

    i2c_write_byte(fd, LSM_CTRL_REG8,    0x05);   /* SW reset + boot   */
    usleep(100000);
    i2c_write_byte(fd, LSM_CTRL_REG8,    0x44);   /* auto-inc, BDU     */

    if (gyro_enabled) {
        /* Gyro: selected ODR, ±2000 dps */
        i2c_write_byte(fd, LSM_CTRL_REG1_G,  (odr_bits << 5) | 0x18);
        i2c_write_byte(fd, LSM_CTRL_REG4,    0x38);   /* gyro X/Y/Z on  */
    } else {
        i2c_write_byte(fd, LSM_CTRL_REG1_G,  0x00);   /* gyro off       */
    }

    /* Accel: selected ODR, ±8 g */
    i2c_write_byte(fd, LSM_CTRL_REG6_XL, (odr_bits << 5) | 0x18);
    i2c_write_byte(fd, LSM_CTRL_REG5_XL, 0x38);   /* accel X/Y/Z on    */
    i2c_write_byte(fd, LSM_CTRL_REG9,    0x02);   /* FIFO enable       */
    i2c_write_byte(fd, LSM_FIFO_CTRL,    0xC0);   /* FIFO continuous   */
    return 0;
}

static int lsm_fifo_count(int fd) {
    int src = i2c_read_byte(fd, LSM_FIFO_SRC);
    return src < 0 ? -1 : (src & 0x3F);
}

static int lsm_read_accel(int fd, double *ax, double *ay, double *az) {
    uint8_t buf[6];
    if (i2c_read_bytes(fd, LSM_OUT_X_L_XL, buf, 6) < 0) return -1;
#define APPLY_CAL(v, mn, mx) (2.0 * ((v) - (mn)) / ((mx) - (mn)) - 1.0)
    *ax = APPLY_CAL(to_i16(buf[0], buf[1]) * ACCEL_SCALE, accel_cal.min_x, accel_cal.max_x);
    *ay = APPLY_CAL(to_i16(buf[2], buf[3]) * ACCEL_SCALE, accel_cal.min_y, accel_cal.max_y);
    *az = APPLY_CAL(to_i16(buf[4], buf[5]) * ACCEL_SCALE, accel_cal.min_z, accel_cal.max_z);
#undef APPLY_CAL
    return 0;
}

static int lsm_read_gyro(int fd, double *gx, double *gy, double *gz) {
    uint8_t buf[6];
    if (i2c_read_bytes(fd, LSM_OUT_X_L_G, buf, 6) < 0) return -1;
    *gx = to_i16(buf[0], buf[1]) * GYRO_SCALE;
    *gy = to_i16(buf[2], buf[3]) * GYRO_SCALE;
    *gz = to_i16(buf[4], buf[5]) * GYRO_SCALE;
    return 0;
}

/* ================================================================
 * LPS25H (pressure + temperature) — init / read
 * ================================================================ */

static int lps_init(int fd) {
    i2c_set_addr(fd, LPS_ADDR);

    if (i2c_read_byte(fd, LPS_WHO_AM_I) != 0xBD)
        return -1;

    i2c_write_byte(fd, LPS_CTRL_REG2, 0x04);      /* SW reset          */
    usleep(50000);
    i2c_write_byte(fd, LPS_CTRL_REG1, 0xC4);      /* active, 25 Hz, BDU */
    return 0;
}

static int lps_read(int fd, double *pressure_hpa, double *temp_c) {
    i2c_set_addr(fd, LPS_ADDR);
    uint8_t buf[5];
    if (i2c_read_bytes(fd, LPS_PRESS_OUT_XL | 0x80, buf, 5) < 0) return -1;

    int32_t press_raw = buf[0] | (buf[1] << 8) | (buf[2] << 16);
    if (press_raw & 0x800000) press_raw |= (int32_t)0xFF000000;
    *pressure_hpa = press_raw / 4096.0;

    int16_t temp_raw = (int16_t)(buf[3] | (buf[4] << 8));
    *temp_c = 42.5 + temp_raw / 480.0;
    return 0;
}

/* ================================================================
 * HTS221 (humidity + temperature) — init / read
 * ================================================================ */

static int hts_init(int fd, hts_cal_t *cal) {
    i2c_set_addr(fd, HTS_ADDR);

    if (i2c_read_byte(fd, HTS_WHO_AM_I) != 0xBC)
        return -1;

    i2c_write_byte(fd, HTS_CTRL_REG1, 0x87);      /* active, BDU, 12.5 Hz */

    /* Factory calibration block 0x30..0x3F (MSB of sub-address = auto-inc) */
    uint8_t c[16];
    if (i2c_read_bytes(fd, HTS_CALIB_0 | 0x80, c, 16) < 0) return -1;
    cal->h0_rh  = c[0] / 2.0;
    cal->h1_rh  = c[1] / 2.0;
    cal->t0_c   = (((c[5] & 0x03) << 8) | c[2]) / 8.0;
    cal->t1_c   = (((c[5] & 0x0C) << 6) | c[3]) / 8.0;
    cal->h0_out = to_i16(c[6],  c[7]);
    cal->h1_out = to_i16(c[10], c[11]);
    cal->t0_out = to_i16(c[12], c[13]);
    cal->t1_out = to_i16(c[14], c[15]);
    if (cal->h1_out == cal->h0_out || cal->t1_out == cal->t0_out)
        return -1;                                 /* corrupt calibration */
    return 0;
}

static int hts_read(int fd, const hts_cal_t *cal, double *rh, double *temp_c) {
    i2c_set_addr(fd, HTS_ADDR);
    uint8_t buf[4];
    if (i2c_read_bytes(fd, HTS_HUMIDITY_OUT | 0x80, buf, 4) < 0) return -1;

    double h_out = to_i16(buf[0], buf[1]);
    double t_out = to_i16(buf[2], buf[3]);
    double h = cal->h0_rh + (h_out - cal->h0_out) * (cal->h1_rh - cal->h0_rh)
                            / (cal->h1_out - cal->h0_out);
    if (h < 0.0)   h = 0.0;
    if (h > 100.0) h = 100.0;
    *rh = h;
    *temp_c = cal->t0_c + (t_out - cal->t0_out) * (cal->t1_c - cal->t0_c)
                          / (cal->t1_out - cal->t0_out);
    return 0;
}

/* ================================================================
 * LSM9DS1 magnetometer — init / read
 * ================================================================ */

static int mag_init(int fd) {
    i2c_set_addr(fd, LSM_M_ADDR);

    if (i2c_read_byte(fd, MAG_WHO_AM_I) != 0x3D)
        return -1;

    i2c_write_byte(fd, MAG_CTRL_REG1, 0x7C);      /* X/Y ultra-high perf, 80 Hz */
    i2c_write_byte(fd, MAG_CTRL_REG2, 0x00);      /* ±4 gauss                   */
    i2c_write_byte(fd, MAG_CTRL_REG3, 0x00);      /* continuous conversion      */
    i2c_write_byte(fd, MAG_CTRL_REG4, 0x0C);      /* Z ultra-high perf          */
    return 0;
}

static int mag_read(int fd, double *mx, double *my, double *mz) {
    i2c_set_addr(fd, LSM_M_ADDR);
    uint8_t buf[6];
    if (i2c_read_bytes(fd, MAG_OUT_X_L | 0x80, buf, 6) < 0) return -1;
    *mx = to_i16(buf[0], buf[1]) * MAG_SCALE;
    *my = to_i16(buf[2], buf[3]) * MAG_SCALE;
    *mz = to_i16(buf[4], buf[5]) * MAG_SCALE;
    return 0;
}

/* ================================================================
 * TCS34725 colour sensor (Sense HAT v2 only) — init / read
 * ================================================================ */

static int tcs_init(int fd) {
    i2c_set_addr(fd, TCS_ADDR);

    int id = i2c_read_byte(fd, TCS_CMD | TCS_ID);
    if (id != 0x44 && id != 0x4D)
        return -1;

    i2c_write_byte(fd, TCS_CMD | TCS_ENABLE,  0x01);   /* power on              */
    usleep(3000);                                      /* >= 2.4 ms warm-up     */
    i2c_write_byte(fd, TCS_CMD | TCS_ENABLE,  0x03);   /* power on + RGBC       */
    i2c_write_byte(fd, TCS_CMD | TCS_ATIME,   0xC0);   /* 64 cycles = 154 ms    */
    i2c_write_byte(fd, TCS_CMD | TCS_CONTROL, 0x02);   /* gain 16x (indoor)     */
    return 0;
}

static int tcs_read(int fd, uint16_t rgbc[4]) {
    i2c_set_addr(fd, TCS_ADDR);
    uint8_t buf[8];                                    /* C, R, G, B (LE u16)  */
    if (i2c_read_bytes(fd, TCS_CMD_AUTOINC | TCS_CDATAL, buf, 8) < 0) return -1;
    rgbc[0] = buf[2] | (buf[3] << 8);                  /* r */
    rgbc[1] = buf[4] | (buf[5] << 8);                  /* g */
    rgbc[2] = buf[6] | (buf[7] << 8);                  /* b */
    rgbc[3] = buf[0] | (buf[1] << 8);                  /* clear */
    return 0;
}

/* ================================================================
 * Joystick — Linux input event device of the sensehat_joystick driver
 * ================================================================ */

static int joy_open(void) {
    int fd = open(JOY_BY_PATH, O_RDONLY | O_NONBLOCK);
    if (fd >= 0) return fd;

    /* Fall back to scanning event devices for the joystick's name. */
    for (int i = 0; i < 32; i++) {
        char path[32], name[128] = {0};
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 &&
            strcmp(name, JOY_NAME) == 0)
            return fd;
        close(fd);
    }
    return -1;
}

static const char *joy_direction(int code) {
    switch (code) {
    case KEY_UP:    return "up";
    case KEY_DOWN:  return "down";
    case KEY_LEFT:  return "left";
    case KEY_RIGHT: return "right";
    case KEY_ENTER: return "middle";
    default:        return NULL;
    }
}

static const char *joy_action(int value) {
    switch (value) {
    case 0:  return "release";
    case 1:  return "press";
    default: return "hold";
    }
}

/* Drain pending key events (non-blocking) into the pump. */
static void joy_poll(sa_datapump *pump, int fd) {
    struct input_event ev;
    while (read(fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
        if (ev.type != EV_KEY || !joy_direction(ev.code)) continue;
        sensehat_item_t item;
        item.tspec.tv_sec  = ev.input_event_sec;
        item.tspec.tv_nsec = ev.input_event_usec * 1000L;
        item.flow          = FLOW_JOYSTICK;
        item.d.key.code    = ev.code;
        item.d.key.value   = ev.value;
        sa_datapump_push(pump, &item);
    }
}

/* ================================================================
 * Datapump callbacks
 * ================================================================ */

/*
 * get_flow_name_fn — routes each item to the right topic based on
 * the flow tag.  The datapump prepends "sensehat:" automatically.
 */
static size_t sensehat_get_flow_name(sa_datapump *pump, void *data,
                                     char *const flow_name,
                                     size_t flow_name_size) {
    (void)pump;
    sensehat_item_t *item = (sensehat_item_t *)data;
    const char *name = flow_names[item->flow];
    size_t len = strlen(name);
    if (len > flow_name_size) len = flow_name_size;
    memcpy(flow_name, name, len);
    return len;
}

/*
 * create_item_fn — called on SA Engine thread with kernel locked.
 *
 * Every item becomes Timeval wrapping [flow_name, payload]:
 *   accel/gyro/mag   → ["accel", [x, y, z]]  (likewise "gyro", "mag")
 *   pressure         → ["pressure", hPa]
 *   temperature      → ["temp", °C]  (likewise "humidity_temp")
 *   humidity         → ["humidity", %rH]
 *   colour           → ["colour", [r, g, b, c]]
 *   joystick         → ["joystick", [dir, action]]
 */
static void sensehat_make_item(ohandle *ret, sa_datapump *pump, void *data) {
    (void)pump;
    sensehat_item_t *item = (sensehat_item_t *)data;
    ohandle payload = nil;

    switch (item->flow) {
    case FLOW_ACCEL:
    case FLOW_GYRO:
    case FLOW_MAG: {
        ohandle v = new_array(3, nil);
        a_seta(v, 0, mkreal(item->d.vec3.x));
        a_seta(v, 1, mkreal(item->d.vec3.y));
        a_seta(v, 2, mkreal(item->d.vec3.z));
        payload = v;
        break;
    }
    case FLOW_PRESSURE:
    case FLOW_TEMP:
    case FLOW_HUMIDITY:
    case FLOW_HUMIDITY_TEMP:
        payload = mkreal(item->d.scalar);
        break;
    case FLOW_COLOUR: {
        ohandle v = new_array(4, nil);
        for (int i = 0; i < 4; i++)
            a_seta(v, i, mkinteger(item->d.rgbc[i]));
        payload = v;
        break;
    }
    case FLOW_JOYSTICK: {
        ohandle v = new_array(2, nil);
        a_seta(v, 0, mkstring(joy_direction(item->d.key.code)));
        a_seta(v, 1, mkstring(joy_action(item->d.key.value)));
        payload = v;
        break;
    }
    }

    ohandle wrapper = new_array(2, nil);
    a_seta(wrapper, 0, mkstring(flow_names[item->flow]));
    a_seta(wrapper, 1, payload);

    a_setf(*ret, a_timestamp(item->tspec.tv_sec,
                             (int)(item->tspec.tv_nsec / 1000), wrapper));
}

/* ================================================================
 * Background I2C reader thread
 *
 * Owns the I2C file descriptor (and the joystick event device) for
 * its lifetime.  Reads the LSM9DS1 FIFO (accel, optionally gyro) and
 * polls the slower sensors on their own intervals.  Every reading is
 * pushed from this one thread as a tagged item into the datapump
 * circular buffer (the buffer is single-producer).
 * ================================================================ */

static double now_s(struct timespec *ts) {
    clock_gettime(CLOCK_REALTIME, ts);
    return ts->tv_sec + ts->tv_nsec * 1e-9;
}

/* Last poll times of the slower sources, owned by the reader thread. */
typedef struct {
    double env, mag, colour, joy;
} sensehat_polls_t;

/* Poll pressure/temp/humidity, magnetometer, colour and joystick when
   their interval is due.  Leaves the I2C address on the accel/gyro. */
static void poll_aux(sa_datapump *pump, sensehat_ctx_t *ctx,
                     sensehat_polls_t *last) {
    int fd = ctx->i2c_fd;
    struct timespec ts;
    double now = now_s(&ts);
    sensehat_item_t item;
    item.tspec = ts;

    if (ctx->env_ms > 0 && now - last->env >= ctx->env_ms / 1000.0) {
        last->env = now;
        double pressure, temp;
        if (lps_read(fd, &pressure, &temp) == 0) {
            item.flow = FLOW_PRESSURE; item.d.scalar = pressure;
            sa_datapump_push(pump, &item);
            item.flow = FLOW_TEMP;     item.d.scalar = temp;
            sa_datapump_push(pump, &item);
        }
        double rh, htemp;
        if (ctx->hts_ok && hts_read(fd, &ctx->hts_cal, &rh, &htemp) == 0) {
            item.flow = FLOW_HUMIDITY;      item.d.scalar = rh;
            sa_datapump_push(pump, &item);
            item.flow = FLOW_HUMIDITY_TEMP; item.d.scalar = htemp;
            sa_datapump_push(pump, &item);
        }
        i2c_set_addr(fd, LSM_AG_ADDR);
    }

    if (ctx->mag_ok && now - last->mag >= ctx->mag_ms / 1000.0) {
        last->mag = now;
        if (mag_read(fd, &item.d.vec3.x, &item.d.vec3.y, &item.d.vec3.z) == 0) {
            item.flow = FLOW_MAG;
            sa_datapump_push(pump, &item);
        }
        i2c_set_addr(fd, LSM_AG_ADDR);
    }

    if (ctx->tcs_ok && now - last->colour >= ctx->colour_ms / 1000.0) {
        last->colour = now;
        if (tcs_read(fd, item.d.rgbc) == 0) {
            item.flow = FLOW_COLOUR;
            sa_datapump_push(pump, &item);
        }
        i2c_set_addr(fd, LSM_AG_ADDR);
    }

    if (ctx->joy_fd >= 0 && now - last->joy >= JOY_POLL_S) {
        last->joy = now;
        joy_poll(pump, ctx->joy_fd);
    }
}

/* Probe the optional devices.  A missing one only disables (and sends
   an error to) its own flow, so e.g. a v1 HAT without the colour
   sensor still streams everything else. */
static void init_optional(sa_datapump *pump, sensehat_ctx_t *ctx) {
    int fd = ctx->i2c_fd;

    if (ctx->env_ms > 0) {
        ctx->hts_ok = hts_init(fd, &ctx->hts_cal) == 0;
        if (!ctx->hts_ok)
            sa_datapump_send_error(pump, "humidity*",
                                   "sensehat: HTS221 not found on I2C bus");
    }
    if (ctx->mag_ms > 0) {
        ctx->mag_ok = mag_init(fd) == 0;
        if (!ctx->mag_ok)
            sa_datapump_send_error(pump, "mag",
                "sensehat: LSM9DS1 magnetometer not found on I2C bus");
    }
    if (ctx->colour_ms > 0) {
        ctx->tcs_ok = tcs_init(fd) == 0;
        if (!ctx->tcs_ok)
            sa_datapump_send_error(pump, "colour",
                "sensehat: TCS34725 colour sensor not found (Sense HAT v1?)");
    }
    if (ctx->joystick) {
        ctx->joy_fd = joy_open();
        if (ctx->joy_fd < 0)
            sa_datapump_send_error(pump, "joystick",
                                   "sensehat: joystick input device not found");
    }
    i2c_set_addr(fd, LSM_AG_ADDR);
}

static void sensehat_reader_thread(size_t arg) {
    sa_datapump *pump = (sa_datapump *)arg;
    sensehat_ctx_t *ctx = (sensehat_ctx_t *)pump->context;

    ctx->i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (ctx->i2c_fd < 0) {
        sa_datapump_send_error(pump, "*",
                               "sensehat: failed to open /dev/i2c-1");
        ctx->done = TRUE;
        return;
    }

    int fd       = ctx->i2c_fd;
    int need_lps = ctx->env_ms > 0;

    if (lsm_init(fd, ctx->odr_bits, ctx->gyro_enabled) < 0) {
        sa_datapump_send_error(pump, "*",
                               "sensehat: LSM9DS1 not found on I2C bus");
        close(fd); ctx->i2c_fd = -1; ctx->done = TRUE;
        return;
    }
    if (need_lps && lps_init(fd) < 0) {
        sa_datapump_send_error(pump, "*",
                               "sensehat: LPS25H not found on I2C bus");
        close(fd); ctx->i2c_fd = -1; ctx->done = TRUE;
        return;
    }
    init_optional(pump, ctx);

    sensehat_polls_t last = {0.0, 0.0, 0.0, 0.0};

    while (ctx->running) {
        poll_aux(pump, ctx, &last);

        int n = lsm_fifo_count(fd);
        if (n <= 0) {
            sa_sys_sleep(200);          /* 200 µs — sub-sample polling */
            continue;
        }

        for (int i = 0; i < n; i++) {
            sensehat_item_t item;
            clock_gettime(CLOCK_REALTIME, &item.tspec);

            /* Gyro must be read before accel to correctly pop the FIFO
               slot when both sensors feed into the FIFO. */
            if (ctx->gyro_enabled) {
                if (lsm_read_gyro(fd, &item.d.vec3.x,
                                  &item.d.vec3.y, &item.d.vec3.z) == 0) {
                    item.flow = FLOW_GYRO;
                    sa_datapump_push(pump, &item);
                }
            }

            if (lsm_read_accel(fd, &item.d.vec3.x,
                                &item.d.vec3.y, &item.d.vec3.z) < 0)
                continue;
            item.flow = FLOW_ACCEL;
            sa_datapump_push(pump, &item);
        }
    }

    if (ctx->joy_fd >= 0) {
        close(ctx->joy_fd);
        ctx->joy_fd = -1;
    }
    close(fd);
    ctx->i2c_fd = -1;
    ctx->done = TRUE;
}

/* ================================================================
 * Pump starter / stopper / stop
 *
 * Global references allow sensehat:stop() to tear down a running
 * pump so that register_sensehat_pump can be called again with
 * different settings.
 * ================================================================ */

static sa_datapump    *g_pump = NULL;
static sensehat_ctx_t *g_ctx  = NULL;

static ohandle sensehat_pump_starter(void *argsp, char *dname) {
    sensehat_reg_args_t *args = (sensehat_reg_args_t *)argsp;
    int odr_bits    = args ? args->odr_bits     : 6;
    int buffer_size = args ? args->buffer_size  : 2048;
    int env_ms      = args ? args->env_ms       : 1000;
    int gyro        = args ? args->gyro_enabled : 0;

    /* Free leftover ctx from a previous stop/restart cycle. */
    if (g_ctx) {
        sa_sys_free(g_ctx);
        g_ctx = NULL;
    }

    sensehat_ctx_t *ctx =
        (sensehat_ctx_t *)sa_sys_malloc(sizeof(sensehat_ctx_t));
    memset(ctx, 0, sizeof(*ctx));
    ctx->running      = TRUE;
    ctx->done         = FALSE;
    ctx->odr_bits     = odr_bits;
    ctx->gyro_enabled = gyro;
    ctx->env_ms       = env_ms;
    ctx->mag_ms       = args ? args->mag_ms    : 0;
    ctx->colour_ms    = args ? args->colour_ms : 0;
    ctx->joystick     = args ? args->joystick  : 0;
    ctx->i2c_fd       = -1;
    ctx->joy_fd       = -1;

    sa_datapump *pump      = sa_datapump_create();
    pump->pump_name        = dname;
    pump->buffer_size      = buffer_size;
    pump->item_size        = sizeof(sensehat_item_t);
    pump->create_item_fn   = sensehat_make_item;
    pump->get_flow_name_fn = sensehat_get_flow_name;
    pump->context          = ctx;
    pump->pump_loop_sleep_time = 100;

    ohandle ret = sa_datapump_init(pump);
    sa_thread_begin(sensehat_reader_thread, (size_t)pump);

    g_pump = pump;
    g_ctx  = ctx;
    return ret;
}

/* Stop the reader thread and wait for it to finish. */
static void sensehat_pump_stopper(sa_datapump *pump) {
    sensehat_ctx_t *ctx = (sensehat_ctx_t *)pump->context;
    ctx->running = FALSE;
    int attempts = 500;                 /* 500 × 10 ms = 5 s timeout */
    while (!ctx->done && attempts-- > 0)
        a_sleep(0.01);
}

/* Tear down the running pump (reader thread + datapump consumer).
   Returns 1 if a pump was stopped, 0 if nothing was running. */
static int stop_running_pump(void) {
    if (!g_pump) return 0;
    sensehat_pump_stopper(g_pump);
    sa_datapump_destroy(g_pump);
    /* Let consumer thread drain and exit before re-use. */
    a_sleep(0.2);
    /* Drop the engine's reference so shutdown does not invoke the
       registered stopper on this already stopped pump. */
    ohandle rname = nil, rres = nil;
    a_setf(rname, mkstring("sensehat"));
    a_setf(rres, call_lisp(mksymbol("reset-ext-pubsub-instance"),
                           NULL, 1, rname));
    a_free(rname);
    a_free(rres);
    g_pump = NULL;
    /* g_ctx freed by next starter call */
    return 1;
}

/*
 * sensehat:stop() -> Integer
 *
 * Tears down the running pump so that register_sensehat_pump can be
 * called again with new settings.
 * Returns 1 if a pump was stopped, 0 if nothing was running.
 */
static ohandle sensehat_stopBF(a_callcontext cxt) {
    a_bind(cxt, 1, mkinteger(stop_running_pump()));
    a_result(cxt);
    return nil;
}

/* Register the "sensehat" pump.  If one is running, stop it first:
   re-registering under a live reader thread used to make the next
   starter free the context that thread still uses (segfault). */
static void register_pump(sensehat_reg_args_t *args) {
    if (stop_running_pump())
        a_sleep(0.5);                   /* let the I2C devices settle */
    sa_datapump_register(args, "sensehat",
                         sensehat_pump_starter, sensehat_pump_stopper);
}

/* ================================================================
 * Foreign function:
 *   register_sensehat_pump(Integer odr, Integer bufsize,
 *                          Integer env_ms, Integer enable_gyro)
 *     -> Charstring
 * ================================================================ */

static ohandle register_sensehat_pumpBBF(a_callcontext cxt) {
    ohandle odr_oh  = a_arg(cxt, 1);
    ohandle buf_oh  = a_arg(cxt, 2);
    ohandle env_oh  = a_arg(cxt, 3);
    ohandle gyro_oh = a_arg(cxt, 4);

    int odr_bits, bufsize, env_ms, gyro;
    IntoInteger32(odr_oh,  odr_bits, a_env(cxt));
    IntoInteger32(buf_oh,  bufsize,  a_env(cxt));
    IntoInteger32(env_oh,  env_ms,   a_env(cxt));
    IntoInteger32(gyro_oh, gyro,     a_env(cxt));

    if (odr_bits < 1 || odr_bits > 6) {
        a_raise_errorno(SENSEHAT_INVALID_ODR, odr_oh);
        return nil;
    }

    sensehat_reg_args_t *args =
        (sensehat_reg_args_t *)sa_sys_malloc(sizeof(sensehat_reg_args_t));
    memset(args, 0, sizeof(*args));
    args->odr_bits     = odr_bits;
    args->buffer_size  = bufsize;
    args->env_ms       = env_ms;
    args->gyro_enabled = gyro ? 1 : 0;
    register_pump(args);

    a_bind(cxt, 5, mkstring("sensehat"));
    a_result(cxt);
    return nil;
}

/* ================================================================
 * Foreign function:
 *   register_sensehat_pump_ex(Integer odr, Integer bufsize,
 *                             Integer env_ms, Integer enable_gyro,
 *                             Integer mag_ms, Integer colour_ms,
 *                             Integer joystick)
 *     -> Charstring
 * ================================================================ */

static ohandle register_sensehat_pump_exBBBBBBBF(a_callcontext cxt) {
    int v[7];
    for (int i = 0; i < 7; i++)
        IntoInteger32(a_arg(cxt, i + 1), v[i], a_env(cxt));

    if (v[0] < 1 || v[0] > 6) {
        a_raise_errorno(SENSEHAT_INVALID_ODR, a_arg(cxt, 1));
        return nil;
    }

    sensehat_reg_args_t *args =
        (sensehat_reg_args_t *)sa_sys_malloc(sizeof(sensehat_reg_args_t));
    args->odr_bits     = v[0];
    args->buffer_size  = v[1];
    args->env_ms       = v[2];
    args->gyro_enabled = v[3] ? 1 : 0;
    args->mag_ms       = v[4] > 0 ? v[4] : 0;
    args->colour_ms    = v[5] > 0 ? v[5] : 0;
    args->joystick     = v[6] ? 1 : 0;
    register_pump(args);

    a_bind(cxt, 8, mkstring("sensehat"));
    a_result(cxt);
    return nil;
}

/* ================================================================
 * LED matrix — the 8x8 RGB565 framebuffer of the rpisense_fb driver
 *
 * Independent of the pump.  The device is found by its fb name and
 * kept open; every call rewrites the whole frame from a shadow copy.
 * ================================================================ */

#define LED_FB_NAME          "RPi-Sense FB"
#define LED_FBIORESET_GAMMA  0xF102     /* rpisense_fb gamma reset ioctl */
#define LED_GAMMA_DEFAULT    0
#define LED_GAMMA_LOW        1

static int      g_led_fd = -1;
static uint16_t g_led[64];

static int led_open(void) {
    if (g_led_fd >= 0) return g_led_fd;
    for (int i = 0; i < 8; i++) {
        char path[64], name[64] = {0};
        snprintf(path, sizeof(path), "/sys/class/graphics/fb%d/name", i);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        if (fgets(name, sizeof(name), f)) name[strcspn(name, "\n")] = 0;
        fclose(f);
        if (strcmp(name, LED_FB_NAME) != 0) continue;
        snprintf(path, sizeof(path), "/dev/fb%d", i);
        g_led_fd = open(path, O_RDWR);
        if (g_led_fd >= 0 &&
            pread(g_led_fd, g_led, sizeof(g_led), 0) != (ssize_t)sizeof(g_led))
            memset(g_led, 0, sizeof(g_led));
        return g_led_fd;
    }
    return -1;
}

static uint16_t rgb565(int r, int g, int b) {
    return (uint16_t)((((r >> 3) & 0x1F) << 11) |
                      (((g >> 2) & 0x3F) << 5)  |
                       ((b >> 3) & 0x1F));
}

/* Write the shadow frame; returns NULL on success, else an error text. */
static const char *led_flush(void) {
    if (led_open() < 0)
        return "sensehat: LED framebuffer '" LED_FB_NAME "' not found";
    if (pwrite(g_led_fd, g_led, sizeof(g_led), 0) != (ssize_t)sizeof(g_led))
        return "sensehat: failed to write LED framebuffer";
    return NULL;
}

/* Open the device (which loads the shadow copy from the display) BEFORE a
   caller modifies the shadow; otherwise the first write after opening
   would be overwritten by the old frame. */
#define LED_OPEN_OR_THROW(cxt)                                              \
    if (led_open() < 0)                                                     \
        return a_throw_errormsg(a_env(cxt),                                 \
            "sensehat: LED framebuffer '" LED_FB_NAME "' not found", nil)

/* One colour component: an Integer (or Real, truncated) in 0-255. */
static int led_component(ohandle h, int *out) {
    double v;
    if (integerp(h))   v = (double)getinteger(h);
    else if (realp(h)) v = getreal(h);
    else return -1;
    if (v < 0 || v >= 256) return -1;   /* range-check before narrowing */
    *out = (int)v;
    return 0;
}

/* Read r,g,b components 0-255 from three handles into RGB565.
   Returns 0, or the position (1-3) of the first invalid component. */
static int led_rgb(ohandle r_oh, ohandle g_oh, ohandle b_oh, uint16_t *out) {
    int r, g, b;
    if (led_component(r_oh, &r) < 0) return 1;
    if (led_component(g_oh, &g) < 0) return 2;
    if (led_component(b_oh, &b) < 0) return 3;
    *out = rgb565(r, g, b);
    return 0;
}

/* sensehat:led_clear(Integer r, Integer g, Integer b) -> Integer */
static ohandle sensehat_led_clearBBBF(a_callcontext cxt) {
    uint16_t c;
    int bad;
    LED_OPEN_OR_THROW(cxt);
    if ((bad = led_rgb(a_arg(cxt, 1), a_arg(cxt, 2), a_arg(cxt, 3), &c)))
        return a_throw_errormsg(a_env(cxt), "sensehat: colour must be 0-255",
                                a_arg(cxt, bad));
    for (int i = 0; i < 64; i++) g_led[i] = c;
    const char *err = led_flush();
    if (err) return a_throw_errormsg(a_env(cxt), (char *)err, nil);
    a_bind(cxt, 4, mkinteger(1));
    a_result(cxt);
    return nil;
}

/* sensehat:led_pixel(Integer x, Integer y, Integer r, Integer g, Integer b)
     -> Integer      (x = column, y = row, both 0-7) */
static ohandle sensehat_led_pixelBBBBBF(a_callcontext cxt) {
    int x, y, bad;
    uint16_t c;
    LED_OPEN_OR_THROW(cxt);
    IntoInteger32(a_arg(cxt, 1), x, a_env(cxt));
    IntoInteger32(a_arg(cxt, 2), y, a_env(cxt));
    if (x < 0 || x > 7 || y < 0 || y > 7)
        return a_throw_errormsg(a_env(cxt), "sensehat: x and y must be 0-7",
                                a_arg(cxt, x < 0 || x > 7 ? 1 : 2));
    if ((bad = led_rgb(a_arg(cxt, 3), a_arg(cxt, 4), a_arg(cxt, 5), &c)))
        return a_throw_errormsg(a_env(cxt), "sensehat: colour must be 0-255",
                                a_arg(cxt, 2 + bad));
    g_led[y * 8 + x] = c;
    const char *err = led_flush();
    if (err) return a_throw_errormsg(a_env(cxt), (char *)err, nil);
    a_bind(cxt, 6, mkinteger(1));
    a_result(cxt);
    return nil;
}

/* sensehat:led_pixels(Vector px) -> Integer
     px = 64 x [r, g, b], row-major (index = y*8 + x) */
static ohandle sensehat_led_pixelsBF(a_callcontext cxt) {
    ohandle px = a_arg(cxt, 1);
    LED_OPEN_OR_THROW(cxt);
    if (a_arraysize(px) != 64)
        return a_throw_errormsg(a_env(cxt),
                                "sensehat: led_pixels needs 64 [r,g,b] vectors", px);
    uint16_t frame[64];
    for (int i = 0; i < 64; i++) {
        ohandle p = a_elt(px, i);
        if (!arrayp(p) || a_arraysize(p) != 3 ||
            led_rgb(a_elt(p, 0), a_elt(p, 1), a_elt(p, 2), &frame[i]))
            return a_throw_errormsg(a_env(cxt),
                                    "sensehat: each pixel must be [r,g,b] in 0-255", p);
    }
    memcpy(g_led, frame, sizeof(g_led));
    const char *err = led_flush();
    if (err) return a_throw_errormsg(a_env(cxt), (char *)err, nil);
    a_bind(cxt, 2, mkinteger(1));
    a_result(cxt);
    return nil;
}

/* sensehat:led_get_pixels() -> Vector of 64 [r, g, b] read from the device */
static ohandle sensehat_led_get_pixelsF(a_callcontext cxt) {
    LED_OPEN_OR_THROW(cxt);
    uint16_t frame[64];
    if (pread(g_led_fd, frame, sizeof(frame), 0) != (ssize_t)sizeof(frame))
        return a_throw_errormsg(a_env(cxt),
                                "sensehat: failed to read LED framebuffer", nil);
    ohandle res = new_array(64, nil);
    for (int i = 0; i < 64; i++) {
        ohandle p = new_array(3, nil);
        a_seta(p, 0, mkinteger((frame[i] >> 8) & 0xF8));
        a_seta(p, 1, mkinteger((frame[i] >> 3) & 0xFC));
        a_seta(p, 2, mkinteger((frame[i] << 3) & 0xF8));
        a_seta(res, i, p);
    }
    a_bind(cxt, 1, res);
    a_result(cxt);
    return nil;
}

/* sensehat:led_low_light(Integer on) -> Integer  (driver gamma table) */
static ohandle sensehat_led_low_lightBF(a_callcontext cxt) {
    int on;
    IntoInteger32(a_arg(cxt, 1), on, a_env(cxt));
    LED_OPEN_OR_THROW(cxt);
    if (ioctl(g_led_fd, LED_FBIORESET_GAMMA,
              on ? LED_GAMMA_LOW : LED_GAMMA_DEFAULT) < 0)
        return a_throw_errormsg(a_env(cxt),
                                "sensehat: failed to set LED gamma", a_arg(cxt, 1));
    a_bind(cxt, 2, mkinteger(on ? 1 : 0));
    a_result(cxt);
    return nil;
}

/* ================================================================
 * Extension entry point
 * ================================================================ */

EXPORT void a_initialize_extension() {
    load_accel_cal();
    a_extimpl("register-sensehat-pump----+", register_sensehat_pumpBBF);
    a_extimpl("register-sensehat-pump-ex-------+",
              register_sensehat_pump_exBBBBBBBF);
    a_extimpl("sensehat-stop+",              sensehat_stopBF);
    a_extimpl("sensehat-led-clear---+",      sensehat_led_clearBBBF);
    a_extimpl("sensehat-led-pixel-----+",    sensehat_led_pixelBBBBBF);
    a_extimpl("sensehat-led-pixels-+",       sensehat_led_pixelsBF);
    a_extimpl("sensehat-led-get-pixels+",    sensehat_led_get_pixelsF);
    a_extimpl("sensehat-led-low-light-+",    sensehat_led_low_lightBF);
    SENSEHAT_INVALID_ODR =
        a_register_error("sensehat: ODR must be 1-6");
}
