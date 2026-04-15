/*
 * sa.rpi-sensehat.c — SA Engine datapump extension for Raspberry Pi Sense HAT v1
 *
 * Uses the sa_datapump infrastructure with per-signal flow routing.
 * A background thread reads the LSM9DS1 IMU FIFO and (optionally) the
 * LPS25H barometer, pushing tagged items into a circular buffer.
 * SA Engine consumers select desired signals via subscribe:merge.
 *
 * Flow names (topics under the "sensehat" pump):
 *   sensehat:accel    — Timeval([ax,ay,az])         at IMU ODR (always on)
 *   sensehat:gyro     — Timeval([gx,gy,gz])         at IMU ODR (if enabled)
 *   sensehat:pressure — Timeval(hPa)                at env_ms  (if > 0)
 *   sensehat:temp     — Timeval(°C)                 at env_ms  (if > 0)
 *
 * Exposed OSQL foreign functions (see sa.rpi-sensehat.osql for full bindings):
 *
 *   register_sensehat_pump(Integer odr, Integer bufsize,
 *                          Integer env_ms, Integer enable_gyro)
 *     -> Charstring
 *
 *     odr         — LSM9DS1 output data rate 1-6 (see table below)
 *     bufsize     — circular-buffer capacity in items
 *     env_ms      — pressure/temp polling interval in ms (0 = disabled)
 *     enable_gyro — 0 = accel only, 1 = accel + gyro
 *
 *   sensehat:stop() -> Integer
 *
 *     Tears down the running pump so register_sensehat_pump can be
 *     called again with new settings.  Returns 1 if a pump was
 *     stopped, 0 if nothing was running.
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
#include "sa_syscalls.h"
#include "sa_threads.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* a_timestamp is EXPORT in sa.engine/system/C/a_time.c but not in sa_core.h */
extern ohandle a_timestamp(time_t seconds, int useconds, ohandle o);

/* ---- I2C addresses ---- */
#define LSM_AG_ADDR 0x6A   /* LSM9DS1 accel/gyro */
#define LPS_ADDR    0x5C   /* LPS25H pressure    */

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

/* ---- Conversion factors ---- */
#define ACCEL_SCALE (16.0 / 32768.0)   /* ±16 g     */
#define GYRO_SCALE  (2000.0 / 32768.0) /* ±2000 dps */

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
    FLOW_TEMP
} sensehat_flow_t;

static const char *flow_names[] = {"accel", "gyro", "pressure", "temp"};

typedef struct {
    struct timespec tspec;
    sensehat_flow_t flow;
    union {
        struct { double x, y, z; } vec3;   /* accel or gyro  */
        double scalar;                      /* pressure or temp */
    } d;
} sensehat_item_t;

/* ================================================================
 * Pump context — carried in pump->context
 * ================================================================ */

typedef struct {
    volatile int running;
    volatile int done;
    int  odr_bits;
    int  gyro_enabled;
    int  env_ms;           /* 0 = pressure/temp disabled */
    int  i2c_fd;
} sensehat_ctx_t;

/* ================================================================
 * Registration arguments — heap-allocated, read once by starter
 * ================================================================ */

typedef struct {
    int odr_bits;
    int buffer_size;
    int env_ms;
    int gyro_enabled;
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

    /* Accel: selected ODR, ±16 g */
    i2c_write_byte(fd, LSM_CTRL_REG6_XL, (odr_bits << 5) | 0x08);
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
    *ax = to_i16(buf[0], buf[1]) * ACCEL_SCALE;
    *ay = to_i16(buf[2], buf[3]) * ACCEL_SCALE;
    *az = to_i16(buf[4], buf[5]) * ACCEL_SCALE;
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
 *   accel/gyro  → ["accel", [x, y, z]]  or  ["gyro", [x, y, z]]
 *   pressure    → ["pressure", hPa]
 *   temperature → ["temp", °C]
 */
static void sensehat_make_item(ohandle *ret, sa_datapump *pump, void *data) {
    (void)pump;
    sensehat_item_t *item = (sensehat_item_t *)data;
    ohandle payload = nil;

    switch (item->flow) {
    case FLOW_ACCEL:
    case FLOW_GYRO: {
        ohandle v = new_array(3, nil);
        a_seta(v, 0, mkreal(item->d.vec3.x));
        a_seta(v, 1, mkreal(item->d.vec3.y));
        a_seta(v, 2, mkreal(item->d.vec3.z));
        payload = v;
        break;
    }
    case FLOW_PRESSURE:
    case FLOW_TEMP:
        payload = mkreal(item->d.scalar);
        break;
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
 * Owns the I2C file descriptor for its lifetime.  Reads the
 * LSM9DS1 FIFO (accel, optionally gyro) and periodically the
 * LPS25H (pressure, temperature).  Each reading is pushed as a
 * tagged item into the datapump circular buffer.
 * ================================================================ */

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

    double env_interval_s = ctx->env_ms / 1000.0;
    double last_lps = 0.0;

    while (ctx->running) {
        i2c_set_addr(fd, LSM_AG_ADDR);
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

        /* ---- Pressure / temperature polling ---- */
        if (need_lps) {
            struct timespec now_ts;
            clock_gettime(CLOCK_REALTIME, &now_ts);
            double now = now_ts.tv_sec + now_ts.tv_nsec * 1e-9;

            if (now - last_lps >= env_interval_s) {
                last_lps = now;
                double pressure, temp;
                if (lps_read(fd, &pressure, &temp) == 0) {
                    sensehat_item_t env_item;
                    env_item.tspec = now_ts;

                    env_item.flow     = FLOW_PRESSURE;
                    env_item.d.scalar = pressure;
                    sa_datapump_push(pump, &env_item);

                    env_item.flow     = FLOW_TEMP;
                    env_item.d.scalar = temp;
                    sa_datapump_push(pump, &env_item);
                }
                i2c_set_addr(fd, LSM_AG_ADDR);
            }
        }
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
    ctx->running      = TRUE;
    ctx->done         = FALSE;
    ctx->odr_bits     = odr_bits;
    ctx->gyro_enabled = gyro;
    ctx->env_ms       = env_ms;
    ctx->i2c_fd       = -1;

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

/*
 * sensehat:stop() -> Integer
 *
 * Tears down the running pump (reader thread + datapump consumer)
 * so that register_sensehat_pump can be called again with new settings.
 * Returns 1 if a pump was stopped, 0 if nothing was running.
 */
static ohandle sensehat_stopBF(a_callcontext cxt) {
    int stopped = 0;
    if (g_pump) {
        sensehat_pump_stopper(g_pump);
        sa_datapump_destroy(g_pump);
        /* Let consumer thread drain and exit before re-use. */
        a_sleep(0.2);
        g_pump = NULL;
        /* g_ctx freed by next starter call */
        stopped = 1;
    }
    a_bind(cxt, 1, mkinteger(stopped));
    a_result(cxt);
    return nil;
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
    args->odr_bits     = odr_bits;
    args->buffer_size  = bufsize;
    args->env_ms       = env_ms;
    args->gyro_enabled = gyro ? 1 : 0;

    sa_datapump_register(args, "sensehat",
                         sensehat_pump_starter, sensehat_pump_stopper);

    a_bind(cxt, 5, mkstring("sensehat"));
    a_result(cxt);
    return nil;
}

/* ================================================================
 * Extension entry point
 * ================================================================ */

EXPORT void a_initialize_extension() {
    a_extimpl("register-sensehat-pump----+", register_sensehat_pumpBBF);
    a_extimpl("sensehat-stop+",              sensehat_stopBF);
    SENSEHAT_INVALID_ODR =
        a_register_error("sensehat: ODR must be 1-6");
}
