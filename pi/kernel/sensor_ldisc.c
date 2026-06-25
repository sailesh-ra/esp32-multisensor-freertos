/*
 * sensor_ldisc.c - Custom TTY line discipline for ESP32 sensor data
 * 
 * DESIGN OVERVIEW
 * --------------------------------------------------------------------
 * This driver registers a custom TTY line discipline (ldisc). When
 * attached to /dev/ttyAMA0, it intercepts the byte stream from the ESP32,
 * parses sensor frames, and exposes the decoded data through a character 
 * device at /dev/sensor_data.
 * 
 *  UART hw -> ttyAMA0 -> [sensor_ldisc] -> /dev/sensor_data
 *                              |
 *                       receive_buf() called by TTY core
 *                       parses ASCII / binary / JSON frames 
 *                       wakes blocked readers via waitqueue
 * 
 * SUPPORTED PROTOCOLS (select via 'proto' module param or ioctl)
 *  0 = ASCII   "T:+023.50, D:00145.3, S:000001\n"
 *  1 = binary  10-byte framed packet (0xAA 0x55 ... CRC-8)
 *  2 = JSON    {"seq":1, "t":23.50, "d":145.3}\n
 *
 * QUICK START
 *  make && sudo insmod sensor_ldisc.ko
 *  sudo ldattach -8 -s 115200 28 /dev/ttyAMA0      # attach ldisc
 *  cat /dev/sensor_data                            # read decoded frames
 *
 * IOCTL INTERFACE
 *  SENSOR_IOC_SETPROTO     - switch protocol at runtime (no reload needed)
 *  SENSOR_IOC_GETREADING   - non-blocking snapshot of latest reading
 * 
 * REFERENCE COUNTING
 *  kref is used to safely share the per-tty context between the 
 *  ldisc callbacks and open char device file descriptors. The 
 *  context is freed only when both the ldisc is detached AND all
 *  char device fds are closed.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/ktime.h>
#include <linux/string.h>

#include "../include/sensor_ioctl.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sailesh Rachapudi");
MODULE_DESCRIPTION("TTY line discipline + char device for ESP32 sensor bridge");
MODULE_VERSION("1.0");

#define DRIVER_NAME     "sensor_ldisc"
#define DEVICE_NAME     "sensor_data"

/* ----- Module parameters -------------------------------------------- */
static int ldisc_num = 28;
module_param(ldisc_num, int, 0444);
MODULE_PARM_DESC(ldisc_num, "TTY line discipline number (default 28)");

static int proto = SENSOR_PROTO_ASCII;
module_param(proto, int, 0644);
MODULE_PARM_DESC(proto, "Active protocol: 0=ASCII (default), 1=binary, 2=JSON");

/* ------Binary frame constants --------------------------------------- */
#define BIN_MAGIC_H     0xAAU
#define BIN_MAGIC_L     0x55U
#define BIN_CRC_START   2
#define BIN_CRC_LEN     7

/* ------- Parser state machine ---------------------------------------- */
enum bin_state {
    S_MAGIC1 = 0,
    S_MAGIC2,
    S_SEQ,
    S_TYPE,
    S_LEN,
    S_PAYLOAD,
    S_CRC,
};

#define RX_BUF_SIZE     256

/*
 * Per-tty context.
 * Lifetime: allocated on ldisc_open, freed via kref when both
 * the ldisc is detached AND all open char device fds are closed.
 */
struct ldisc_ctx {
    struct tty_struct   *tty;
    struct kref         refcount;

    /* ASCII/JSON accumulation buffer */
    uint8_t rx_buf[RX_BUF_SIZE];
    int     rx_pos;

    /* Binary state machine */
    enum bin_state  current_state;
    uint8_t     bin_frame[16];
    int         bin_frame_pos;
    int         bin_payload_expected;

    /* Latest decoded reading (spin-lock protected) */
    spinlock_t              lock;
    struct sensor_reading   latest;
    bool                    data_ready; /* set by receive_buf, cleared by read() */

    /* Blocking read / poll support */
    wait_queue_head_t       read_wq;
};

/* ----- Globals ------------------------------------------------ */
static dev_t            sensor_devno;
static struct class    *sensor_class;
static struct cdev      sensor_cdev;

/* g_ctx: Non-NULL when a tty haas the ldisc attaached. 
 * Protected by g_ctx_lock for get/set; kref handles memory.      */
 static DEFINE_SPINLOCK(g_ctx_lock);
 static struct ldisc_ctx *g_ctx;

/* ================================================================== 
 *  CRC-8/SMBUS     (poly 0x07, init 0x00, no reflection)
 *  Must match the ESP32's crc8_smbus() exactly.
 * ================================================================== */
static uint8_t crc8_smbus(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    int i, j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}

/* ---- kref helpers ----------------------------------------------- */
static void ctx_release(struct kref *kref)
{
    struct ldisc_ctx *ctx = container_of(kref, struct ldisc_ctx, refcount);

    pr_debug("ctx freed\n");
    kfree(ctx);
}

/* Returns ctx with incremented refcount, or NULL if no ldisc attached. 
 * Caller must call ctx_put() when done.                               */
static struct ldisc_ctx *ctx_get(void)
{
    struct ldisc_ctx *ctx;
    unsigned long flags;

    spin_lock_irqsave(&g_ctx_lock, flags);
    ctx = g_ctx;
    if (ctx) kref_get(&ctx->refcount);
    spin_unlock_irqrestore(&g_ctx_lock, flags);

    return ctx;
}

static void ctx_put(struct ldisc_ctx *ctx)
{
    kref_put(&ctx->refcount, ctx_release);
}

/* ================================================================ 
 * Commit a completed reading - called from all three parsers.
 * Safe to call from any context (spinlock + wakeup).
 * ================================================================ */
static void commit_reading(struct ldisc_ctx *ctx,
                            int32_t t100, uint32_t d10, uint32_t seq)
{
    unsigned long flags;

    spin_lock_irqsave(&ctx->lock, flags);
    ctx->latest.temp_x100       = t100;
    ctx->latest.dist_x10        = d10;
    ctx->latest.seq             = seq;
    ctx->latest.timestamp_ms     = (uint64_t)ktime_to_ms(ktime_get());
    WRITE_ONCE(ctx->data_ready, true);
    spin_unlock_irqrestore(&ctx->lock, flags);
    
    wake_up_interruptible(&ctx->read_wq);
}

/* ============================================================ 
 * ASCII parser
 * Input (newline stripped):    "T:+023.50,D:00145.3,S:000001"
 * No floating point - integer-only arithmetic through out.
   ============================================================ */
static int parse_ascii(struct ldisc_ctx *ctx, const char *buf, int len)
{
    const char *p = buf;
    int sign      = 1, fd;
    long v;
    int32_t t100 = 0;
    uint32_t d10 = 0, seq = 0;

    if (len < 20)                   return -EINVAL;
    if (p[0] != 'T' || p[1] != ':') return -EINVAL;
    p += 2;

    /* Temperature: optional sign, integer digits, '.', 2 frac digits */
    if (*p == '-')      { sign = -1; p++; }
    else if (*p == '+') { p++; }

    v = 0;
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    t100 = sign * v * 100;
    if (*p++ != '.') return -EINVAL;
    v = 0; fd = 0;
    while (*p >= '0' && *p <= '9' && fd < 2) { v = v * 10 + (*p++ - '0'); fd++;}
    while (fd++ < 2) v *= 10;
    t100 += sign * (int32_t)v;

    /* Distance: no sign, integer digits, '.', 1 frac digit */
    if (*p++ != ',') return -EINVAL;
    if (*p++ != 'D') return -EINVAL;
    if (*p++ != ':') return -EINVAL;
    v = 0;
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    d10 = (uint32_t)v * 10;
    if (*p == '.') {
        p++;
        d10 += (uint32_t)(*p++ - '0');
    }

    /* Sequence number */
    if (*p++ != ',') return -EINVAL;
    if (*p++ != 'S') return -EINVAL;
    if (*p++ != ':') return -EINVAL;
    while (*p >= '0' && *p <= '9') seq = seq * 10 + (uint32_t)(*p++ - '0');

    commit_reading(ctx, t100, d10, seq);
    return 0;
}

/* ===========================================================
 * JSON parser (fixed-key, no general-purpose JSON needed)
 * Input:   {"seq":1,"t":23.50,"d":145.3}
 * ============================================================ */
static int parse_json(struct ldisc_ctx *ctx, const char *buf, int len)
{
    const char *p;
    int sign, fd;
    long v;
    int32_t t100 = 0;
    uint32_t d10 = 0, seq = 0;

    /* "seq": */
    p = strstr(buf, "\"seq\":");
    if (!p) return -EINVAL;
    p += 6;
    while (*p >= '0' && *p <= '9') seq = seq * 10 + (uint32_t)(*p++ - '0');

    /* "t": */
    p = strstr(buf, "\"t\":");
    if (!p) return -EINVAL;
    p += 4;
    sign = 1;
    if (*p == '-')      { sign = -1; p++; }
    else if (*p == '+') { p++; }
    v = 0;
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    t100 = sign * v * 100;
    if (*p == '.') {
        p++; v = 0; fd = 0;
        while (*p >= '0' && *p <= '9' && fd < 2) { v = v * 10 + (*p++ - '0'); fd++; }
        while (fd++ < 2) v *= 10;
        t100 += sign * (int32_t)v;
    }

    /* "d": */
    p = strstr(buf, "\"d\":");
    if (!p) return -EINVAL;
    p += 4;
    v = 0;
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    d10 = (uint32_t)v * 10;
    if (*p == '.') {
        p++;
        d10 += (uint32_t)(*p++ - '0');
    }

    commit_reading(ctx, t100, d10, seq);
    return 0;
}

/* ================================================================
 * Binary frame parser - byte at a time state machine.
 * 
 * Frame layout (10 bytes):
 * [0]      0xAA          magic high
 * [1]      0x55          magic low
 * [2]      seq   u8      sequence (wraps 0-255)
 * [3]      type  u8      0x01 = combined temp+dist
 * [4]      plen  u8      0x04 = payload length
 * [5-6]    temp  i16le   temperature x 100 (signed little-endian)
 * [7-8]    dist  u16le   distance x 10     (unsigned little-endian)
 * [9]      crc   u8      CRC-8/SMBUS over bytes [2..8]
 * ================================================================= */
static void process_bin_byte(struct ldisc_ctx *ctx, uint8_t b)
{
    switch (ctx->current_state) {

        case S_MAGIC1:
            if (b == BIN_MAGIC_H) {
                ctx->bin_frame[0]   = b;
                ctx->current_state  = S_MAGIC2;
            }
            break;

        case S_MAGIC2:
            ctx->bin_frame[1]  = b;
            ctx->current_state = (b == BIN_MAGIC_L) ? S_SEQ : S_MAGIC1;
            break;
        
        case S_SEQ:
            ctx->bin_frame[2]   = b;
            ctx->current_state  = S_TYPE;
            break;

        case S_TYPE:
            ctx->bin_frame[3]   = b;
            ctx->current_state  = S_LEN;
            break;
        
        case S_LEN:
            if (b != 4) {           /* unexpected payload length - resync */
                ctx->current_state = S_MAGIC1;
                break;
            }
            ctx->bin_frame[4]           = b;
            ctx->bin_payload_expected   = (int)b;
            ctx->bin_frame_pos          = 5;
            ctx->current_state          = S_PAYLOAD;
            break;
        
        case S_PAYLOAD:
            ctx->bin_frame[ctx->bin_frame_pos++] = b;
            if (ctx->bin_frame_pos == 5 + ctx->bin_payload_expected) ctx->current_state = S_CRC;
            break;

        case S_CRC: {
            uint8_t expect = crc8_smbus(ctx->bin_frame + BIN_CRC_START,
                                         BIN_CRC_LEN);
            if(b == expect) {
                /* Extract little-endian signed temp and unsigned dist */
                int16_t t = (int16_t)((uint16_t)ctx->bin_frame[5] | 
                                        ((uint16_t)ctx->bin_frame[6] << 8));
                uint16_t d = (uint16_t)ctx->bin_frame[7] | 
                                ((uint16_t)ctx->bin_frame[8] << 8);
                
                commit_reading(ctx, (int32_t)t, (uint32_t)d, ctx->bin_frame[2]);
            } else {
                pr_warn_ratelimited("CRC mismatch  got=0x%02X  want=0x%02X\n",
                                        b, expect);
            }
            ctx->current_state = S_MAGIC1;
            break;
        }

        default:
            ctx->current_state = S_MAGIC1;
            break;     
    }
}

/* ====================================================================
 *  TTY line discipline callbacks
 * ==================================================================== */

static int sensor_ldisc_open(struct tty_struct *tty)
{
    struct ldisc_ctx *ctx;
    unsigned long flags;

    spin_lock_irqsave(&g_ctx_lock, flags);
    if (g_ctx) {
        spin_unlock_irqrestore(&g_ctx_lock, flags);
        pr_err("already attached to a tty\n");
        return -EBUSY;
    }
    spin_unlock_irqrestore(&g_ctx_lock, flags);

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->tty            = tty;
    ctx->current_state  = S_MAGIC1;
    kref_init(&ctx->refcount);          /* refcount = 1 (owned by ldisc) */
    spin_lock_init(&ctx->lock);
    init_waitqueue_head(&ctx->read_wq);

    tty->disc_data = ctx;

    spin_lock_irqsave(&g_ctx_lock, flags);
    g_ctx = ctx;
    spin_unlock_irqrestore(&g_ctx_lock, flags);

    pr_info("attached to %s  (proto=%d)\n", tty->name, proto);
    return 0;
}

static void sensor_ldisc_close(struct tty_struct *tty)
{
    struct ldisc_ctx *ctx = tty->disc_data;
    unsigned long flags;

    if(!ctx) return;

    /* Remove from global pointer so new ctx_get() calls return NULL */
    spin_lock_irqsave(&g_ctx_lock, flags);
    g_ctx           = NULL;
    tty->disc_data  = NULL;
    spin_unlock_irqrestore(&g_ctx_lock, flags);

    /* Wake any blocked readers so they can return -ENODEV */
    wake_up_interruptible(&ctx->read_wq);

    /* Drop the ldisc's reference; char device fds hold their own refs */
    ctx_put(ctx);

    pr_info("detached\n");
}

/*
 * Called by the TTY core when bytes arrive from the UART hardware.
 * This runs in interrupt context on same platforms - keep it quick
 * and spinlock-safe (no sleeping, no mutex).
 */
static size_t sensor_ldisc_receive_buf2(struct tty_struct *tty,
                                    const unsigned char *cp,
                                    const u8 *fp, size_t count)
{
    struct ldisc_ctx *ctx = tty->disc_data;
    int i;

    if (!ctx) return 0;

    for (i = 0; i < count; i++) {
        uint8_t b = cp[i];

        if(proto == SENSOR_PROTO_BINARY) {
            process_bin_byte(ctx, b);

        } else {    /* ASCII or JSON: accumulate until newline */
            if (b == '\r')
                continue;
            
            if(b == '\n') {
                if (ctx->rx_pos > 0) {
                    ctx->rx_buf[ctx->rx_pos] = '\0';
                    if (proto == SENSOR_PROTO_JSON)
                        parse_json(ctx, ctx->rx_buf, ctx->rx_pos);
                    else
                        parse_ascii(ctx, ctx->rx_buf, ctx->rx_pos);
                }
                ctx->rx_pos = 0;
            } else if (ctx->rx_pos < RX_BUF_SIZE - 1) {
                ctx->rx_buf[ctx->rx_pos++] = b;
            } else {
                ctx->rx_pos = 0;        /* overflow - discard and resync */
                pr_warn_ratelimited("rx buffer overflow - discarding\n");
            }
        }
    }
    return count;
}

static struct tty_ldisc_ops sensor_ldisc_ops = {
    .owner          = THIS_MODULE,
    .name           = DRIVER_NAME,
    /* .num set dynamically in module_init from ldisc_num param */
    .open           = sensor_ldisc_open,
    .close         = sensor_ldisc_close,
    .receive_buf2    = sensor_ldisc_receive_buf2,
};

/* ==================================================================
 *  Character device: /dev/sensor_data
 * ================================================================== */

static int sensor_dev_open(struct inode *inode, struct file *filp)
{
    struct ldisc_ctx *ctx = ctx_get();      /* bumps refcount if attached */

    if (!ctx) return -ENODEV;

    filp->private_data = ctx;
    return 0;
}

static int sensor_dev_release(struct inode *inode, struct file *filp)
{
    struct ldisc_ctx *ctx = filp->private_data;

    if (ctx) ctx_put(ctx);
    return 0;
}

/* 
 *  Blocking read: waits until a new frame arrives, then returns
 *  a human-readable line.   Clears data_ready after consuming.
 * 
 *  Example output:
 *      "T: +23.50 C    D:145.3 cm  SEQ:42  TS:123456 ms\n"
 */
static ssize_t sensor_dev_read(struct file *filp, char __user *buf,
                                size_t count, loff_t *ppos)
{
    struct ldisc_ctx *ctx = filp->private_data;
    struct sensor_reading snap;
    unsigned long flags;
    char out[96];
    int len, t_abs, t_int, t_frac;

    if (!ctx) return -ENODEV;

    /* Block until data arrives or ldisc detaches */
    if (wait_event_interruptible(ctx->read_wq, 
                                    READ_ONCE(ctx->data_ready) || !g_ctx))
        return -ERESTARTSYS;
    
    if(!g_ctx && !READ_ONCE(ctx->data_ready)) return -ENODEV;

    spin_lock_irqsave(&ctx->lock,flags);
    snap = ctx->latest;
    WRITE_ONCE(ctx->data_ready,false);
    spin_unlock_irqrestore(&ctx->lock, flags);

    /* Integer-only formatting (no float in kernel) */
    t_abs   = (snap.temp_x100 < 0) ? -(int)snap.temp_x100 : (int)snap.temp_x100;
    t_int   = t_abs / 100;
    t_frac  = t_abs % 100;

    len = snprintf(out, sizeof(out),
                    "T:%c%d.%02d C  D:%u.%u cm  SEQ:%u  TS:%llu ms\n",
                    (snap.temp_x100 < 0) ? '-' : '+',
                    t_int, t_frac,
                    snap.dist_x10 / 10, snap.dist_x10 % 10,
                    snap.seq, snap.timestamp_ms);

    if ((size_t)len > count) return -EINVAL;
    if (copy_to_user(buf, out, (unsigned long)len)) return -EFAULT;

    return (ssize_t)len;
}

/* poll/select support - lets userspace do event-driven reads */
static __poll_t sensor_dev_poll(struct file *filp, poll_table *wait)
{
    struct ldisc_ctx *ctx = filp->private_data;

    if (!ctx) return EPOLLERR;

    poll_wait(filp, &ctx->read_wq, wait);

    if (READ_ONCE(ctx->data_ready)) return EPOLLIN | EPOLLRDNORM;

    return 0;
}

/* 
 * ioctl interface:
 *  SENSOR_IOC_SETPROTO     -- switch protocol without reloading module
 *  SENSOR_IOC_GETREADING   -- non-blocking snapshot (raw fixed-point)
 */
static long sensor_dev_ioctl(struct file *filp, unsigned int cmd,
                                unsigned long arg)
{
    struct ldisc_ctx *ctx = filp->private_data;

    switch(cmd) {

        case SENSOR_IOC_SETPROTO: {
            int p = (int)arg;
            if (p < 0 || p > 2) return -EINVAL;
            proto = p;
            pr_info("protocol switched to %d\n", proto);
            /* Reset ASCII accumulator to avoid stale partial line */
            if (ctx) ctx->rx_pos = 0;
            return 0;
        }

        case SENSOR_IOC_GETREADING: {
            struct sensor_reading snap;
            unsigned long flags;

            if(!ctx) return -ENODEV;

            spin_lock_irqsave(&ctx->lock, flags);
            snap = ctx->latest;
            spin_unlock_irqrestore(&ctx->lock, flags);

            if(copy_to_user((void __user *)arg, &snap, sizeof(snap)))
                return -EFAULT;
            return 0;
        }

        default:
            return -ENOTTY;
    }
}

static const struct file_operations sensor_fops = {
    .owner          = THIS_MODULE,
    .open           = sensor_dev_open,
    .release        = sensor_dev_release,
    .read           = sensor_dev_read,
    .poll           = sensor_dev_poll,
    .unlocked_ioctl = sensor_dev_ioctl,
};

/* ============================================================== 
 * Module init / exit
 * ==============================================================*/

static int __init sensor_ldisc_module_init(void)
{
    int ret;

    /* 1. Register the line discipline */
    sensor_ldisc_ops.num = ldisc_num;
    ret = tty_register_ldisc(&sensor_ldisc_ops);
    if (ret) {
        pr_err("failed to register ldisc %d     err=%d\n", ldisc_num, ret);
        return ret;
    }

    /* 2. Allocate a dynamic major: minor for /dev/sensor_data */
    ret = alloc_chrdev_region(&sensor_devno, 0, 1, DEVICE_NAME);
    if (ret) {
        pr_err("alloc_chrdev_region failed      err=%d\n", ret);
        goto err_ldisc;
    }

    /* 3. Create /sys/class/sensor_data */
    sensor_class = class_create(DEVICE_NAME);
    if (IS_ERR(sensor_class)) {
        ret = (int)PTR_ERR(sensor_class);
        pr_err("class_create failed err=%d\n", ret);
        goto err_chrdev;
    }

    /* 4. Register the char device */
    cdev_init(&sensor_cdev, &sensor_fops);
    sensor_cdev.owner = THIS_MODULE;
    ret = cdev_add(&sensor_cdev, sensor_devno, 1);
    if (ret) {
        pr_err("cdev_add failed    err=%d\n", ret);
        goto err_class;
    }

    /* 5. Create /dev/sensor_data via udev */
    if (IS_ERR(device_create(sensor_class, NULL, sensor_devno,
                                NULL, DEVICE_NAME))) 
    {
        ret = -ENODEV;
        pr_err("device_create failed\n");
        goto err_cdev;
    }

    pr_info("loaded ldisc=%d  /dev/%s  major=%d  proto=%d\n",
            ldisc_num, DEVICE_NAME, MAJOR(sensor_devno), proto);
    pr_info("attach: sudo ldattach -8 -s 115200 %d /dev/ttyAMA0\n",
            ldisc_num);
    return 0;

err_cdev:       cdev_del(&sensor_cdev);
err_class:      class_destroy(sensor_class);
err_chrdev:     unregister_chrdev_region(sensor_devno, 1);
err_ldisc:      tty_unregister_ldisc(&sensor_ldisc_ops);
        return ret;
}

static void __exit sensor_ldisc_module_exit(void)
{
    device_destroy(sensor_class, sensor_devno);
    cdev_del(&sensor_cdev);
    class_destroy(sensor_class);
    unregister_chrdev_region(sensor_devno, 1);
    tty_unregister_ldisc(&sensor_ldisc_ops);
    pr_info("unloaded\n");
}

module_init(sensor_ldisc_module_init);
module_exit(sensor_ldisc_module_exit);