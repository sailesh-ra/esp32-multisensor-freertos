/*
 * sensor_ioctl.h -- Shared definitions for /dev/sensor_data ioctls.
 * 
 * Include in both kernel driver (sensor_ldisc.c) and any userspace
 * program that wants to use SENSOR_IOC_SETPROTO or SENSOR_IOC_GETREADING.
 * 
 * From userspace, compile with:
 *  gcc -I../include sensor_read.c -o sensor_read
 */

#ifndef _SENSOR_IOCTL_H
#define _SENSOR_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

/* -- Wire-compatible sensor reading struct --------*/
struct sensor_reading {
    int32_t     temp_x100;      /* temperature x 100, signed  (+2350 = +23.50 °C) */
    uint32_t    dist_x10;       /* distance x 10, unsigned    ( 1453 =  145.3 cm) */
    uint32_t    seq;            /* sequence number from ESP32 frame               */
    uint64_t    timestamp_ms;   /* kernel timestamp at receive time (ktime_get ms)*/
};

/* ---- ioctl magic and commands ----*/
#define SENSOR_IOC_MAGIC            'S'

/*
 *  SENSOR_IOC_SETPROTO     -- switch the active parse protocol at runtime
 *      arg: int 0 = ASCII, 1 = binary, 2 = JSON
 *      returns 0 on success, -EINVAL if arg > 2
 */
#define SENSOR_IOC_SETPROTO             _IOW(SENSOR_IOC_MAGIC, 0, int)

/*
 *  SENSOR_IOC_GETREADING   -- non-blocking snapshot of latest reading 
 *     arg: struct sensor_reading * (userspace pointer)
 *     returns 0 on success, -ENODEV if no ldisc attached
 *     Note: does Not clear data_ready; use read() for that.
 */
#define SENSOR_IOC_GETREADING      _IOR(SENSOR_IOC_MAGIC, 1, struct sensor_reading)

/* Protocol ID constants (mirrors PROTO_* in uart_bridge.h) */
#define SENSOR_PROTO_ASCII     0
#define SENSOR_PROTO_BINARY    1
#define SENSOR_PROTO_JSON      2

#endif  /* _SENSOR_IOCTL_H */