/* 
 * sensor_read.c -- Userspace client for /dev/sensor_data
 * Demonstrates all three interaction modes with the kernel driver:
 *  1. Blocking_read()      - waits for next frame, then prints
 *  2. Non-blocking ioctl() - snapshot of latest reading
 *  3. poll()               - event-driven; triggers on new data
 *  
 * Compile on the Pi:
 *  gcc -Wall -I/home/sailesh/sensor_ldisc/include sensor_read.c -o sensor_read
 *  
 *  Usage:
 *      ./sensor_read               # blocking read loop (default)
 *      ./sensor_read --ioctl       # ioctl snapshot
 *      ./sensor_read --poll        # poll-based event loop
 *      ./sensor_read --setproto 1  # switch kernel to binary protocol
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#include "sensor_ioctl.h"

#define DEVICE "/dev/sensor_data"

static volatile int running = 1;

static void sigint_handler(int sig) { (void)sig; running = 0; }

/* --- 1. Blocking read loop --- */
static void mode_read(int fd)
{
    char buf[128];
    ssize_t n;

    printf("[read] Blocking on %s - Ctrl+C to stop\n\n", DEVICE);

    while(running) {
        n = read(fd, buf, sizeof(buf) - 1);
        if (n < 0) {
            if (errno == EINTR) break;
            if (errno == ENODEV) { fprintf(stderr, "ldisc detached\n"); break; }
            perror("read"); break;
        }
        if (n == 0) continue;

        buf[n] = '\0';
        /* Strip trailing newline for clean printf */
        if (n > 0 && buf[n - 1] == '\n') buf[n - 1] = '\0';

        printf("%s\n", buf);
        fflush(stdout);
    }
}

/* --- 2. ioctl snapshot --- */
static void mode_ioctl(int fd)
{
    struct sensor_reading r;

    if(ioctl(fd, SENSOR_IOC_GETREADING, &r) < 0) {
        perror("SENSOR_IOC_GETREADING");
        return;
    }

    int t_abs   = (r.temp_x100 < 0) ? -r.temp_x100 : r.temp_x100;
    int t_sign  = (r.temp_x100 < 0) ? -1 : 1;
    
    printf("[ioctl snapshot]\n");
    printf("    Temperature    : %c%d.%02d  °C\n",
            t_sign < 0 ? '-' : '+', t_abs / 100, t_abs % 100);
    printf("    Distance       : %u.%u cm\n", r.dist_x10 / 10, r.dist_x10 % 10);
    printf("    Sequence       : %u\n", r.seq);
    printf("    Timestamp      : %lu ms (kernel monotonic)\n", r.timestamp_ms);
}

/* --- 3. poll-based event loop --- */
static void mode_poll(int fd)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    char buf[128];
    ssize_t n;
    int ret;

    printf("[poll] Waiting for events on %s -- Ctrl+C to stop\n\n", DEVICE);

    while (running) {
        ret = poll(&pfd, 1, 1000);      /* 1s timeout for Ctrl+C responsiveness */

        if (ret < 0) {
            if (errno == EINTR) break;
            perror("poll"); break;
        }
        if (ret == 0) continue;     /* timeout -- loop back */

        if (pfd.revents & POLLIN) {
            n = read(fd, buf, sizeof(buf) - 1);
            if (n <= 0) break;
            buf[n] = '\0';
            if (n > 0 && buf[n -1] == '\n') buf[n -1] = '\0';
            printf("[POLLIN] %s\n", buf);
            fflush(stdout);
        }

        if (pfd.revents & POLLERR) {
            fprintf(stderr, "POLLERR -- ldisc may have detached\n");
            break;
        }
    }
}

/* -- 4. Protocol switch -- */
static void mode_setproto(int fd, int p)
{
    const char *names[] = {"ASCII", "binary", "JSON"};

    if (ioctl(fd, SENSOR_IOC_SETPROTO, p) < 0) {
        perror("SENSOR_IOC_SETPROTO");
        return;
    }
    printf("Protocol switched to %d (%s)\n", p,
            (p >= 0 && p <= 2) ? names[p] : "?");
}

/* --- Entry point --- */
int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);

    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", DEVICE, strerror(errno));
        fprintf(stderr, " IS sensor_ldisc.ko loaded and ldisc attached ?\n");
        return 1;
    }

    if (argc > 1) {
        if (strcmp(argv[1], "--ioctl") == 0) {
            mode_ioctl(fd);
        } else if (strcmp(argv[1], "--poll") == 0) {
            mode_poll(fd);
        } else if (strcmp(argv[1], "--setproto") == 0 && argc > 2) {
            mode_setproto(fd, atoi(argv[2]));
        } else {
            fprintf(stderr, "Usage: %s [--ioctl | --poll | --setproto 0|1|2]\n",
                        argv[0]);
            close(fd); return 1;
        }    
    } else {
        mode_read(fd);      /* default */
    }

    close(fd);
    return 0;
}