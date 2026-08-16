/*
 * readserial.c - read the miniAHRS live stream from a serial (COM) port
 *                on Windows and print every decoded packet.
 *
 * The miniAHRS uses an RS-422 interface; connect it through an RS-422-to-USB
 * adapter, which appears as a virtual COM port (e.g. COM3). Default line
 * settings follow the Inertial Labs family: 115200 baud, 8 data bits,
 * no parity, 1 stop bit.
 *
 * Build (MSYS2 / MinGW):
 *     gcc -std=c99 -Wall -O2 -o readserial readserial.c miniahrs.c -lm
 *
 * Run:
 *     ./readserial COM3            (default 115200 baud)
 *     ./readserial COM3 460800     (custom baud)
 *
 * Press Ctrl+C to stop.
 */

#include "miniahrs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>

/* Open and configure a COM port. Returns INVALID_HANDLE_VALUE on failure. */
static HANDLE open_port(const char *name, int baud)
{
    char path[64];
    HANDLE h;
    DCB dcb;
    COMMTIMEOUTS to;

    /* "\\.\COMx" form is required for COM10 and above. */
    snprintf(path, sizeof path, "\\\\.\\%s", name);

    h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                    OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "cannot open %s (error %lu)\n",
                name, (unsigned long)GetLastError());
        return h;
    }

    memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = sizeof dcb;
    if (!GetCommState(h, &dcb)) {
        fprintf(stderr, "GetCommState failed\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_ENABLE;
    dcb.fRtsControl  = RTS_CONTROL_ENABLE;
    if (!SetCommState(h, &dcb)) {
        fprintf(stderr, "SetCommState failed (bad baud rate?)\n");
        CloseHandle(h);
        return INVALID_HANDLE_VALUE;
    }

    /* Return as soon as any bytes are available; small read timeout. */
    memset(&to, 0, sizeof to);
    to.ReadIntervalTimeout         = 50;
    to.ReadTotalTimeoutConstant    = 100;
    to.ReadTotalTimeoutMultiplier  = 0;
    SetCommTimeouts(h, &to);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

static volatile int g_run = 1;
static BOOL WINAPI on_ctrl_c(DWORD type)
{
    (void)type;
    g_run = 0;
    return TRUE;
}

/* Initial yaw (degrees) assumed at the first sample, since gyro+accel alone
 * cannot observe absolute heading. */
#define INITIAL_YAW_DEG  0.0

typedef struct {
    unsigned long count;
    miniahrs_gyro_acc_state_t state;
    LARGE_INTEGER freq;
    LARGE_INTEGER last_tick;
    int have_last_tick;
} sensors_ctx_t;

static void on_sensors(const miniahrs_sensors_t *s, void *user)
{
    sensors_ctx_t *ctx = (sensors_ctx_t *)user;
    miniahrs_orientation_t att;
    LARGE_INTEGER now;
    double dt_sec;
    ++ctx->count;

    QueryPerformanceCounter(&now);
    if (!ctx->have_last_tick) {
        dt_sec = 0.0;
        ctx->have_last_tick = 1;
    } else {
        dt_sec = (double)(now.QuadPart - ctx->last_tick.QuadPart) / (double)ctx->freq.QuadPart;
    }
    ctx->last_tick = now;

    /* Compute Pitch/Roll from accel and integrate Yaw from gyro_z only
     * (no magnetometer). Yaw is relative to INITIAL_YAW_DEG and will drift. */
    miniahrs_orientation_from_gyro_acc(s, dt_sec, INITIAL_YAW_DEG, &ctx->state, &att);

    printf("[%6lu] yaw=%7.2f pitch=%+7.2f roll=%+7.2f deg | "
           "gyro(%+7.3f %+7.3f %+7.3f) acc(%+6.3f %+6.3f %+6.3f) "
           "mag(%5d %5d %5d) T=%u\n",
           ctx->count,
           att.heading, att.pitch, att.roll,
           s->gyro_x, s->gyro_y, s->gyro_z,
           s->acc_x,  s->acc_y,  s->acc_z,
           s->mag_x,  s->mag_y,  s->mag_z,
           s->temperature_raw);
    fflush(stdout);
}

static void on_orientation(const miniahrs_orientation_t *o, void *user)
{
    (void)user;
    printf("        ORIENTATION  heading=%7.2f  pitch=%+7.2f  roll=%+7.2f deg\n",
           o->heading, o->pitch, o->roll);
    fflush(stdout);
}

static void on_quaternion(const miniahrs_quaternion_t *q, void *user)
{
    miniahrs_orientation_t e;
    (void)user;
    miniahrs_quat_to_euler(q, &e);
    printf("        QUATERNION   (%+.4f %+.4f %+.4f %+.4f)"
           " -> heading=%7.2f pitch=%+7.2f roll=%+7.2f\n",
           q->q0, q->q1, q->q2, q->q3, e.heading, e.pitch, e.roll);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "COM3";
    int baud = (argc > 2) ? atoi(argv[2]) : 115200;
    HANDLE h;
    miniahrs_parser_t parser;
    miniahrs_callbacks_t cbs;
    sensors_ctx_t ctx;
    uint8_t buf[512];

    printf("opening %s at %d baud (Ctrl+C to stop)...\n", port, baud);

    h = open_port(port, baud);
    if (h == INVALID_HANDLE_VALUE)
        return 1;

    SetConsoleCtrlHandler(on_ctrl_c, TRUE);

    memset(&ctx, 0, sizeof ctx);
    QueryPerformanceFrequency(&ctx.freq);

    cbs.on_sensors     = on_sensors;
    cbs.on_orientation = on_orientation;
    cbs.on_quaternion  = on_quaternion;
    cbs.user           = &ctx;
    miniahrs_parser_init(&parser);

    while (g_run) {
        DWORD got = 0;
        if (!ReadFile(h, buf, sizeof buf, &got, NULL)) {
            fprintf(stderr, "read error %lu\n", (unsigned long)GetLastError());
            break;
        }
        if (got > 0)
            miniahrs_parser_push_ex(&parser, buf, (size_t)got, &cbs);
        /* got == 0 just means a read timeout elapsed with no data; keep going */
    }

    CloseHandle(h);
    printf("\nstopped. %lu packets decoded.\n", ctx.count);
    return 0;
}

#else  /* non-Windows: POSIX termios */

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>

static volatile sig_atomic_t g_run = 1;
static void on_sig(int s) { (void)s; g_run = 0; }

static int open_port(const char *name, int baud)
{
    int fd = open(name, O_RDONLY | O_NOCTTY);
    struct termios t;
    speed_t spd;
    if (fd < 0) { perror(name); return -1; }
    if (tcgetattr(fd, &t) != 0) { perror("tcgetattr"); close(fd); return -1; }
    cfmakeraw(&t);
    switch (baud) {
        case 9600:   spd = B9600;   break;
        case 19200:  spd = B19200;  break;
        case 38400:  spd = B38400;  break;
        case 57600:  spd = B57600;  break;
        case 230400: spd = B230400; break;
        default:     spd = B115200; break;
    }
    cfsetispeed(&t, spd);
    cfsetospeed(&t, spd);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &t);
    return fd;
}

typedef struct {
    unsigned long count;
    miniahrs_gyro_acc_state_t state;
    struct timespec last_tick;
    int have_last_tick;
} sensors_ctx_t;

static void on_sensors(const miniahrs_sensors_t *s, void *user)
{
    sensors_ctx_t *ctx = user;
    miniahrs_orientation_t att;
    struct timespec now;
    double dt_sec;
    ++ctx->count;

    clock_gettime(CLOCK_MONOTONIC, &now);
    if (!ctx->have_last_tick) {
        dt_sec = 0.0;
        ctx->have_last_tick = 1;
    } else {
        dt_sec = (double)(now.tv_sec - ctx->last_tick.tv_sec)
               + (double)(now.tv_nsec - ctx->last_tick.tv_nsec) / 1e9;
    }
    ctx->last_tick = now;

    /* Compute Pitch/Roll from accel and integrate Yaw from gyro_z only
     * (no magnetometer). Yaw is relative to INITIAL_YAW_DEG and will drift. */
    miniahrs_orientation_from_gyro_acc(s, dt_sec, INITIAL_YAW_DEG, &ctx->state, &att);

    printf("[%6lu] yaw=%7.2f pitch=%+7.2f roll=%+7.2f t=%-10u "
           "gyro(%+8.3f %+8.3f %+8.3f) acc(%+7.4f %+7.4f %+7.4f) status=0x%04X\n",
           ctx->count, att.heading, att.pitch, att.roll, s->timestamp,
           s->gyro_x, s->gyro_y, s->gyro_z,
           s->acc_x, s->acc_y, s->acc_z, s->status);
    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    int baud = (argc > 2) ? atoi(argv[2]) : 115200;
    int fd = open_port(port, baud);
    miniahrs_parser_t parser;
    miniahrs_callbacks_t cbs = {0};
    sensors_ctx_t ctx;
    uint8_t buf[512];
    ssize_t got;

    if (fd < 0) return 1;
    signal(SIGINT, on_sig);
    memset(&ctx, 0, sizeof ctx);
    cbs.on_sensors = on_sensors; cbs.user = &ctx;
    miniahrs_parser_init(&parser);

    printf("reading %s at %d baud (Ctrl+C to stop)...\n", port, baud);
    while (g_run) {
        got = read(fd, buf, sizeof buf);
        if (got > 0) miniahrs_parser_push_ex(&parser, buf, (size_t)got, &cbs);
    }
    close(fd);
    printf("\nstopped. %lu packets decoded.\n", ctx.count);
    return 0;
}

#endif
