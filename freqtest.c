/* Throwaway: measure the miniAHRS sensor packet rate (Hz) over ~3 seconds. */
#include "miniahrs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

static HANDLE open_port(const char *name, int baud)
{
    char path[64];
    HANDLE h;
    DCB dcb;
    COMMTIMEOUTS to;
    snprintf(path, sizeof path, "\\\\.\\%s", name);
    h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "cannot open %s (error %lu)\n", name, (unsigned long)GetLastError());
        return h;
    }
    memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = sizeof dcb;
    GetCommState(h, &dcb);
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    SetCommState(h, &dcb);
    memset(&to, 0, sizeof to);
    to.ReadIntervalTimeout = 50;
    to.ReadTotalTimeoutConstant = 100;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

static void on_sensors(const miniahrs_sensors_t *s, void *user)
{
    unsigned long *count = user;
    (void)s;
    ++(*count);
}

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "COM3";
    int baud = (argc > 2) ? atoi(argv[2]) : 921600;
    double duration_sec = (argc > 3) ? atof(argv[3]) : 3.0;
    HANDLE h;
    miniahrs_parser_t parser;
    miniahrs_callbacks_t cbs = {0};
    unsigned long count = 0;
    uint8_t buf[512];
    LARGE_INTEGER freq, t0, tnow;
    double elapsed;

    h = open_port(port, baud);
    if (h == INVALID_HANDLE_VALUE) return 1;

    cbs.on_sensors = on_sensors;
    cbs.user = &count;
    miniahrs_parser_init(&parser);

    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t0);

    printf("measuring packet rate on %s at %d baud for %.1f s...\n", port, baud, duration_sec);

    do {
        DWORD got = 0;
        if (!ReadFile(h, buf, sizeof buf, &got, NULL)) break;
        if (got > 0) miniahrs_parser_push_ex(&parser, buf, (size_t)got, &cbs);
        QueryPerformanceCounter(&tnow);
        elapsed = (double)(tnow.QuadPart - t0.QuadPart) / (double)freq.QuadPart;
    } while (elapsed < duration_sec);

    CloseHandle(h);
    printf("packets=%lu elapsed=%.3fs -> %.2f Hz\n", count, elapsed, (double)count / elapsed);
    return 0;
}
