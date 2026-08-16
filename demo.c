/*
 * demo.c - miniAHRS library demonstration.
 *
 * Reads a capture file containing the raw miniAHRS byte stream and prints
 * every decoded sensor packet. The capture may be either raw binary or an
 * ASCII hex dump (as produced by the GUI logger, e.g. output_*.log) -- the
 * loader auto-detects and strips non-hex characters.
 *
 * Build (see Makefile):   cc -o demo demo.c miniahrs.c
 * Run:                    ./demo output_2026-06-29_10-48-12.log
 */

#include "miniahrs.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Load a capture file into a freshly malloc'd byte buffer.
 * Detects ASCII-hex captures and decodes them to raw bytes. */
static uint8_t *load_capture(const char *path, size_t *out_len)
{
    FILE *f = fopen(path, "rb");
    long fsz;
    uint8_t *raw;
    size_t i, n;
    int looks_hex = 1;

    if (!f) {
        perror(path);
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsz <= 0) {
        fclose(f);
        return NULL;
    }
    raw = (uint8_t *)malloc((size_t)fsz);
    if (!raw) {
        fclose(f);
        return NULL;
    }
    n = fread(raw, 1, (size_t)fsz, f);
    fclose(f);

    /* Heuristic: if the content is entirely hex digits / whitespace,
     * treat it as an ASCII hex dump. */
    for (i = 0; i < n; ++i) {
        int c = raw[i];
        if (!isspace(c) && hexval(c) < 0) {
            looks_hex = 0;
            break;
        }
    }

    if (looks_hex) {
        uint8_t *dec = (uint8_t *)malloc(n / 2 + 1);
        size_t dn = 0;
        int hi = -1;
        if (!dec) {
            free(raw);
            return NULL;
        }
        for (i = 0; i < n; ++i) {
            int v = hexval(raw[i]);
            if (v < 0)
                continue; /* skip whitespace/separators */
            if (hi < 0) {
                hi = v;
            } else {
                dec[dn++] = (uint8_t)((hi << 4) | v);
                hi = -1;
            }
        }
        free(raw);
        *out_len = dn;
        return dec;
    }

    *out_len = n;
    return raw;
}

static void on_sensors(const miniahrs_sensors_t *s, void *user)
{
    size_t *count = (size_t *)user;
    miniahrs_orientation_t att;
    ++(*count);
    miniahrs_orientation_from_sensors(s, 0.0, &att); /* 0 = magnetic heading */
    printf("[%6lu] yaw=%7.2f pitch=%+7.2f roll=%+7.2f deg | "
           "gyro(%+7.3f %+7.3f %+7.3f) acc(%+6.3f %+6.3f %+6.3f) "
           "mag(%5d %5d %5d) T=%u\n",
           (unsigned long)*count,
           att.heading, att.pitch, att.roll,
           s->gyro_x, s->gyro_y, s->gyro_z,
           s->acc_x,  s->acc_y,  s->acc_z,
           s->mag_x,  s->mag_y,  s->mag_z,
           s->temperature_raw);
}

static void on_orientation(const miniahrs_orientation_t *o, void *user)
{
    (void)user;
    printf("        ORIENTATION  heading=%7.2f  pitch=%+7.2f  roll=%+7.2f  deg\n",
           o->heading, o->pitch, o->roll);
}

static void on_quaternion(const miniahrs_quaternion_t *q, void *user)
{
    miniahrs_orientation_t e;
    (void)user;
    miniahrs_quat_to_euler(q, &e);
    printf("        QUATERNION   (%+.4f %+.4f %+.4f %+.4f)"
           "  -> heading=%7.2f pitch=%+7.2f roll=%+7.2f\n",
           q->q0, q->q1, q->q2, q->q3, e.heading, e.pitch, e.roll);
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "output_2026-06-29_10-48-12.log";
    size_t len = 0;
    uint8_t *data = load_capture(path, &len);
    miniahrs_parser_t parser;
    size_t count = 0, decoded;

    if (!data) {
        fprintf(stderr, "failed to load capture: %s\n", path);
        return 1;
    }
    printf("loaded %lu bytes from %s\n\n", (unsigned long)len, path);

    {
        miniahrs_callbacks_t cbs;
        cbs.on_sensors     = on_sensors;
        cbs.on_orientation = on_orientation;   /* OPVT packets (id 0x52) */
        cbs.on_quaternion  = on_quaternion;    /* QPVT packets (id 0x56) */
        cbs.user           = &count;

        miniahrs_parser_init(&parser);
        /* Feed the whole buffer; a real app would call push() per serial read. */
        decoded = miniahrs_parser_push_ex(&parser, data, len, &cbs);
    }

    printf("\ndecoded %lu miniAHRS packets total\n", (unsigned long)decoded);
    free(data);
    return 0;
}
