/*
 * test.c - self-contained unit tests for the miniAHRS library.
 *
 * Builds synthetic, checksum-valid frames for every supported packet type
 * (sensors 0x9D, orientation 0x52, quaternion 0x56), feeds them through the
 * streaming parser, and checks the decoded values. Also exercises the
 * quaternion -> Euler conversion and the parser's resynchronisation on
 * leading garbage.
 *
 * Build: cc -std=c99 -Wall -Wextra -O2 -o test test.c miniahrs.c -lm
 * Run:   ./test         (prints PASS/FAIL lines, exit code 0 = all passed)
 */

#include "miniahrs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static int g_fail = 0;

#define CHECK(cond, msg) do {                              \
    if (cond) { printf("  PASS  %s\n", msg); }             \
    else      { printf("  FAIL  %s\n", msg); ++g_fail; }   \
} while (0)

#define CLOSE(a, b, eps) (fabs((double)(a) - (double)(b)) <= (eps))

/* M_PI is not in standard C99; define it locally. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- little-endian writers --------------------------------------------- */

static void w16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void w32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}

/* Wrap a payload into a full frame with a valid checksum.
 * Returns total frame length written to 'out'. */
static size_t build_frame(uint8_t *out, uint8_t id,
                          const uint8_t *payload, size_t plen)
{
    size_t flen = 6 + plen + 2;
    uint16_t csum;
    out[0] = MINIAHRS_HEADER0;
    out[1] = MINIAHRS_HEADER1;
    out[2] = MINIAHRS_MSGTYPE_DATA;
    out[3] = id;
    w16(&out[4], (uint16_t)(plen + 6));
    memcpy(&out[6], payload, plen);
    csum = miniahrs_checksum(out, flen);
    w16(&out[flen - 2], csum);
    return flen;
}

/* ---- captured results from the parser ---------------------------------- */

typedef struct {
    miniahrs_sensors_t     last_sensors;
    miniahrs_orientation_t last_orient;
    miniahrs_quaternion_t  last_quat;
    int n_sensors, n_orient, n_quat;
} capture_t;

static void cb_sensors(const miniahrs_sensors_t *s, void *u)
{ capture_t *c = u; c->last_sensors = *s; c->n_sensors++; }
static void cb_orient(const miniahrs_orientation_t *o, void *u)
{ capture_t *c = u; c->last_orient = *o; c->n_orient++; }
static void cb_quat(const miniahrs_quaternion_t *q, void *u)
{ capture_t *c = u; c->last_quat = *q; c->n_quat++; }

/* ---- tests ------------------------------------------------------------- */

static void test_sensors(void)
{
    uint8_t pl[MINIAHRS_SENSORS_PAYLOAD] = {0};
    uint8_t frame[MINIAHRS_MAX_FRAME];
    size_t flen;
    capture_t cap; memset(&cap, 0, sizeof cap);
    miniahrs_callbacks_t cbs = { cb_sensors, cb_orient, cb_quat, &cap };
    miniahrs_parser_t p;

    /* Gyro=(1.5,-2.5,0.25) deg/s ; Acc=(0,0,1) g ; mag=(10,-20,30) */
    w32(pl + 0,  (uint32_t)(int32_t)(1.5   * MINIAHRS_GYRO_SCALE));
    w32(pl + 4,  (uint32_t)(int32_t)(-2.5  * MINIAHRS_GYRO_SCALE));
    w32(pl + 8,  (uint32_t)(int32_t)(0.25  * MINIAHRS_GYRO_SCALE));
    w32(pl + 12, (uint32_t)(int32_t)(0.0   * MINIAHRS_ACC_SCALE));
    w32(pl + 16, (uint32_t)(int32_t)(0.0   * MINIAHRS_ACC_SCALE));
    w32(pl + 20, (uint32_t)(int32_t)(1.0   * MINIAHRS_ACC_SCALE));
    w16(pl + 24, (uint16_t)(int16_t)10);
    w16(pl + 26, (uint16_t)(int16_t)-20);
    w16(pl + 28, (uint16_t)(int16_t)30);
    w16(pl + 30, 0x000B);
    w32(pl + 32, 123456789u);
    w16(pl + 36, 271);

    flen = build_frame(frame, MINIAHRS_ID_SENSORS, pl, sizeof pl);

    printf("[sensors packet]\n");
    miniahrs_parser_init(&p);
    miniahrs_parser_push_ex(&p, frame, flen, &cbs);

    CHECK(cap.n_sensors == 1, "one sensor packet decoded");
    CHECK(CLOSE(cap.last_sensors.gyro_x,  1.5,  1e-4), "gyro_x = 1.5");
    CHECK(CLOSE(cap.last_sensors.gyro_y, -2.5,  1e-4), "gyro_y = -2.5");
    CHECK(CLOSE(cap.last_sensors.acc_z,   1.0,  1e-4), "acc_z = 1.0");
    CHECK(cap.last_sensors.mag_x == 10 && cap.last_sensors.mag_y == -20
          && cap.last_sensors.mag_z == 30, "mag = (10,-20,30)");
    CHECK(cap.last_sensors.status == 0x000B, "status = 0x000B");
    CHECK(cap.last_sensors.timestamp == 123456789u, "timestamp ok");
}

static void test_orientation(void)
{
    uint8_t pl[38] = {0};
    uint8_t frame[MINIAHRS_MAX_FRAME];
    size_t flen;
    capture_t cap; memset(&cap, 0, sizeof cap);
    miniahrs_callbacks_t cbs = { cb_sensors, cb_orient, cb_quat, &cap };
    miniahrs_parser_t p;

    /* heading=123.45 pitch=-12.30 roll=45.60 */
    w16(pl + 0, (uint16_t)(123.45 * MINIAHRS_ANGLE_SCALE));
    w16(pl + 2, (uint16_t)(int16_t)(-12.30 * MINIAHRS_ANGLE_SCALE));
    w16(pl + 4, (uint16_t)(int16_t)( 45.60 * MINIAHRS_ANGLE_SCALE));

    flen = build_frame(frame, MINIAHRS_ID_OPVT, pl, sizeof pl);

    printf("[orientation packet]\n");
    miniahrs_parser_init(&p);
    miniahrs_parser_push_ex(&p, frame, flen, &cbs);

    CHECK(cap.n_orient == 1, "one orientation packet decoded");
    CHECK(CLOSE(cap.last_orient.heading, 123.45, 1e-2), "heading = 123.45");
    CHECK(CLOSE(cap.last_orient.pitch,  -12.30, 1e-2), "pitch = -12.30");
    CHECK(CLOSE(cap.last_orient.roll,    45.60, 1e-2), "roll = 45.60");
}

static void test_quaternion(void)
{
    uint8_t pl[38] = {0};
    uint8_t frame[MINIAHRS_MAX_FRAME];
    size_t flen;
    capture_t cap; memset(&cap, 0, sizeof cap);
    miniahrs_callbacks_t cbs = { cb_sensors, cb_orient, cb_quat, &cap };
    miniahrs_parser_t p;
    miniahrs_orientation_t e;

    /* Identity quaternion -> heading=0 pitch=0 roll=0 */
    w16(pl + 0, (uint16_t)(1.0 * MINIAHRS_QUAT_SCALE));
    w16(pl + 2, 0);
    w16(pl + 4, 0);
    w16(pl + 6, 0);

    flen = build_frame(frame, MINIAHRS_ID_QPVT, pl, sizeof pl);

    printf("[quaternion packet]\n");
    miniahrs_parser_init(&p);
    miniahrs_parser_push_ex(&p, frame, flen, &cbs);

    CHECK(cap.n_quat == 1, "one quaternion packet decoded");
    CHECK(CLOSE(cap.last_quat.q0, 1.0, 1e-4), "q0 = 1.0");

    miniahrs_quat_to_euler(&cap.last_quat, &e);
    CHECK(CLOSE(e.heading, 0.0, 1e-3) || CLOSE(e.heading, 360.0, 1e-3),
          "identity quat -> heading 0");
    CHECK(CLOSE(e.pitch, 0.0, 1e-3), "identity quat -> pitch 0");
    CHECK(CLOSE(e.roll,  0.0, 1e-3), "identity quat -> roll 0");

    /* 90 deg heading: q = (cos45, 0, 0, -sin45) about Z (ICD sign). */
    {
        double c = cos(45.0 * M_PI / 180.0), s = sin(45.0 * M_PI / 180.0);
        miniahrs_quaternion_t q = { c, 0.0, 0.0, -s };
        miniahrs_quat_to_euler(&q, &e);
        CHECK(CLOSE(e.heading, 90.0, 1e-2), "yaw quat -> heading 90");
        CHECK(CLOSE(e.pitch, 0.0, 1e-2) && CLOSE(e.roll, 0.0, 1e-2),
              "yaw quat -> pitch/roll 0");
    }
}

static void test_gyro_acc_yaw(void)
{
    miniahrs_sensors_t s;
    miniahrs_gyro_acc_state_t st;
    miniahrs_orientation_t o;
    memset(&s, 0, sizeof s);
    memset(&st, 0, sizeof st);

    s.acc_z = 1.0;
    s.gyro_z = 90.0;

    printf("[gyro + accel yaw]\n");
    CHECK(miniahrs_orientation_from_gyro_acc(&s, 0.0, 0.0, &st, &o) == 0,
          "gyro+acc estimator initializes");
    CHECK(CLOSE(o.heading, 0.0, 1e-3), "initial yaw = 0");

    CHECK(miniahrs_orientation_from_gyro_acc(&s, 1.0, 0.0, &st, &o) == 0,
          "gyro+acc estimator integrates yaw");
    CHECK(CLOSE(o.heading, 90.0, 1e-3), "90 deg/s for 1 s -> yaw 90");
    CHECK(CLOSE(o.pitch, 0.0, 1e-3), "level accel -> pitch 0");
    CHECK(CLOSE(o.roll,  0.0, 1e-3), "level accel -> roll 0");
}

static void test_resync_and_garbage(void)
{
    uint8_t pl[MINIAHRS_SENSORS_PAYLOAD] = {0};
    uint8_t frame[MINIAHRS_MAX_FRAME];
    uint8_t stream[MINIAHRS_MAX_FRAME * 2];
    size_t flen, n = 0;
    capture_t cap; memset(&cap, 0, sizeof cap);
    miniahrs_callbacks_t cbs = { cb_sensors, cb_orient, cb_quat, &cap };
    miniahrs_parser_t p;

    w32(pl + 20, (uint32_t)(int32_t)(1.0 * MINIAHRS_ACC_SCALE)); /* acc_z=1 */
    flen = build_frame(frame, MINIAHRS_ID_SENSORS, pl, sizeof pl);

    /* leading junk, then a frame with one corrupted copy, then a good copy */
    stream[n++] = 0x11; stream[n++] = 0xAA; stream[n++] = 0x22; stream[n++] = 0x55;
    memcpy(stream + n, frame, flen); stream[n + 7] ^= 0xFF; /* corrupt payload */
    n += flen;
    memcpy(stream + n, frame, flen); n += flen;             /* good frame */

    printf("[resync / garbage / bad checksum]\n");
    miniahrs_parser_init(&p);
    /* feed one byte at a time to stress the streaming state machine */
    {
        size_t i;
        for (i = 0; i < n; ++i)
            miniahrs_parser_push_ex(&p, &stream[i], 1, &cbs);
    }
    CHECK(cap.n_sensors == 1, "exactly one valid frame survives (bad one rejected)");
    CHECK(CLOSE(cap.last_sensors.acc_z, 1.0, 1e-4), "surviving frame decoded ok");
}

int main(void)
{
    printf("miniAHRS library self-test\n\n");
    test_sensors();
    test_orientation();
    test_quaternion();
    test_gyro_acc_yaw();
    test_resync_and_garbage();
    printf("\n%s (%d failure%s)\n",
           g_fail ? "SOME TESTS FAILED" : "ALL TESTS PASSED",
           g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
