/*
 * miniahrs.c - Inertial Labs miniAHRS parsing library implementation.
 * See miniahrs.h for the protocol description.
 */

#include "miniahrs.h"
#include <string.h>
#include <math.h>

/* ---- Little-endian readers ----------------------------------------------- */

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)rd_u16(p);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)rd_u32(p);
}

/* ---- Checksum ------------------------------------------------------------
 *
 * The protocol checksum is the 16-bit arithmetic sum of every byte from
 * the message-type field (index 2) through the last payload byte, i.e.
 * everything except the two header bytes and the checksum itself.
 */
uint16_t miniahrs_checksum(const uint8_t *frame, size_t frame_len)
{
    uint16_t sum = 0;
    size_t i;
    if (frame_len < 2)
        return 0;
    /* bytes [2 .. frame_len-3] = type, id, len, payload */
    for (i = 2; i + 2 < frame_len; ++i)
        sum = (uint16_t)(sum + frame[i]);
    return sum;
}

/* ---- Single-frame decode ------------------------------------------------- */

miniahrs_status_t miniahrs_decode_sensors(const uint8_t *frame, size_t len,
                                          miniahrs_sensors_t *out)
{
    const uint8_t *pl;
    uint16_t csum_calc, csum_pkt;

    if (len < MINIAHRS_SENSORS_FRAME)
        return MINIAHRS_NEED_MORE;

    if (frame[3] != MINIAHRS_ID_SENSORS)
        return MINIAHRS_UNKNOWN_ID;

    csum_calc = miniahrs_checksum(frame, MINIAHRS_SENSORS_FRAME);
    csum_pkt  = rd_u16(&frame[MINIAHRS_SENSORS_FRAME - 2]);
    if (csum_calc != csum_pkt)
        return MINIAHRS_BAD_CHECKSUM;

    pl = &frame[6]; /* payload start */

    if (out) {
        out->gyro_x = (double)rd_i32(pl + 0)  / MINIAHRS_GYRO_SCALE;
        out->gyro_y = (double)rd_i32(pl + 4)  / MINIAHRS_GYRO_SCALE;
        out->gyro_z = (double)rd_i32(pl + 8)  / MINIAHRS_GYRO_SCALE;

        out->acc_x  = (double)rd_i32(pl + 12) / MINIAHRS_ACC_SCALE;
        out->acc_y  = (double)rd_i32(pl + 16) / MINIAHRS_ACC_SCALE;
        out->acc_z  = (double)rd_i32(pl + 20) / MINIAHRS_ACC_SCALE;

        out->mag_x  = rd_i16(pl + 24);
        out->mag_y  = rd_i16(pl + 26);
        out->mag_z  = rd_i16(pl + 28);

        out->status          = rd_u16(pl + 30);
        out->timestamp       = rd_u32(pl + 32);
        out->temperature_raw = rd_u16(pl + 36);
    }
    return MINIAHRS_OK;
}

/* ---- Generic frame validation -------------------------------------------
 *
 * Confirms header bytes, that 'len' covers the full frame declared by the
 * length field, the expected data id, and the trailing checksum. On success
 * returns MINIAHRS_OK and writes the payload pointer and length via the
 * output parameters pl and pl_len.
 */
static miniahrs_status_t validate_frame(const uint8_t *frame, size_t len,
                                        uint8_t expect_id,
                                        const uint8_t **pl, size_t *pl_len)
{
    uint16_t msg_len, csum_calc, csum_pkt;
    size_t frame_len;

    if (len < 6)
        return MINIAHRS_NEED_MORE;
    if (frame[0] != MINIAHRS_HEADER0 || frame[1] != MINIAHRS_HEADER1)
        return MINIAHRS_UNKNOWN_ID;

    msg_len   = rd_u16(&frame[4]);   /* payload + 6           */
    frame_len = (size_t)msg_len + 2; /* + the two header bytes */
    if (frame_len < 8 || frame_len > MINIAHRS_MAX_FRAME)
        return MINIAHRS_UNKNOWN_ID;
    if (len < frame_len)
        return MINIAHRS_NEED_MORE;

    if (frame[3] != expect_id)
        return MINIAHRS_UNKNOWN_ID;

    csum_calc = miniahrs_checksum(frame, frame_len);
    csum_pkt  = rd_u16(&frame[frame_len - 2]);
    if (csum_calc != csum_pkt)
        return MINIAHRS_BAD_CHECKSUM;

    if (pl)     *pl     = &frame[6];
    /* payload bytes = frame minus header(2)+type(1)+id(1)+len(2)+csum(2) = 8 */
    if (pl_len) *pl_len = frame_len - 8;
    return MINIAHRS_OK;
}

/* ---- Orientation / quaternion decode ------------------------------------- */

miniahrs_status_t miniahrs_decode_orientation(const uint8_t *frame, size_t len,
                                              miniahrs_orientation_t *out)
{
    const uint8_t *pl;
    size_t pl_len;
    miniahrs_status_t st = validate_frame(frame, len, MINIAHRS_ID_OPVT, &pl, &pl_len);
    if (st != MINIAHRS_OK)
        return st;
    if (pl_len < 6)
        return MINIAHRS_UNKNOWN_ID; /* payload too short for H/P/R */

    if (out) {
        out->heading = (double)rd_u16(pl + 0) / MINIAHRS_ANGLE_SCALE; /* word  */
        out->pitch   = (double)rd_i16(pl + 2) / MINIAHRS_ANGLE_SCALE; /* sword */
        out->roll    = (double)rd_i16(pl + 4) / MINIAHRS_ANGLE_SCALE; /* sword */
    }
    return MINIAHRS_OK;
}

miniahrs_status_t miniahrs_decode_quaternion(const uint8_t *frame, size_t len,
                                             miniahrs_quaternion_t *out)
{
    const uint8_t *pl;
    size_t pl_len;
    miniahrs_status_t st = validate_frame(frame, len, MINIAHRS_ID_QPVT, &pl, &pl_len);
    if (st != MINIAHRS_OK)
        return st;
    if (pl_len < 8)
        return MINIAHRS_UNKNOWN_ID; /* payload too short for Lk0..Lk3 */

    if (out) {
        out->q0 = (double)rd_i16(pl + 0) / MINIAHRS_QUAT_SCALE;
        out->q1 = (double)rd_i16(pl + 2) / MINIAHRS_QUAT_SCALE;
        out->q2 = (double)rd_i16(pl + 4) / MINIAHRS_QUAT_SCALE;
        out->q3 = (double)rd_i16(pl + 6) / MINIAHRS_QUAT_SCALE;
    }
    return MINIAHRS_OK;
}

/* Quaternion -> Euler angles, following MRU ICD Appendix D (eq. D.7):
 *   K (heading) =  arctan( 2(q1q2 - q0q3) / (q0^2 + q1^2 - q2^2 - q3^2) )
 *   theta(pitch)=  arcsin( 2(q2q3 + q0q1) )
 *   gamma(roll) = -arctan( 2(q1q3 - q0q2) / (q0^2 + q3^2 - q1^2 - q2^2) )
 * Output angles are in degrees: heading 0..360, pitch +-90, roll +-180. */
void miniahrs_quat_to_euler(const miniahrs_quaternion_t *q,
                            miniahrs_orientation_t *out)
{
    const double RAD2DEG = 57.29577951308232;
    double q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    double sp = 2.0 * (q2 * q3 + q0 * q1); /* arcsin argument for pitch */
    double heading, pitch, roll;

    heading = atan2(2.0 * (q1 * q2 - q0 * q3),
                    q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD2DEG;
    if (heading < 0.0)
        heading += 360.0;

    if (sp >  1.0) sp =  1.0;   /* clamp for numerical safety */
    if (sp < -1.0) sp = -1.0;
    pitch = asin(sp) * RAD2DEG;

    roll = -atan2(2.0 * (q1 * q3 - q0 * q2),
                  q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2) * RAD2DEG;

    out->heading = heading;
    out->pitch   = pitch;
    out->roll    = roll;
}

/* ---- Orientation from raw accel + mag (tilt-compensated AHRS) ------------
 *
 * Device body frame (ICD Fig. 1.3): X lateral (right), Y longitudinal
 * (forward), Z vertical (up). At rest the accelerometers sense +1 g on Z.
 *
 *   roll  (about Y) = atan2( ax , az )
 *   pitch (about X) = atan2( -ay , sqrt(ax^2 + az^2) )
 *   heading: de-rotate the magnetic vector by roll then pitch, then
 *            yaw = atan2( -my', mx' ) measured clockwise from X.
 *
 * The heading convention is chosen so that 0..360 increases clockwise,
 * matching the Inertial Labs heading definition.
 */
int miniahrs_orientation_from_sensors(const miniahrs_sensors_t *s,
                                      double declination_deg,
                                      miniahrs_orientation_t *out)
{
    const double RAD2DEG = 57.29577951308232;
    double ax = s->acc_x, ay = s->acc_y, az = s->acc_z;
    double mx = (double)s->mag_x, my = (double)s->mag_y, mz = (double)s->mag_z;
    double anorm = sqrt(ax * ax + ay * ay + az * az);
    double roll, pitch, sr, cr, sp, cp, mx2, my2, heading;

    if (anorm < 1e-6)
        return 1; /* no usable gravity vector */

    roll  = atan2(ax, az);
    pitch = atan2(-ay, sqrt(ax * ax + az * az));

    sr = sin(roll);  cr = cos(roll);
    sp = sin(pitch); cp = cos(pitch);

    /* Tilt-compensate the magnetic vector into the horizontal plane. */
    mx2 = mx * cp + my * sr * sp + mz * cr * sp;
    my2 = my * cr - mz * sr;

    
    while (heading <   0.0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;

    out->heading = heading;
    out->pitch   = pitch * RAD2DEG;
    out->roll    = roll  * RAD2DEG;
    return 0;
}

static double wrap_360(double deg)
{
    while (deg <   0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

int miniahrs_orientation_from_gyro_acc(const miniahrs_sensors_t *s,
                                       double dt_sec,
                                       double initial_yaw_deg,
                                       miniahrs_gyro_acc_state_t *state,
                                       miniahrs_orientation_t *out)
{
    const double RAD2DEG = 57.29577951308232;
    double ax = s->acc_x, ay = s->acc_y, az = s->acc_z;
    double anorm = sqrt(ax * ax + ay * ay + az * az);
    double roll, pitch;

    if (anorm < 1e-6 || dt_sec < 0.0 || !state || !out)
        return 1;

    if (!state->initialized) {
        state->yaw = wrap_360(initial_yaw_deg);
        state->initialized = 1;
    } else {
        state->yaw = wrap_360(state->yaw + s->gyro_z * dt_sec);
    }

    roll  = atan2(ax, az);
    pitch = atan2(-ay, sqrt(ax * ax + az * az));

    out->heading = state->yaw;
    out->pitch   = pitch * RAD2DEG;
    out->roll    = roll  * RAD2DEG;
    return 0;
}

/* ---- Streaming parser ---------------------------------------------------- */

void miniahrs_parser_init(miniahrs_parser_t *p)
{
    p->len = 0;
    p->last_id = 0;
}

/* Drop n bytes from the front of the buffer. */
static void parser_consume(miniahrs_parser_t *p, size_t n)
{
    if (n >= p->len) {
        p->len = 0;
        return;
    }
    memmove(p->buf, p->buf + n, p->len - n);
    p->len -= n;
}

/* Slide the buffer to the next plausible 0xAA 0x55 header. Returns 1 if a
 * header prefix is present at index 0 afterwards, 0 if the buffer drained. */
static int parser_resync(miniahrs_parser_t *p)
{
    size_t i;
    for (i = 0; i + 1 < p->len; ++i) {
        if (p->buf[i] == MINIAHRS_HEADER0 && p->buf[i + 1] == MINIAHRS_HEADER1) {
            if (i)
                parser_consume(p, i);
            return 1;
        }
    }
    /* No full header found. Keep at most the trailing 0xAA so a header
     * split across reads can still be recognised. */
    if (p->len && p->buf[p->len - 1] == MINIAHRS_HEADER0) {
        p->buf[0] = MINIAHRS_HEADER0;
        p->len = 1;
    } else {
        p->len = 0;
    }
    return 0;
}

/* Dispatch one validated frame held at p->buf to the relevant callback.
 * Returns 1 if a packet was recognised and reported, 0 otherwise. */
static size_t parser_dispatch(miniahrs_parser_t *p, size_t frame_len,
                              const miniahrs_callbacks_t *cbs)
{
    uint8_t id = p->buf[3];
    p->last_id = id;

    switch (id) {
    case MINIAHRS_ID_SENSORS:
        if (frame_len == MINIAHRS_SENSORS_FRAME) {
            miniahrs_sensors_t s;
            if (miniahrs_decode_sensors(p->buf, frame_len, &s) == MINIAHRS_OK) {
                if (cbs->on_sensors)
                    cbs->on_sensors(&s, cbs->user);
                return 1;
            }
        }
        break;
    case MINIAHRS_ID_OPVT: {
        miniahrs_orientation_t o;
        if (miniahrs_decode_orientation(p->buf, frame_len, &o) == MINIAHRS_OK) {
            if (cbs->on_orientation)
                cbs->on_orientation(&o, cbs->user);
            return 1;
        }
        break;
    }
    case MINIAHRS_ID_QPVT: {
        miniahrs_quaternion_t q;
        if (miniahrs_decode_quaternion(p->buf, frame_len, &q) == MINIAHRS_OK) {
            if (cbs->on_quaternion)
                cbs->on_quaternion(&q, cbs->user);
            return 1;
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

size_t miniahrs_parser_push_ex(miniahrs_parser_t *p,
                               const uint8_t *data, size_t n,
                               const miniahrs_callbacks_t *cbs)
{
    miniahrs_callbacks_t none = {0, 0, 0, 0};
    size_t produced = 0;
    size_t i;

    if (!cbs)
        cbs = &none;

    for (i = 0; i < n; ++i) {
        /* Guard against a runaway buffer (should never fill in practice). */
        if (p->len >= MINIAHRS_MAX_FRAME)
            parser_consume(p, p->len - 1);
        p->buf[p->len++] = data[i];

        for (;;) {
            /* Need at least the framing header + length field. */
            if (p->len < 2)
                break;
            if (p->buf[0] != MINIAHRS_HEADER0 || p->buf[1] != MINIAHRS_HEADER1) {
                if (!parser_resync(p))
                    break;
                continue;
            }
            if (p->len < 6)
                break; /* need length field */

            {
                uint16_t msg_len = rd_u16(&p->buf[4]); /* payload + 6 */
                size_t frame_len = (size_t)msg_len + 2; /* + 2 header bytes */

                if (frame_len < 8 || frame_len > MINIAHRS_MAX_FRAME) {
                    /* Implausible length: bad header, skip it and resync. */
                    parser_consume(p, 2);
                    continue;
                }
                if (p->len < frame_len)
                    break; /* wait for the rest of the frame */

                {
                    uint16_t cc = miniahrs_checksum(p->buf, frame_len);
                    uint16_t cp = rd_u16(&p->buf[frame_len - 2]);
                    if (cc != cp) {
                        /* Corrupt frame: discard header, hunt for next. */
                        parser_consume(p, 2);
                        continue;
                    }
                }

                produced += parser_dispatch(p, frame_len, cbs);
                /* Consume the whole validated frame and continue. */
                parser_consume(p, frame_len);
                continue;
            }
        }
    }
    return produced;
}

size_t miniahrs_parser_push(miniahrs_parser_t *p,
                            const uint8_t *data, size_t n,
                            miniahrs_sensors_cb cb, void *user)
{
    miniahrs_callbacks_t cbs = {cb, 0, 0, user};
    return miniahrs_parser_push_ex(p, data, n, &cbs);
}
