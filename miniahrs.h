/*
 * miniahrs.h - Inertial Labs miniAHRS parsing library
 *
 * Decodes the binary serial stream emitted by the Inertial Labs
 * Miniature Attitude & Heading Reference System (miniAHRS) over its
 * RS-422 interface.
 *
 * Protocol reference: Inertial Labs MRU ICD rev 1.10, Table 6.2
 * (shared byte framing across Inertial Labs devices) plus the
 * miniAHRS sensor-data payload (data identifier 0x9D) reverse
 * engineered from a captured stream.
 *
 * Wire framing (all multi-byte integers are little-endian, low byte first):
 *
 *   byte 0      0xAA            header 0
 *   byte 1      0x55            header 1
 *   byte 2      msg type        0 = command, 1 = data
 *   byte 3      data identifier (0x9D for the miniAHRS sensor packet)
 *   byte 4..5   message length  = payload length + 6
 *   byte 6..N   payload
 *   byte N+1..  16-bit checksum = arithmetic sum of bytes 2..N (low byte first)
 *
 * The 0x9D payload (38 bytes) carries calibrated raw inertial data:
 *
 *   off  0  int32  Gyro X      deg/s  * 1e5
 *   off  4  int32  Gyro Y      deg/s  * 1e5
 *   off  8  int32  Gyro Z      deg/s  * 1e5
 *   off 12  int32  Acc  X      g      * 1e6
 *   off 16  int32  Acc  Y      g      * 1e6
 *   off 20  int32  Acc  Z      g      * 1e6
 *   off 24  int16  Mag  X      (raw / sensor units)
 *   off 26  int16  Mag  Y
 *   off 28  int16  Mag  Z
 *   off 30  uint16 status / unit status word
 *   off 32  uint32 timestamp   sample counter / time tag
 *   off 36  uint16 temperature (raw)
 *
 * This library is dependency-free C99 and suitable for desktop or MCU use.
 */

#ifndef MINIAHRS_H
#define MINIAHRS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Protocol constants -------------------------------------------------- */

#define MINIAHRS_HEADER0          0xAA
#define MINIAHRS_HEADER1          0x55
#define MINIAHRS_MSGTYPE_DATA     0x01

/* Data identifier for the miniAHRS calibrated-sensor packet
 * (gyro + accel + mag + time). Confirmed from a captured stream. */
#define MINIAHRS_ID_SENSORS       0x9D

/*
 * Orientation data identifiers.
 *
 * NOTE: the miniAHRS datasheet advertises Heading/Pitch/Roll and Quaternion
 * outputs, but a capture containing those packets was not available, so the
 * data-id values below are taken from the Inertial Labs command set (OPVT =
 * 0x52, QPVT = 0x56 in the MRU ICD) and should be confirmed against your
 * unit's ICD. They are #defines so you can adjust them in one place.
 *
 * The orientation packet layouts assumed here are the classic Inertial Labs
 * fixed-point encodings:
 *   OPVT (angles):  Heading word*100, Pitch sword*100, Roll sword*100
 *   QPVT (quat):    Lk0..Lk3 sword*10000
 * Trailing fields (sensors, status, time, ...) are ignored by the decoders.
 */
#define MINIAHRS_ID_OPVT          0x52   /* orientation as Euler angles */
#define MINIAHRS_ID_QPVT          0x56   /* orientation as quaternion   */

/* Payload length (bytes) of the 0x9D sensor packet. */
#define MINIAHRS_SENSORS_PAYLOAD  38

/* Full on-wire frame length of the 0x9D packet:
 * header(2) + type(1) + id(1) + len(2) + payload(38) + checksum(2). */
#define MINIAHRS_SENSORS_FRAME    (6 + MINIAHRS_SENSORS_PAYLOAD + 2)  /* 46 */

/* Largest frame we will ever buffer while resynchronising. */
#define MINIAHRS_MAX_FRAME        256

/* ---- Fixed-point scale factors ------------------------------------------- */

#define MINIAHRS_GYRO_SCALE       1.0e5     /* raw -> deg/s : value / 1e5   */
#define MINIAHRS_ACC_SCALE        1.0e6     /* raw -> g     : value / 1e6   */
#define MINIAHRS_ANGLE_SCALE      100.0     /* raw -> deg   : value / 100   */
#define MINIAHRS_QUAT_SCALE       10000.0   /* raw -> unit  : value / 10000 */

/* ---- Decoded data -------------------------------------------------------- */

typedef struct {
    /* Calibrated angular rate, degrees/second. */
    double gyro_x, gyro_y, gyro_z;
    /* Calibrated specific force, g (1 g = 9.8106 m/s^2). */
    double acc_x, acc_y, acc_z;
    /* Magnetometer components, raw sensor units. */
    int16_t mag_x, mag_y, mag_z;
    /* Unit status / flags word. */
    uint16_t status;
    /* Device timestamp / sample counter. */
    uint32_t timestamp;
    /* Raw temperature code. */
    uint16_t temperature_raw;
} miniahrs_sensors_t;

/* Orientation as Euler angles (OPVT-style packet). */
typedef struct {
    double heading;   /* 0..360 degrees           */
    double pitch;     /* +-90 degrees             */
    double roll;      /* +-180 degrees            */
} miniahrs_orientation_t;

/* Orientation as a unit quaternion (QPVT-style packet).
 * q0 is the scalar part; (q1,q2,q3) the vector part. */
typedef struct {
    double q0, q1, q2, q3;
} miniahrs_quaternion_t;

/* Stateful gyro+accelerometer attitude estimate.
 * Yaw is relative to the initial value and will drift without magnetometer
 * or another external heading reference. */
typedef struct {
    double yaw;       /* degrees, 0..360 */
    int initialized;
} miniahrs_gyro_acc_state_t;

/* ---- Return codes -------------------------------------------------------- */

typedef enum {
    MINIAHRS_OK = 0,         /* a complete, valid packet was produced  */
    MINIAHRS_NEED_MORE,      /* more bytes required before a packet     */
    MINIAHRS_BAD_CHECKSUM,   /* a framed packet failed checksum         */
    MINIAHRS_UNKNOWN_ID      /* valid frame but unhandled data id       */
} miniahrs_status_t;

/* ---- Streaming parser ----------------------------------------------------
 *
 * Feed bytes from the serial port as they arrive; the parser handles
 * resynchronisation, framing and checksum validation internally.
 */

typedef struct {
    uint8_t  buf[MINIAHRS_MAX_FRAME];
    size_t   len;            /* bytes currently held in buf            */
    uint8_t  last_id;        /* data id of the most recent frame       */
} miniahrs_parser_t;

/* Per-packet callbacks. Any of them may be NULL. */
typedef void (*miniahrs_sensors_cb)(const miniahrs_sensors_t *s, void *user);
typedef void (*miniahrs_orientation_cb)(const miniahrs_orientation_t *o, void *user);
typedef void (*miniahrs_quaternion_cb)(const miniahrs_quaternion_t *q, void *user);

/* Bundle of callbacks passed to the parser. Unset entries are ignored. */
typedef struct {
    miniahrs_sensors_cb     on_sensors;
    miniahrs_orientation_cb on_orientation;
    miniahrs_quaternion_cb  on_quaternion;
    void                   *user;
} miniahrs_callbacks_t;

/* Reset a parser to the empty state. Always call before first use. */
void miniahrs_parser_init(miniahrs_parser_t *p);

/*
 * Push a block of received bytes into the parser. For each complete,
 * checksum-valid 0x9D sensor packet found, 'cb' is invoked (if non-NULL).
 * Returns the number of sensor packets decoded from this call.
 *
 * This is the simple, backward-compatible entry point that reports only
 * sensor (0x9D) packets. Use miniahrs_parser_push_ex() to additionally
 * receive orientation / quaternion packets.
 */
size_t miniahrs_parser_push(miniahrs_parser_t *p,
                            const uint8_t *data, size_t n,
                            miniahrs_sensors_cb cb, void *user);

/*
 * Push a block of bytes and dispatch every decoded packet to the matching
 * callback in 'cbs' (sensors, orientation, and/or quaternion).
 * Returns the total number of packets dispatched from this call.
 */
size_t miniahrs_parser_push_ex(miniahrs_parser_t *p,
                               const uint8_t *data, size_t n,
                               const miniahrs_callbacks_t *cbs);

/* ---- Low-level helpers (single-frame) ------------------------------------ */

/*
 * Validate and decode exactly one 0x9D frame that begins at frame[0].
 * 'len' is the number of bytes available at 'frame'.
 *   - returns MINIAHRS_NEED_MORE  if len < MINIAHRS_SENSORS_FRAME
 *   - returns MINIAHRS_BAD_CHECKSUM on checksum mismatch
 *   - returns MINIAHRS_UNKNOWN_ID  if the id is not 0x9D
 *   - returns MINIAHRS_OK and fills *out on success
 */
miniahrs_status_t miniahrs_decode_sensors(const uint8_t *frame, size_t len,
                                          miniahrs_sensors_t *out);

/*
 * Decode one OPVT-style orientation frame (data id MINIAHRS_ID_OPVT).
 * Reads Heading (word*100), Pitch and Roll (sword*100) from the start of
 * the payload. Validates header, length field and checksum.
 */
miniahrs_status_t miniahrs_decode_orientation(const uint8_t *frame, size_t len,
                                              miniahrs_orientation_t *out);

/*
 * Decode one QPVT-style quaternion frame (data id MINIAHRS_ID_QPVT).
 * Reads Lk0..Lk3 (sword*10000) from the start of the payload.
 * Validates header, length field and checksum.
 */
miniahrs_status_t miniahrs_decode_quaternion(const uint8_t *frame, size_t len,
                                             miniahrs_quaternion_t *out);

/* Convert a decoded quaternion to Euler angles (degrees), per ICD App. D. */
void miniahrs_quat_to_euler(const miniahrs_quaternion_t *q,
                            miniahrs_orientation_t *out);

/*
 * Estimate orientation (heading/yaw, pitch, roll) directly from the raw
 * accelerometer and magnetometer readings of a sensor packet.
 *
 * This is a static, tilt-compensated solution (the classic accel+mag
 * AHRS formula): pitch and roll come from gravity sensed by the
 * accelerometers; heading comes from the magnetometers after de-tilting.
 * It needs no gyro integration and works while the unit is roughly still
 * or under low dynamics. For full dynamic accuracy use the device's own
 * fused orientation packet (OPVT/QPVT) instead.
 *
 * 'declination_deg' is the local magnetic declination to add to the
 * magnetic heading to obtain true heading (pass 0 for magnetic heading).
 *
 * Output: heading 0..360, pitch +-90, roll +-180 (degrees).
 * Returns 0 on success, non-zero if the accel vector is too small to use.
 */
int miniahrs_orientation_from_sensors(const miniahrs_sensors_t *s,
                                      double declination_deg,
                                      miniahrs_orientation_t *out);

/*
 * Estimate orientation from gyro + accelerometer only.
 *
 * Pitch and roll are measured from gravity using the accelerometer. Yaw is
 * integrated from gyro_z using dt_sec, so it is only a relative yaw and will
 * slowly drift because accelerometers cannot observe heading/yaw.
 *
 * Set state->initialized = 0 before the first sample. Use initial_yaw_deg to
 * choose the starting yaw, commonly 0. Returns 0 on success, non-zero if the
 * accel vector is too small or dt_sec is negative.
 */
int miniahrs_orientation_from_gyro_acc(const miniahrs_sensors_t *s,
                                       double dt_sec,
                                       double initial_yaw_deg,
                                       miniahrs_gyro_acc_state_t *state,
                                       miniahrs_orientation_t *out);

/* Compute the protocol checksum over a framed message (header excluded). */
uint16_t miniahrs_checksum(const uint8_t *frame, size_t frame_len);

#ifdef __cplusplus
}
#endif

#endif /* MINIAHRS_H */
