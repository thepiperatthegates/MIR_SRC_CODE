#ifndef SENSOR_FRAME_H
#define SENSOR_FRAME_H
/* sensor_frame.h — decode / reduce the sensor frame.
 * No malloc, no stdio, endian-safe. Frame layout:
 *   [0..7]  header: two big-endian uint32 = { channel_count, samples_per_sec }
 *   [8..]   channel_count blocks, each samples_per_sec * int16 (big-endian,
 *           signed), row-major: channel 0 in full, then channel 1, ...
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>   /* memcpy */

#define SENSOR_HEADER_BYTES 8u
#define BYTE_SIZE_UINT6 (2U2)

enum {                    /* channel indices in the FULL frame */
    CH_DAC1   = 2, CH_DAC2   = 3,
    CH_HALL_A = 4, CH_HALL_B = 5,
    CH_COIL_A = 6, CH_COIL_B = 7
};

enum {                    /* channel indices in the REDUCED (4-ch) frame */
    R_HALL_A = 0, R_HALL_B = 1, R_COIL_A = 2, R_COIL_B = 3
};

typedef struct {
    const uint8_t *base;  /* points at the frame buffer (not copied) */
    uint32_t channel_count;
    uint32_t samples_per_channel;
} sensor_frame_t;

#define BYTE_UINT16T 2U

/* Read a big-endian uint32 from a byte buffer. */
uint32_t sensor_frame_read_be32(const uint8_t *p);
/* Write a uint32 to a byte buffer as big-endian. */
void sensor_frame_write_be32(uint8_t *p, uint32_t v);

/* Validate + bind to a whole frame buffer. Returns 0 on success. */
int sensor_frame_init(sensor_frame_t *f, const uint8_t *buf, uint32_t len);

/* One sample as signed int16 (big-endian -> host), zero-copy. */
int16_t sensor_sample(const sensor_frame_t *f, uint32_t ch, uint32_t i);

/* Reduce the full frame to only HALL_A/B, COIL_A/B, SAME on-disk format.
 * Output = 8-byte header (channel_count=4, same samples_per_channel) + the 4
 * channel blocks copied verbatim (still big-endian). Channel order in
 * output: HALL_A, HALL_B, COIL_A, COIL_B  ->  use the R_* indices to read
 * it back.
 * Returns output length in bytes, or 0 on error.
 * 'out' must hold at least SENSOR_HEADER_BYTES + 4*samples_per_channel*2
 * bytes. */
uint32_t sensor_reduce_to_4(const uint8_t *in, uint32_t in_len,
                             uint8_t *out, uint32_t out_cap);

#endif /* SENSOR_FRAME_H */
