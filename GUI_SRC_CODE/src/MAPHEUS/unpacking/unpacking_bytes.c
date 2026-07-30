#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensor_frame.h"

#define IN_FILE      "20260723_133155,286.bin"   /* Binary input */
#define OUT_FILE_BIN "unpacked_byte.bin"              /* Output: raw reduced frame */
#define OUT_FILE_CSV "unpacked_byte.csv"              /* Output: decoded CSV */
#define CSV_SEP      ";"    


#define SAMPLE_FREQ (1000U)
#define TIME_STAMP  (float) (1.0f/SAMPLE_FREQ)




/* max sizes for this test (8 ch in, 4 ch out, 1000 samples) */
static uint8_t in_buf [SENSOR_HEADER_BYTES + 8 * 1000 * 2];
static uint8_t reduced_buf[SENSOR_HEADER_BYTES + 4 * 1000 * 2];

uint32_t sensor_frame_read_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
}
void sensor_frame_write_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >>  8); p[3] = (uint8_t)v;
}

/* Validate + bind to a whole frame buffer. Returns 0 on success. */
int sensor_frame_init(sensor_frame_t *f, const uint8_t *buf, uint32_t len) {
    if (len < SENSOR_HEADER_BYTES) return -1;
    f->channel_count       = sensor_frame_read_be32(buf);
    f->samples_per_channel = sensor_frame_read_be32(buf + 4);
    if (f->channel_count == 0u || f->samples_per_channel == 0u) return -1;   /* sanity */
    if (len <  + (uint64_t)f->channel_count * f->samples_per_channel * 2u) return -1;
    /* caller must request channel indices < f->channel_count (CH_* for full, R_* for reduced) */
    f->base = buf;
    return 0;
}

/* One sample as signed int16 (big-endian -> host), zero-copy. */
uint16_t sensor_sample(const sensor_frame_t *f, uint32_t ch, uint32_t i) {
    const uint8_t *p = f->base + SENSOR_HEADER_BYTES + ((size_t)ch * f->samples_per_channel + i) * BYTE_UINT16T;
    uint16_t sample_byte = (uint16_t)p[0] << 8 | p[1];
    return sample_byte;
}

uint32_t sensor_reduce_to_4(const uint8_t *in, uint32_t in_len,
                             uint8_t *out, uint32_t out_cap) {
    sensor_frame_t f;

    // Feed in_buf into the struct
    int init_check = sensor_frame_init(&f, in, in_len);
    if(init_check != 0){
        printf("Feeding inner buffer to reduce produced an error!\n\r");
        return 0;
    }

    // See if channels x samples match the length

    if (f.channel_count <= CH_COIL_B)
        return 0;          /* source must contain ch 4..7 */
    
    // calculate the total block from the file
    uint32_t block = f.samples_per_channel * BYTE_UINT16T;

    uint32_t need  = block * 4U + SENSOR_HEADER_BYTES;
    if (out_cap < need)
        return 0;

    //Handle 8 bytes FRAME HEADER
    sensor_frame_write_be32(out,     4u);                     /* new channel count */
    sensor_frame_write_be32(out + 4, f.samples_per_channel);  /* unchanged sample count */

    // What's there to keep
    static const uint32_t keep[4] = { CH_HALL_A, CH_HALL_B, CH_COIL_A, CH_COIL_B };
    uint8_t *w = out + SENSOR_HEADER_BYTES;
    for (int k = 0; k < 4; k++) {
        const uint8_t *src = f.base + SENSOR_HEADER_BYTES + (size_t)keep[k] * block;
        memcpy(w, src, block);      /* verbatim, endianness preserved */
        w += block;
    }
    return need;
}

/// Main program
/* Some basic notes for me 
argc -> arg count: count how many argument i passed onto the main function
argv -> arg vector: array of C strings, first arg/argv[0] is always the program's own PATH
argsv[1], argsv[2] and so on....... is the next real argument passed on to the program
*/
int main(int argc, char *argv[]) {
    /* Sort out the passed argument from Python */

    const char *in_file;
    const char *out_file_bin;
    const char *out_file_csv;

    if (argc > 1)   // input binary file
        in_file = argv[1];
    else
        in_file = IN_FILE;

    if (argc > 2)   //output binary file
        out_file_bin = argv[2];
    else
        out_file_bin = OUT_FILE_BIN;

    if (argc > 3)   //output csv file with headers and separator
        out_file_csv = argv[3];
    else
        out_file_csv = OUT_FILE_CSV;

    /* --- read the whole file into in_buf --- */
    FILE* fptr = fopen(in_file, "rb");

    if(!fptr)
    {
        printf("Failed to open file!");
        return 1;
    }
    
    //Get the size of the binary file 
    uint32_t in_len = (uint32_t)fread(in_buf, 1, sizeof (in_buf), fptr);
    //Close the file
    fclose(fptr);
    
    printf("read %u bytes from %s\n", in_len, in_file);

    /* --- reduce to the 4 channels --- */
    uint32_t rlen = sensor_reduce_to_4(in_buf, in_len, reduced_buf, sizeof reduced_buf);
    if (!rlen) 
    { 
        fprintf(stderr, "not a valid full frame\n"); 
        return 1; 
    }
    
    printf("reduced_buf to %u bytes (dropped internals + both DACs)\n", rlen);

    /* --- decode the reduced_buf buffer --- */

    // struct refer to reduced 4 buffer
    sensor_frame_t r;


    if (sensor_frame_init(&r, reduced_buf, rlen) != 0) 
    { 
        fprintf(stderr, "bad reduced_buf frame\n");
        return 1; 
    }
    printf("reduced_buf frame: channel_count=%u samples_per_channel=%u\n",
           r.channel_count, r.samples_per_channel);

    /* --- write the raw reduced_buf binary file --- */
    fptr = fopen(out_file_bin, "wb");

    if (!fptr) {
        printf("open %s", out_file_bin);
        return 1;
    }

    fwrite(reduced_buf, 1, rlen, fptr);
    fclose(fptr);

    printf("wrote %s (%u bytes)\n", out_file_bin, rlen);

    /* --- Open the CSV file --- */
    fptr = fopen(out_file_csv, "w");

    if (!fptr) {
        printf("open %s", out_file_csv);
        return 1;
    }

    float time_stamp = 0.0f;
    //Write the header
    fprintf(fptr, "i" CSV_SEP "HALL_A" CSV_SEP "HALL_B" CSV_SEP "COIL_A" CSV_SEP "COIL_B\n");
    for (uint32_t i = 0; i < r.samples_per_channel; i++){
        //Write the actual data in CSV
        fprintf(fptr, "%f" CSV_SEP "%u" CSV_SEP "%u" CSV_SEP "%u" CSV_SEP "%u\n", time_stamp,
                sensor_sample(&r, R_HALL_A, i), sensor_sample(&r, R_HALL_B, i),
                sensor_sample(&r, R_COIL_A, i), sensor_sample(&r, R_COIL_B, i));
        time_stamp += TIME_STAMP;
    }
    fclose(fptr);

    printf("wrote %s (%u rows)\n", out_file_csv, r.samples_per_channel);


    //END 

    
    /* --- print a few samples to stdout for a quick sanity check --- */
    printf(" i  HALL_A HALL_B COIL_A COIL_B\n");
    for (uint32_t i = 0; i < 3; i++)
        printf("%3u %6d %6d %6d %6d\n", i,
               sensor_sample(&r, R_HALL_A, i), sensor_sample(&r, R_HALL_B, i),
               sensor_sample(&r, R_COIL_A, i), sensor_sample(&r, R_COIL_B, i));
    printf("... last sample %u: %d %d %d %d\n", r.samples_per_channel - 1,
           sensor_sample(&r, R_HALL_A, r.samples_per_channel - 1), sensor_sample(&r, R_HALL_B, r.samples_per_channel - 1),
           sensor_sample(&r, R_COIL_A, r.samples_per_channel - 1), sensor_sample(&r, R_COIL_B, r.samples_per_channel - 1));
    return 0;
}
