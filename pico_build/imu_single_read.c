#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SDA       2
#define SCL       3
#define LED       25
#define BUF_SIZE  64        // largest BNO085 report is ~22 bytes, 64 is safe

uint32_t const baudrate    = 400000;
uint8_t        address_BNO = 0x4A;

// ── serial framing ──────────────────────────────────────────────────────────
void packnpost_float(float *bytes, uint32_t *tmp)
{
    memcpy(tmp, bytes, 0x04);
    *tmp >>= 4;
    putchar_raw((*tmp >> 21) & 0x7f);
    putchar_raw((*tmp >> 14) & 0x7f);
    putchar_raw((*tmp >>  7) & 0x7f);
    putchar_raw((*tmp >>  0) & 0x7f);
}

// ── single-read SHTP ────────────────────────────────────────────────────────
//
//  One I2C transaction gives us the full packet:
//  buf[0]   = length LSB
//  buf[1]   = length MSB  (bit 15 = continuation flag)
//  buf[2]   = channel
//  buf[3]   = sequence
//  buf[4]   = 0xFB  (base-timestamp discriminator)
//  buf[5-8] = 32-bit timestamp (µs, BNO085 timebase)
//  buf[9]   = sensor report ID
//  buf[10]  = report sequence
//  buf[11]  = status / accuracy
//  buf[12]  = delay MSB
//  buf[13-14] = X  int16
//  buf[15-16] = Y  int16
//  buf[17-18] = Z  int16
//  buf[19-20] = W  int16  (rotation vector only)
//
//  Returns parsed packet length (0 = nothing ready)
uint16_t shtp_read(uint8_t *buf)
{
    int rc = i2c_read_blocking(i2c1, address_BNO, buf, BUF_SIZE, false);
    if (rc < 4) return 0;

    uint16_t len = ((buf[1] & 0x7F) << 8) | buf[0];
    return len;
}

int main()
{
    sleep_ms(1000);
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    stdio_init_all();
    sleep_ms(3000);
    gpio_put(LED, 1);

    i2c_init(i2c1, baudrate);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);

    uint8_t  buf[BUF_SIZE];
    uint16_t pkt_len;

    // ── reset ────────────────────────────────────────────────────────────────
    uint8_t reset_pkt[5] = {0x05, 0x00, 0x01, 0x00, 0x01};
    i2c_write_blocking(i2c1, address_BNO, reset_pkt, 5, false);
    sleep_ms(300);

    // ── wait for boot advertisement (channel 1, report 1) ───────────────────
    // original used payload[4] and payload[2] — in single-read buf those are
    // buf[4] (first cargo byte) and buf[2] (channel). same indices, keep them.
    while (1)
    {
        pkt_len = shtp_read(buf);
        if (pkt_len > 4 && buf[2] == 1 && buf[4] == 1)
            break;
    }

    // ── enable sensor reports ────────────────────────────────────────────────
    // gyro uncal  0x07 @ 200 Hz  (period = 5000 µs = 0x00001388)
    uint8_t req_gyro[21] = {0x15,0x00,0x02,0x02,
                             0xFD,0x07,0x00,0x00,0x00,
                             0x88,0x13,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    // linear accel 0x04 @ 100 Hz (period = 10000 µs = 0x00002710)
    uint8_t req_lin[21]  = {0x15,0x00,0x02,0x03,
                             0xFD,0x04,0x00,0x00,0x00,
                             0x10,0x27,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    // rotation vector 0x05 @ 50 Hz (period = 20000 µs = 0x00004E20)
    uint8_t req_rot[21]  = {0x15,0x00,0x02,0x04,
                             0xFD,0x05,0x00,0x00,0x00,
                             0x20,0x4E,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//    // accelerometer 0x01 @ 25 Hz (period = 40000 µs = 0x00009C40)
//    uint8_t req_acc[21]  = {0x15,0x00,0x02,0x05,
//                             0xFD,0x01,0x00,0x00,0x00,
//                             0x40,0x9C,0x00,0x00,
//                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    i2c_write_blocking(i2c1, address_BNO, req_gyro, 21, false); sleep_ms(100);
    i2c_write_blocking(i2c1, address_BNO, req_lin,  21, false); sleep_ms(100);
    i2c_write_blocking(i2c1, address_BNO, req_rot,  21, false); sleep_ms(100);
//    i2c_write_blocking(i2c1, address_BNO, req_acc,  21, false); sleep_ms(100);

    // ── main loop ────────────────────────────────────────────────────────────
    int16_t  x_raw, y_raw, z_raw, w_raw;
    float    xf, yf, zf, wf;
    uint32_t tmp;

    for (;;)
    {
        pkt_len = shtp_read(buf);

        if (pkt_len == 0) continue;

        // channel 3 = sensor hub input; anything else is housekeeping, ignore
        if (buf[2] != 3) continue;

        // buf[4] should be 0xFB (base timestamp ref); real report ID is at [9]
        uint8_t report_id = buf[9];

        switch (report_id)
        {
            case 0x07:  // gyro uncalibrated  (Q9 → ÷512 rad/s)
                x_raw = (int16_t)(buf[14] << 8 | buf[13]);
                y_raw = (int16_t)(buf[16] << 8 | buf[15]);
                z_raw = (int16_t)(buf[18] << 8 | buf[17]);
                xf = x_raw / 512.0f;
                yf = y_raw / 512.0f;
                zf = z_raw / 512.0f;
                putchar_raw(0xF1); putchar_raw(0x01); putchar_raw(0x0C);
                packnpost_float(&xf, &tmp);
                packnpost_float(&yf, &tmp);
                packnpost_float(&zf, &tmp);
                putchar_raw(0xF2); stdio_flush();
                break;

            case 0x04:  // linear acceleration (Q8 → ÷256 m/s²)
                x_raw = (int16_t)(buf[14] << 8 | buf[13]);
                y_raw = (int16_t)(buf[16] << 8 | buf[15]);
                z_raw = (int16_t)(buf[18] << 8 | buf[17]);
                xf = x_raw / 256.0f;
                yf = y_raw / 256.0f;
                zf = z_raw / 256.0f;
                putchar_raw(0xF1); putchar_raw(0x02); putchar_raw(0x0C);
                packnpost_float(&xf, &tmp);
                packnpost_float(&yf, &tmp);
                packnpost_float(&zf, &tmp);
                putchar_raw(0xF2); stdio_flush();
                break;

            case 0x05:  // rotation vector    (Q14 → ÷16384)
                x_raw = (int16_t)(buf[14] << 8 | buf[13]);
                y_raw = (int16_t)(buf[16] << 8 | buf[15]);
                z_raw = (int16_t)(buf[18] << 8 | buf[17]);
                w_raw = (int16_t)(buf[20] << 8 | buf[19]);
                xf = x_raw / 16384.0f;
                yf = y_raw / 16384.0f;
                zf = z_raw / 16384.0f;
                wf = w_raw / 16384.0f;
                putchar_raw(0xF1); putchar_raw(0x03); putchar_raw(0x10);
                packnpost_float(&xf, &tmp);
                packnpost_float(&yf, &tmp);
                packnpost_float(&zf, &tmp);
                packnpost_float(&wf, &tmp);
                putchar_raw(0xF2); stdio_flush();
                break;

//            case 0x01:  // accelerometer      (Q8 → ÷256 m/s²)
//                x_raw = (int16_t)(buf[14] << 8 | buf[13]);
//                y_raw = (int16_t)(buf[16] << 8 | buf[15]);
//                z_raw = (int16_t)(buf[18] << 8 | buf[17]);
//                xf = x_raw / 256.0f;
//                yf = y_raw / 256.0f;
//                zf = z_raw / 256.0f;
//                putchar_raw(0xF1); putchar_raw(0x04); putchar_raw(0x0C);
//                packnpost_float(&xf, &tmp);
//                packnpost_float(&yf, &tmp);
//                packnpost_float(&zf, &tmp);
//                putchar_raw(0xF2); stdio_flush();
//                break;

            default:
                break;  // ignore calibration status, FRS, etc.
        }
    }
}
