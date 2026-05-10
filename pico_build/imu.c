#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SDA 2
#define SCL 3
#define LED 25
uint32_t const baudrate = 400000;
uint8_t address_BNO = 0x4A;

uint16_t shtp_read_header(uint8_t *length_lsb, uint8_t *length_msb, uint8_t *channel, uint8_t *seq)
{
	uint8_t buf[4] = {*length_lsb, *length_msb, *channel, *seq};
	i2c_read_blocking(i2c1, address_BNO, buf, 4, false);
	*length_lsb = buf[0];
	*length_msb = buf[1];
	*channel    = buf[2];
	*seq        = buf[3];

	uint16_t payload_length = (*length_msb & ~(1<<7))<<8 | *length_lsb ;
	return payload_length;
}

int shtp_read_packet(uint8_t *payload, uint16_t payload_len)
{
	int test = i2c_read_blocking(i2c1, address_BNO, payload, payload_len, false);
	return test;
}

void packnpost_float(float *bytes,uint32_t *tmp)
{
	memcpy(tmp, bytes, 0x04);
        *tmp >>= 4;
        uint8_t x_0 = (*tmp >> 21) & (0x7f);
	uint8_t x_1 = (*tmp >> 14) & (0x7f);
	uint8_t x_2 = (*tmp >>  7) & (0x7f);
	uint8_t x_3 = (*tmp >>  0) & (0x7f);

	putchar_raw(x_0); putchar_raw(x_1); putchar_raw(x_2); putchar_raw(x_3);
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
	uint8_t header[4];
	uint8_t payload[300];
	uint16_t payload_len;
	int result;

	// restart the BNO
	uint8_t reset_pkt[5] = {0x05, 0x00, 0x01, 0x00, 0x01};
	i2c_write_blocking(i2c1, address_BNO, reset_pkt, 5, false);

	uint8_t booting = 0;

        // drain loop
        sleep_ms(300);
	while(!booting)
	{
		payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
		shtp_read_packet(payload, payload_len);
		if(payload[4] == 1 && payload[2] == 1)
			booting++;
	}

	// SHTP feature requests: gyro_uncal@200Hz, lin_acc@100Hz, rot_vec@50Hz

	uint8_t req_gyro_pkt[21] = {0x15, 0x00, 0x02, 0x02,
				0xFD, 0x07, 0x00, 0x00, 0x00,
				0x88, 0x13, 0x00, 0x00,         // 5000 us = 200 Hz
				0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00};

	uint8_t req_lin_acc_pkt[21] = {0x15, 0x00, 0x02, 0x03,
				0xFD, 0x04, 0x00, 0x00, 0x00,
				0x10, 0x27, 0x00, 0x00,         // 10000 us = 100 Hz
				0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00};

	uint8_t req_rot_vec_pkt[21] = {0x15, 0x00, 0x02, 0x04,
				0xFD, 0x05, 0x00, 0x00, 0x00,
				0x20, 0x4E, 0x00, 0x00,         // 20000 us = 50 Hz
				0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00};

//	uint8_t req_accel_pkt[21] = {0x15, 0x00, 0x02, 0x05,
//				0xFD, 0x01, 0x00, 0x00, 0x00,
//				0x40, 0x9C, 0x00, 0x00,         // 40000 us = 25 Hz
//				0x00, 0x00, 0x00, 0x00,
//				0x00, 0x00, 0x00, 0x00};

	i2c_write_blocking(i2c1, address_BNO, req_gyro_pkt, 21, false);
	sleep_ms(100);
	i2c_write_blocking(i2c1, address_BNO, req_lin_acc_pkt, 21, false);
	sleep_ms(100);
	i2c_write_blocking(i2c1, address_BNO, req_rot_vec_pkt, 21, false);
//	i2c_write_blocking(i2c1, address_BNO, req_accel_pkt, 21, false);

	// reading loop
	int16_t x_raw, y_raw, z_raw, w_raw;
	float xf, yf, zf, wf;
	uint32_t tmp;
	for(int i = 0; i >= 0; i++)
	{
		payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
		if(payload_len == 0) continue;
		result = shtp_read_packet(payload, payload_len);

		if(payload[9] == 0x07)  // gyroscope uncalibrated, Q9 -> /512 rad/s, msg 0x01
		{
			putchar_raw(0xF1); putchar_raw(0x01); putchar_raw(0x0c);
			x_raw = payload[14] << 8 | payload[13];
			y_raw = payload[16] << 8 | payload[15];
			z_raw = payload[18] << 8 | payload[17];
			xf = x_raw / 512.0f; yf = y_raw / 512.0f; zf = z_raw / 512.0f;
			packnpost_float(&xf, &tmp);
			packnpost_float(&yf, &tmp);
			packnpost_float(&zf, &tmp);
			putchar_raw(0xF2); stdio_flush();
		}
		if(payload[9] == 0x04)  // linear acceleration, Q8 -> /256 m/s^2, msg 0x02
		{
			putchar_raw(0xF1); putchar_raw(0x02); putchar_raw(0x0c);
			x_raw = payload[14] << 8 | payload[13];
			y_raw = payload[16] << 8 | payload[15];
			z_raw = payload[18] << 8 | payload[17];
			xf = x_raw / 256.0f; yf = y_raw / 256.0f; zf = z_raw / 256.0f;
			packnpost_float(&xf, &tmp);
			packnpost_float(&yf, &tmp);
			packnpost_float(&zf, &tmp);
			putchar_raw(0xF2); stdio_flush();
		}
		if(payload[9] == 0x05)  // rotation vector, Q14 -> /16384, msg 0x03
		{
			putchar_raw(0xF1); putchar_raw(0x03); putchar_raw(0x10);
			x_raw = payload[14] << 8 | payload[13];
			y_raw = payload[16] << 8 | payload[15];
			z_raw = payload[18] << 8 | payload[17];
			w_raw = payload[20] << 8 | payload[19];
			xf = x_raw / 16384.0f; yf = y_raw / 16384.0f;
			zf = z_raw / 16384.0f; wf = w_raw / 16384.0f;
			packnpost_float(&xf, &tmp);
			packnpost_float(&yf, &tmp);
			packnpost_float(&zf, &tmp);
			packnpost_float(&wf, &tmp);
			putchar_raw(0xF2); stdio_flush();
		}
//		if(payload[9] == 0x01)  // accelerometer, Q8 -> /256 m/s^2, msg 0x04
//		{
//			putchar_raw(0xF1); putchar_raw(0x04); putchar_raw(0x0c);
//			x_raw = payload[14] << 8 | payload[13];
//			y_raw = payload[16] << 8 | payload[15];
//			z_raw = payload[18] << 8 | payload[17];
//			xf = x_raw / 256.0f; yf = y_raw / 256.0f; zf = z_raw / 256.0f;
//			packnpost_float(&xf, &tmp);
//			packnpost_float(&yf, &tmp);
//			packnpost_float(&zf, &tmp);
//			putchar_raw(0xF2); stdio_flush();
//		}
	}
}
