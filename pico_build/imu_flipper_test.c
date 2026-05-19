#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SDA 2
#define SCL 3
#define LED 25
#define INT 8

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

	uint16_t payload_length = (*length_msb & ~(1<<7))<<8 | *length_lsb;
	return payload_length;
}

int shtp_read_packet(uint8_t *payload, uint16_t payload_len)
{
	return i2c_read_blocking(i2c1, address_BNO, payload, payload_len, false);
}

int main()
{
	sleep_ms(1000);
	gpio_init(LED);
	gpio_set_dir(LED, GPIO_OUT);
	gpio_init(INT);
	gpio_set_dir(INT, GPIO_IN);
	gpio_pull_up(INT);
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

	uint8_t reset_pkt[5] = {0x05, 0x00, 0x01, 0x00, 0x01};
	i2c_write_blocking(i2c1, address_BNO, reset_pkt, 5, false);

	uint8_t booting = 0;
	sleep_ms(300);
	while(!booting)
	{
		payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
		shtp_read_packet(payload, payload_len);
		if(payload[4] == 1 && payload[2] == 1)
			booting++;
	}

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

	i2c_write_blocking(i2c1, address_BNO, req_gyro_pkt, 21, false);    sleep_ms(100);
	i2c_write_blocking(i2c1, address_BNO, req_lin_acc_pkt, 21, false); sleep_ms(100);
	i2c_write_blocking(i2c1, address_BNO, req_rot_vec_pkt, 21, false);

	int16_t x_raw, y_raw, z_raw, w_raw;
	for(int i = 0; i >= 0; i++)
	{
		while(gpio_get(INT)) tight_loop_contents();

		payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
		if(payload_len == 0) continue;
		shtp_read_packet(payload, payload_len);

		switch(payload[9])
		{
			case 0x07:  // gyro uncalibrated, Q9
				x_raw = payload[14] << 8 | payload[13];
				y_raw = payload[16] << 8 | payload[15];
				z_raw = payload[18] << 8 | payload[17];
				printf("G,%d,%d,%d\n", x_raw, y_raw, z_raw);
				stdio_flush();
				break;

			case 0x04:  // linear acceleration, Q8
				x_raw = payload[14] << 8 | payload[13];
				y_raw = payload[16] << 8 | payload[15];
				z_raw = payload[18] << 8 | payload[17];
				printf("A,%d,%d,%d\n", x_raw, y_raw, z_raw);
				stdio_flush();
				break;

			case 0x05:  // rotation vector, Q14
				x_raw = payload[14] << 8 | payload[13];
				y_raw = payload[16] << 8 | payload[15];
				z_raw = payload[18] << 8 | payload[17];
				w_raw = payload[20] << 8 | payload[19];
				printf("R,%d,%d,%d,%d\n", x_raw, y_raw, z_raw, w_raw);
				stdio_flush();
				break;

			default:
				break;
		}
	}
}
