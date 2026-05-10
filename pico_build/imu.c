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

	// sending SHTP request for linear_acceleration and quaternion data
	
	uint8_t req_accel_pkt[21] = {0x15, 0x00, 0x02, 0x02,
				0xFD, 0x04, 0x00, 0x00, 0x00,
				0x88, 0x13, 0x00, 0x00,         // 5000 micro seconds delay for reports
				0x00, 0x00, 0x00, 0x00,		// 0 batch delay
				0x00, 0x00, 0x00, 0x00};

	uint8_t req_quaternion_pkt[] = {0x10, 0x00, 0x02, 0x03,
					0xFD, 0x05,0x00, 0x00, 0x00,
	                                0x88, 0x13, 0x00, 0x00,         // 5000 micro seconds delay for reports
        	                        0x00, 0x00, 0x00, 0x00,         // 0 batch delay
                	                0x00, 0x00, 0x00, 0x00};
	
	i2c_write_blocking(i2c1, address_BNO, req_accel_pkt, 21, false);
	sleep_ms(300);
	i2c_write_blocking(i2c1, address_BNO, req_quaternion_pkt, 21, false);

	// reading loop
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	float accel_x;
	float accel_y;
	float accel_z;
        int16_t quat_x;
        int16_t quat_y;
        int16_t quat_z;
	int16_t quat_w;
        float quatel_x;
        float quatel_y;
        float quatel_z;
	float quatel_w;
	uint16_t accuracy;
	float accu;
	uint32_t tmp;
	uint8_t msg_id = 0x01;
        for(int i = 0; i >= 0; i++)
        {
                payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                result = shtp_read_packet(payload, payload_len);
		if(payload[9] == 0x04)
		{
			putchar_raw(0xF1);putchar_raw(0x02);putchar_raw(0x0c);

			acc_x = payload[14] << 8 | payload[13];
			acc_y = payload[16] << 8 | payload[15];
			acc_z = payload[18] << 8 | payload[17];
			accel_x = (acc_x / 256.0f);
			accel_y = (acc_y / 256.0f);
			accel_z = (acc_z / 256.0f);
			
			packnpost_float(&accel_x, &tmp);	
			packnpost_float(&accel_y, &tmp);
			packnpost_float(&accel_z, &tmp);
			
			putchar_raw(0xF2);
			stdio_flush();
		}
		if(payload[9] == 0x05)
		{
			putchar_raw(0xF1);putchar_raw(0x03);putchar_raw(0x10);

                        quat_x = payload[14] << 8 | payload[13];
                        quat_y = payload[16] << 8 | payload[15];
                        quat_z = payload[18] << 8 | payload[17];
			quat_w = payload[20] << 8 | payload[19];
			accuracy = payload[22] << 8 | payload[21];
                        quatel_x = (quat_x / 4096.0f)/4.0f;
                        quatel_y = (quat_y / 4096.0f)/4.0f;
                        quatel_z = (quat_z / 4096.0f)/4.0f;
			quatel_w = (quat_w / 4096.0f)/4.0f;
			accu = accuracy/ 4096.0f;
                 
                        packnpost_float(&quatel_x, &tmp); 
                        packnpost_float(&quatel_y, &tmp);
                        packnpost_float(&quatel_z, &tmp);
                	packnpost_float(&quatel_w, &tmp);
		        packnpost_float(&accu, &tmp);

                        putchar_raw(0xF2);
                        stdio_flush();
		}
        }
}
