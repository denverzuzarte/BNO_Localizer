#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define SDA 2
#define SCL 3
#define LED 25
uint32_t const baudrate = 100000;
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

	//printf("\n\nReading Header...\n");
	//printf("least significant byte of length = %d\n", *length_lsb);
	//printf("most significant byte of the length = %d\n", (*length_msb & ~(1<<7)));
	//printf("channel of data stream for SHTP = %02X\n", *channel);
	//printf("sequence number of data = %d\n", *seq);	
	//printf("payload length = %d", payload_length);

	return payload_length;
}

int shtp_read_packet(uint8_t *payload, uint16_t payload_len)
{
	int test = i2c_read_blocking(i2c1, address_BNO, payload, payload_len, false);
	//printf("\narr : \n");
	//printf("payload[0]=%02X payload[1]=%02X payload[2]=%02X payload[3]=%02X\n",
       	//payload[0], payload[1], payload[2], payload[3]);	
	return test;
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
	printf("hello world\n");
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

	printf("\n--------------- booting 1 complete --------------\n");

	// restart the BNO
        i2c_write_blocking(i2c1, address_BNO, reset_pkt, 5, false);
		
	booting = 0;
	reset_pkt[3]++ ;	

	// drain loop
	sleep_ms(300);

        while(!booting)
        {
                payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                shtp_read_packet(payload, payload_len);
                if(payload[4] == 1 && payload[2] == 1)
                        booting++;
        }

        printf("\n--------------- booting 2 complete --------------\n");

	// reading loop	
	for(int i = 0; i <= 3; i++) 
	{
		payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
		result = shtp_read_packet(payload, payload_len);
		for(uint8_t i = 0; i < 4; i++)
		{
			
			printf("payload[%d] : %02X ",i , payload[i]);
		}
		printf("\n");
		for(uint8_t i = 4; i < 9; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
		printf("\nread packet result : %d\n", result);
	}

	printf("\n--------------- requesting accelerometer data --------------\n");
	
	// sending SHTP request for accelerometer data
	
	uint8_t req_accel_pkt[21] = {0x15, 0x00, 0x02, 0x02,
				0xFD, 0x01, 0x00, 0x00, 0x00,
				0x88, 0x13, 0x00, 0x00,         // 5000 micro seconds delay for reports
				0x00, 0x00, 0x00, 0x00,		// 0 batch delay
				0x00, 0x00, 0x00, 0x00};
	
	i2c_write_blocking(i2c1, address_BNO, req_accel_pkt, 21, false);
	
	// reading loop
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	float acc_sq;
	float acc;
        for(int i = 0; i <= 100; i++)
        {
                sleep_ms(5);
                payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                result = shtp_read_packet(payload, payload_len);
                for(uint8_t i = 0; i < 4; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
		printf("\n");
                for(uint8_t i = 4; i < 9; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\n");
                for(uint8_t i = 9; i < 17; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\n");
		acc_x = payload[14] << 8 | payload[13];
		acc_y = payload[16] << 8 | payload[15];
		acc_z = payload[18] << 8 | payload[17];
		acc_sq = (acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)/(256.0f*256.0f);
		acc = sqrt(acc_sq);
		printf("accel x = %.3f m/s2\n", acc_x / 256.0f);
		printf("accel y = %.3f m/s2\n", acc_y / 256.0f);
		printf("accel z = %.3f m/s2\n", acc_z / 256.0f);
		printf("accel = %.3f", acc);
		printf("\nread packet result : %d\n", result);
        }

	printf("\n -------------------- acc complete ----------------------\n");

	// restart the BNO
        reset_pkt[3] = 0x01;
        i2c_write_blocking(i2c1, address_BNO, reset_pkt, 5, false);

        booting = 0;
        reset_pkt[3] += 2 ;

        // drain loop
        sleep_ms(300);

        while(!booting)
        {
                payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                shtp_read_packet(payload, payload_len);
                if(payload[4] == 1 && payload[2] == 1)
                        booting++;
        }

        printf("\n--------------- booting 3 complete --------------\n");

        // reading loop 
        for(int i = 0; i <= 3; i++)
        {
                payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                result = shtp_read_packet(payload, payload_len);
                for(uint8_t i = 0; i < 4; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\n");
                for(uint8_t i = 4; i < 9; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\nread packet result : %d\n", result);
        }

        printf("\n--------------- requesting gyrelerometer data --------------\n");

        // sending SHTP request for gyrelerometer data

        uint8_t req_gyrel_pkt[21] =  {0x15, 0x00, 0x02, 0x02,
                                0xFD, 0x02, 0x00, 0x00, 0x00,
                                0x88, 0x13, 0x00, 0x00,         // 5000 micro seconds delay for reports
                                0x00, 0x00, 0x00, 0x00,         // 0 batch delay
                                0x00, 0x00, 0x00, 0x00};

        i2c_write_blocking(i2c1, address_BNO, req_gyrel_pkt, 21, false);

        // reading loop
        int16_t gyr_x;
        int16_t gyr_y;
        int16_t gyr_z;
        float gyr_sq;
        float gyr;
        for(int i = 0; i <= 100; i++)
        {
                sleep_ms(5);
                payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                result = shtp_read_packet(payload, payload_len);
                for(uint8_t i = 0; i < 4; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\n");
                for(uint8_t i = 4; i < 9; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\n");
                for(uint8_t i = 9; i < 17; i++)
                {

                        printf("payload[%d] : %02X ",i , payload[i]);
                }
                printf("\n");
                gyr_x = payload[14] << 8 | payload[13];
                gyr_y = payload[16] << 8 | payload[15];
                gyr_z = payload[18] << 8 | payload[17];
                gyr_sq = (gyr_x*gyr_x + gyr_y*gyr_y + gyr_z*gyr_z)/(4.0f*256.0f*256.0f);
                gyr = sqrt(gyr_sq);
                printf("gyr x = %.3f m/s2\n", gyr_x / 512.0f);
                printf("gyr y = %.3f m/s2\n", gyr_y / 512.0f);
                printf("gyr z = %.3f m/s2\n", gyr_z / 512.0f);
                printf("gyr = %.3f", gyr);
                printf("\nread packet result : %d\n", result);
        }
	while(1)
	{
		sleep_ms(5);
		payload_len = shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
                result = shtp_read_packet(payload, payload_len);
		printf("\n");
                gyr_x = payload[14] << 8 | payload[13];
                gyr_y = payload[16] << 8 | payload[15];
                gyr_z = payload[18] << 8 | payload[17];
                gyr_sq = (gyr_x*gyr_x + gyr_y*gyr_y + gyr_z*gyr_z)/(4.0f*256.0f*256.0f);
                gyr = sqrt(gyr_sq);
                printf("gyr x = %.3f m/s2   ", gyr_x / 512.0f);
                printf("gyr y = %.3f m/s2   ", gyr_y / 512.0f);
                printf("gyr z = %.3f m/s2   ", gyr_z / 512.0f);
		printf("gyr = %.3f\n", gyr);
	}
}
