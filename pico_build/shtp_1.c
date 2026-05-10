#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <stdio.h>

#define SDA 2
#define SCL 3
#define LED 25
uint32_t const baudrate = 100000;
uint8_t address_BNO = 0x4A;

void shtp_read_header(uint8_t *length_lsb, uint8_t *length_msb, uint8_t *channel, uint8_t *seq)
{
	uint8_t buf[4] = {*length_lsb, *length_msb, *channel, *seq};
	i2c_read_blocking(i2c1, address_BNO, buf, 4, false);
	*length_lsb = buf[0];
	*length_msb = buf[1];
	*channel    = buf[2];
	*seq        = buf[3];

	printf("Reading Header...\n");
	printf("least significant byte of length = %d\n", *length_lsb);
	printf("most significant byte of the length = %d\n", (*length_msb & ~(1<<7)));
	printf("channel of data stream for SHTP = %02X\n", *channel);
	printf("sequence number of data = %d\n", *seq);	
}

int shtp_read_packet(uint8_t *header, uint8_t *payload, uint16_t *payload_len)
{
	shtp_read_header(&(header[0]), &(header[1]), &(header[2]), &(header[3]));
	*payload_len = (((header[1] & ~(1<<7)) << 8) | header[0]) - 4 ;
	int test = i2c_read_blocking(i2c1, address_BNO, payload, *payload_len, false);
	printf("\narr : \n");
	if(test >= 0)
	return 0;
	return -1;
}


int main()
{
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
	uint8_t arr[4];
	//uint8_t reading_length;
	uint8_t payload[300];
	uint16_t payload_len;
	int result;
	while(1)
	{
		sleep_ms(300);
		result = shtp_read_packet(arr, payload, &payload_len);
		//printf("\nreading_length = %d\narr = ", reading_length);
		for(uint8_t i = 0; i < (sizeof(arr)/sizeof(arr[0])); i++)
		{
			
			printf("%d : %02X ",i , arr[i]);
		}
		printf("\nread packet result : %d\n", result);
	}
}
