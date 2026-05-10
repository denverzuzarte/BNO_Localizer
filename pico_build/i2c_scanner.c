#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

#define SDA 2
#define SCL 3
#define LED 25
uint32_t const baudrate = 100000;

int main()
{
	gpio_init(LED);
	gpio_set_dir(LED, GPIO_OUT);
	stdio_init_all();
	sleep_ms(2000);
	gpio_put(LED, 1);
	i2c_init(i2c1, baudrate);
	gpio_set_function(SCL, GPIO_FUNC_I2C);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_pull_up(SDA);
	gpio_pull_up(SCL);
	uint8_t address = 0x08;
	uint8_t BNO_addr;
	int device_presence;
	printf("hello world");
	while(1)
	{
	    sleep_ms(10);
	    uint8_t dummy = 0;
	    device_presence = i2c_write_blocking(i2c1, address, &dummy, 1, false);
	    sleep_ms(10);
	    if(device_presence >= 0)
	    printf("Scanning address 0x%02X result: %d\n", address, device_presence);
	    address++;
	    if(address == 0x77)
	    {
		address = 0x08;
		printf("check cycle\n");
	    }
	}
}

