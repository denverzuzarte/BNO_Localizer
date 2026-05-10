#include "pico/stdlib.h"

#define PIN 25

int main()
{
	gpio_init(PIN);
	gpio_set_dir(PIN, GPIO_OUT);
	while(1)
	{
		gpio_put(PIN, 1);
		sleep_ms(500);
		gpio_put(PIN,0);
		sleep_ms(500);
	}
}
