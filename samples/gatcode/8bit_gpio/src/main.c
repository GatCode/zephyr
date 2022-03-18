#include <zephyr.h>
#include <drivers/gpio.h>

#include <io_coder.h>

static struct io_coder io_8bit_coder = {0};

void main(void)
{
	int err;
	uint8_t p0_01_to_p0_08_reading, p1_01_to_p1_08_reading;

	err = setup_P0_01_to_P0_08(&io_8bit_coder);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	err = setup_P1_01_to_P1_08(&io_8bit_coder);
	if(err) {
		printk("Error setting up P1.01 - P1.08 (err %d)\n", err);
	}

	while(1) {
		err = read_P0_01_to_P0_08(&io_8bit_coder, &p0_01_to_p0_08_reading);
		if(err) {
			printk("Error reading 8bit value from P1.01 - P1.08 (err %d)\n", err);
		}

		err = read_P1_01_to_P1_08(&io_8bit_coder, &p1_01_to_p1_08_reading);
		if(err) {
			printk("Error reading 8bit value from P1.01 - P1.08 (err %d)\n", err);
		}

		printk("P0 1-8: %u | P1 1-8: %u\n", p0_01_to_p0_08_reading, p1_01_to_p1_08_reading);

		// gpio_port_pins_t mask = BIT(13);
		// gpio_port_set_masked(port0, mask, 1);

		k_msleep(1000);
	}
}
